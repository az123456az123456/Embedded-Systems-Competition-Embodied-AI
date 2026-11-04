#include "sensor_bridge.h"
#include "../common/sonar_range.h"
#include "vendor/picojson.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_nav {
namespace {
constexpr double PI=3.14159265358979323846;
double angle(double value){return std::atan2(std::sin(value),std::cos(value));}
}
NavigationInputs::NavigationInputs(Config c,double sign)
    :config_(std::move(c)),yaw_sign_(sign),alignment_(config_.max_measured_turn_rps,.06,
      uint32_t(std::llround(config_.sensor_timeout_s*1000))){
    if(sign!=1&&sign!=-1)throw std::invalid_argument("invalid navigation yaw sign");
}
void NavigationInputs::reset(){
    alignment_.reset();pending_.clear();initial_yaw_.reset();host_offset_lower_.reset();host_anchors_.clear();last_sensor_sequence_.reset();
    established_=notified_=startup_reset_pending_=false;fault_.clear();wait_="waiting_imu_source";
    imu_accepted_=imu_duplicates_=imu_ignored_=sensor_queued_=sensor_ingested_=startup_dropped_=sensor_ignored_=transport_waits_=0;
}
void NavigationInputs::latch(const std::string& reason){if(fault_.empty())fault_=reason;wait_.clear();}
void NavigationInputs::clear_startup_imu(){
    startup_reset_pending_=true;
    alignment_.reset();initial_yaw_.reset();host_offset_lower_.reset();host_anchors_.clear();
    startup_dropped_+=pending_.size();pending_.clear();last_sensor_sequence_.reset();
}
void NavigationInputs::update_imu(const robotnav::GatewayTelemetry& t,double now){
    if(!fault_.empty())return;
    if(!std::isfinite(now)){latch("imu_host_clock_invalid");return;}
    if(!t.available){++transport_waits_;wait_="waiting_imu_transport";return;}
    std::string invalid;
    if(!t.imu_valid)invalid="imu_source_invalid";
    else if(t.imu_calibrating||!t.imu_bias_valid||t.imu_action_pending)invalid="imu_source_reference_unavailable";
    else if(!t.imu_fresh)invalid="imu_source_stale";
    else if(!t.imu_timestamp_valid||!std::isfinite(t.yaw_deg)||t.imu_sequence>0xffffffffULL||
            !std::isfinite(t.imu_sample_age_ms)||t.imu_sample_age_ms<0||
            !t.request_started_ms||t.request_started_ms>t.received_ms||
            double(t.received_ms)/1000.>now+.02)invalid="imu_source_metadata_invalid";
    if(!invalid.empty()){
        if(established_)latch(invalid);else {clear_startup_imu();wait_=invalid;}
        return;
    }
    if(!established_&&host_offset_lower_&&alignment_.latest_unwrapped_ms()&&
       now-(*host_offset_lower_+double(*alignment_.latest_unwrapped_ms())/1000.)>config_.sensor_timeout_s)
        clear_startup_imu();
    const double yaw=t.yaw_deg*PI/180;
    if(!initial_yaw_)initial_yaw_=yaw;
    const auto result=alignment_.push({t.imu_timestamp_ms,uint32_t(t.imu_sequence),angle(yaw-*initial_yaw_)*yaw_sign_});
    if(result==ImuPushStatus::Fault){latch(alignment_.fault_reason());return;}
    if(result==ImuPushStatus::IgnoredOld){++imu_ignored_;return;}
    if(result==ImuPushStatus::Accepted)++imu_accepted_;else ++imu_duplicates_;
    const auto source_time=alignment_.latest_unwrapped_ms();
    // request_started <= server state construction <= response receipt. Thus
    // request_started - reported age is a LOWER bound on the source host time.
    // Repeated observations may tighten the bound but cannot renew source time.
    const double lower=double(t.request_started_ms)/1000.-t.imu_sample_age_ms/1000.-.002;
    const double offset=lower-double(*source_time)/1000.;
    host_anchors_.emplace_back(now,offset);
    while(host_anchors_.size()>256||(!host_anchors_.empty()&&now-host_anchors_.front().first>2.))host_anchors_.pop_front();
    host_offset_lower_=host_anchors_.front().second;
    for(const auto& anchor:host_anchors_)*host_offset_lower_=std::max(*host_offset_lower_,anchor.second);
    wait_="waiting_sensor_alignment";
}
void NavigationInputs::enqueue(const navwire::Sensors& s,double received){
    if(!fault_.empty())return;
    if(!std::isfinite(received)){latch("sensor_host_timestamp_invalid");return;}
    if(last_sensor_sequence_){
        const uint32_t advance=s.sequence-*last_sensor_sequence_;
        if(!advance||advance>=0x80000000u){++sensor_ignored_;return;}
    }
    if(pending_.size()>=256){latch("sensor_alignment_queue_overflow");return;}
    last_sensor_sequence_=s.sequence;pending_.push_back({s,received});++sensor_queued_;
}
void NavigationInputs::drain(Navigator& nav,double now){
    // The first seed frame is not an established map/pose. If startup IMU
    // calibration invalidated that reference, remove the seed before accepting
    // a new heading origin; never clear an established or latched reference.
    if(startup_reset_pending_&&!established_&&!nav.localization_diagnostics().latched){
        nav.reset_localization();startup_reset_pending_=false;
    }
    if(!fault_.empty()){
        if(!notified_){nav.invalidate_localization(fault_,now);notified_=true;}return;
    }
    if(nav.localization_diagnostics().latched){latch(nav.localization_diagnostics().first_reason);notified_=true;return;}
    if(!std::isfinite(now)){latch("sensor_host_clock_invalid");drain(nav,now);return;}
    auto latest=alignment_.latest_unwrapped_ms();
    // Do not fabricate an invalid SensorFrame because an HTTP request failed.
    // A brief absence waits; an actually unobserved source timeout stops and
    // invalidates an established reference instead of silently reconnecting it.
    if(latest&&host_offset_lower_&&now-(*host_offset_lower_+double(*latest)/1000.)>config_.sensor_timeout_s){
        if(established_){latch("imu_source_stale");drain(nav,now);return;}
        wait_="waiting_fresh_imu_source";
    }
    while(!pending_.empty()){
        const auto p=pending_.front();double age=now-p.received_s;
        if(age<-.02){latch("sensor_host_timestamp_in_future");break;}
        if(age>config_.sensor_timeout_s){
            if(established_){latch("sensor_alignment_timeout");break;}
            pending_.pop_front();++startup_dropped_;wait_="waiting_fresh_sensor_baseline";continue;
        }
        if(!latest||!host_offset_lower_){wait_="waiting_imu_source";break;}
        const auto aligned=alignment_.lookup(p.sample.uptime_ms);
        if(aligned.status==ImuLookupStatus::Fault){latch(alignment_.fault_reason());break;}
        if(aligned.status==ImuLookupStatus::BeforeHistory){
            if(established_){latch("imu_history_missing");break;}
            pending_.pop_front();++startup_dropped_;wait_="waiting_common_source_baseline";continue;
        }
        if(aligned.status==ImuLookupStatus::NeedFuture){wait_="waiting_future_imu_bracket";break;}
        const double source_lower=*host_offset_lower_+double(aligned.source_unwrapped_ms)/1000.;
        if(now-source_lower>config_.sensor_timeout_s){
            if(established_){latch("aligned_imu_source_stale");break;}
            pending_.pop_front();++startup_dropped_;wait_="waiting_fresh_aligned_baseline";continue;
        }
        SensorFrame f;f.timestamp_s=p.received_s;f.device_timestamp_s=double(aligned.source_unwrapped_ms)/1000.;
        f.sequence=p.sample.sequence;f.yaw_rad=aligned.yaw_rad;f.imu_valid=true;
        f.imu_age_s=std::max(0.,p.received_s-source_lower);f.sonar_observation_ids_valid=true;
        for(unsigned i=0;i<4;++i){f.counts[i]=uint32_t(p.sample.counts[i]);f.encoder_ok[i]=!(p.sample.illegal&(1U<<i));}
        for(unsigned i=0;i<3;++i){
            f.sonar_age_s[i]=p.sample.age_ms[i]/1000.;
            f.sonar_observation_id[i]=p.sample.uptime_ms-p.sample.age_ms[i];
            // SNS1 has already passed framing/CRC/flag checks. Preserve the
            // original source age/observation ID, and normalize only this
            // raw-before-filter far-censor case. An invalid,
            // faulted or expired channel cannot be revived by saturation.
            if((p.sample.valid&(1U<<i))&&!(p.sample.fault&(1U<<i))&&
               robot_sonar_range_supported(p.sample.distance_mm[i]))
                f.sonar_m[i]=robot_sonar_range_censored(p.sample.distance_mm[i])/1000.;
        }
        pending_.pop_front();++sensor_ingested_;
        const bool accepted=nav.ingest(f,now);
        if(nav.localization_diagnostics().latched){latch(nav.localization_diagnostics().first_reason);notified_=true;break;}
        if(accepted)established_=true;
        wait_=accepted?"":"waiting_second_aligned_baseline";
    }
    if(!fault_.empty()&&!notified_){nav.invalidate_localization(fault_,now);notified_=true;}
}
std::string NavigationInputs::diagnostics_json() const{
    picojson::object o;auto num=[&](const char* k,double v){o[k]=picojson::value(v);};
    o["wait_reason"]=picojson::value(wait_);o["fault_reason"]=picojson::value(fault_);
    o["established"]=picojson::value(established_);num("pending",pending_.size());
    num("imu_accepted",imu_accepted_);num("imu_duplicates",imu_duplicates_);num("imu_ignored",imu_ignored_);
    num("sensor_queued",sensor_queued_);num("sensor_ingested",sensor_ingested_);num("startup_dropped",startup_dropped_);
    num("sensor_ignored",sensor_ignored_);num("transport_waits",transport_waits_);
    const auto d=alignment_.diagnostics();o["source_event"]=picojson::value(d.event);
    num("before_sequence",d.before_sequence);num("after_sequence",d.after_sequence);
    num("before_timestamp_ms",d.before_timestamp_ms);num("after_timestamp_ms",d.after_timestamp_ms);
    num("source_dt_s",d.dt_s);num("source_dyaw_rad",d.dyaw_rad);
    if(host_offset_lower_)num("host_offset_lower_s",*host_offset_lower_);
    return picojson::value(o).serialize();
}
} // namespace robot_nav
