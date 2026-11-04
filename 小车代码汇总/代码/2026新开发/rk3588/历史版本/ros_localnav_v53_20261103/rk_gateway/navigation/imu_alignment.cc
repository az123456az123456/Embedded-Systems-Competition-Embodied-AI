#include "imu_alignment.h"
#include <cmath>
#include <stdexcept>

namespace robot_nav {
namespace {
double angle(double value){return std::atan2(std::sin(value),std::cos(value));}
int64_t modular_delta(uint32_t newer,uint32_t older){
    uint32_t d=newer-older;
    return d<0x80000000u?int64_t(d):-int64_t(uint64_t(0x100000000ULL)-d);
}
ImuAlignmentDiagnostics bracket(const ImuYawSample& a,const ImuYawSample& b,int64_t delta){
    ImuAlignmentDiagnostics d;d.before_sequence=a.sequence;d.after_sequence=b.sequence;
    d.before_timestamp_ms=a.timestamp_ms;d.after_timestamp_ms=b.timestamp_ms;
    d.dt_s=delta/1000.;d.dyaw_rad=angle(b.yaw_rad-a.yaw_rad);return d;
}
}
ImuAlignment::ImuAlignment(double max_turn,double allowance,uint32_t gap)
    :max_turn_rps_(max_turn),jump_allowance_rad_(allowance),max_gap_ms_(gap){
    if(!std::isfinite(max_turn)||max_turn<=0||!std::isfinite(allowance)||allowance<0||!gap||gap>=0x80000000u)
        throw std::invalid_argument("invalid IMU alignment limits");
}
void ImuAlignment::reset(){history_.clear();fault_.clear();diagnostics_=ImuAlignmentDiagnostics{};}
ImuPushStatus ImuAlignment::fail(const std::string& why){fault_=why;diagnostics_.event=why;return ImuPushStatus::Fault;}
std::optional<int64_t> ImuAlignment::latest_unwrapped_ms() const{
    if(history_.empty())return {};
    return history_.back().time_ms;
}
ImuPushStatus ImuAlignment::push(const ImuYawSample& input){
    if(faulted())return ImuPushStatus::Fault; // retain the FIRST fault evidence
    if(!std::isfinite(input.yaw_rad))return fail("imu_source_yaw_nonfinite");
    ImuYawSample sample=input;sample.yaw_rad=angle(sample.yaw_rad);
    if(history_.empty()){
        history_.push_back({sample,int64_t(sample.timestamp_ms)});
        diagnostics_=bracket(sample,sample,0);diagnostics_.event="first_source_sample";
        return ImuPushStatus::Accepted;
    }
    const Entry& previous=history_.back();
    const uint32_t advance=sample.sequence-previous.sample.sequence;
    const int64_t delta=modular_delta(sample.timestamp_ms,previous.sample.timestamp_ms);
    diagnostics_=bracket(previous.sample,sample,delta);
    if(!advance){
        // A reused HTTP snapshot cannot move either its yaw or source time.
        // Zero/calibration actions must reset localization explicitly instead.
        if(delta||std::abs(diagnostics_.dyaw_rad)>1e-9)return fail("imu_source_snapshot_conflict");
        diagnostics_.event="duplicate_source_sample";return ImuPushStatus::Duplicate;
    }
    if(advance>=0x80000000u){diagnostics_.event="old_source_sequence";return ImuPushStatus::IgnoredOld;}
    if(delta<=0)return fail("imu_source_timestamp_not_forward");
    if(uint64_t(delta)>max_gap_ms_)return fail("imu_source_gap");
    if(std::abs(diagnostics_.dyaw_rad)>max_turn_rps_*diagnostics_.dt_s+jump_allowance_rad_)
        return fail("imu_source_yaw_jump");
    const int64_t time=previous.time_ms+delta;history_.push_back({sample,time});
    // Keep a lower bracket at the two-second boundary. The sample count bound
    // also prevents unbounded memory if a caller supplies a 1 kHz source.
    while(history_.size()>2&&(time-history_[1].time_ms>2000||history_.size()>2048))history_.pop_front();
    diagnostics_.event="accepted_source_sample";return ImuPushStatus::Accepted;
}
AlignedYaw ImuAlignment::lookup(uint32_t target) const{
    AlignedYaw out;
    if(faulted()){out.status=ImuLookupStatus::Fault;out.bracket=diagnostics_;return out;}
    if(history_.empty())return out;
    const Entry& latest=history_.back();
    const int64_t time=latest.time_ms+modular_delta(target,latest.sample.timestamp_ms);
    out.source_unwrapped_ms=time;
    if(time<history_.front().time_ms){out.status=ImuLookupStatus::BeforeHistory;return out;}
    if(time>latest.time_ms)return out; // NO extrapolation, including a 1 ms gap
    for(size_t i=0;i<history_.size();++i){
        const Entry& upper=history_[i];
        if(time>upper.time_ms)continue;
        out.status=ImuLookupStatus::Ready;
        if(time==upper.time_ms){
            out.yaw_rad=upper.sample.yaw_rad;out.bracket=bracket(upper.sample,upper.sample,0);
            out.bracket.event="exact_source_sample";return out;
        }
        const Entry& lower=history_[i-1];const int64_t span=upper.time_ms-lower.time_ms;
        const double fraction=double(time-lower.time_ms)/span;
        out.bracket=bracket(lower.sample,upper.sample,span);out.bracket.event="interpolated_source_samples";
        out.yaw_rad=angle(lower.sample.yaw_rad+fraction*out.bracket.dyaw_rad);return out;
    }
    return out;
}
} // namespace robot_nav
