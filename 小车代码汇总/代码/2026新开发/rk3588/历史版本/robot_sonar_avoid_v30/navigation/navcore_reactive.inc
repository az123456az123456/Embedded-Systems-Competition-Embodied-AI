// Included inside robot_nav: local obstacle response shared by heading assist
// and map navigation. Distances are measured, not time-based PWM estimates.
LocalAvoidanceDiagnostics Navigator::local_avoidance_diagnostics()const{
    return {bypass_!=Bypass::None,bypass_heading_,bypass_target_,bypass_turn_travel_,bypass_scan_,
            bypass_departure_distance_,bypass_parallel_distance_,bypass_attempts_,bypass_side_};
}
void Navigator::bypass_stop(Bypass next,double now,const std::string& why){
    bypass_=Bypass::Stop;bypass_after_stop_=next;bypass_turning_=false;
    stopped_requested_s_=now;stationary_since_.reset();
    state_="AVOID_STOP";reason_=why;waiting_control();
}
bool Navigator::bypass_stopped(double now){
    bool still=frame_->timestamp_s>stopped_requested_s_&&std::abs(measured_yaw_rps_)<=.08;
    for(double v:measured_wheel_mps_)still=still&&std::abs(v)<=config_.stopped_wheel_mps;
    if(!still){stationary_since_.reset();reason_="avoid_waiting_encoder_stop";return false;}
    if(!stationary_since_)stationary_since_=now;
    if(now-*stationary_since_<config_.stop_dwell_s){reason_="avoid_stop_dwell";return false;}
    // Check source observation ages directly. Requiring a successful map
    // projection here would reintroduce the old pre-mapped-footprint deadlock.
    for(unsigned i=0;i<3;++i)
        if(frame_->timestamp_s-frame_->sonar_age_s[i]-config_.sonar_latency_s<*stationary_since_){
            reason_="avoid_waiting_new_sonar";return false;
        }
    last_motion_mps_=0;return true;
}
bool Navigator::bypass_rotation_clear(double target,double now)const{
    local_guard_=LocalGuardDiagnostics{};local_guard_.evaluated=true;
    local_guard_.evaluated_s=now;local_guard_.target_yaw_rad=target;local_guard_.pure_rotation=true;
    const double radius=std::hypot(std::max(config_.body_front_m,config_.body_rear_m),
                                  std::max(config_.body_left_m,config_.body_right_m))+config_.safety_margin_m;
    // Local avoidance uses current ranges, never historical map occupancy.
    // Three beams cannot certify unseen space; retain their measured physical
    // rotation-radius bound and the independent FPGA direction floor.
    for(unsigned i=0;i<3;++i){const auto& m=config_.sonar_mounts[i];
        const double origin_projection=m[0]*std::cos(m[2])+m[1]*std::sin(m[2]);
        const double required=std::max(config_.sonar_min_m,radius-origin_projection);
        if(*frame_->sonar_m[i]<=required){
            local_guard_.blocked=true;local_guard_.reason="range_rotation_radius_insufficient";
            local_guard_.channel=int(i);local_guard_.range_m=*frame_->sonar_m[i];local_guard_.range_required_m=required;
            local_guard_.required_radius_m=radius;return false;
        }
    }
    // Only the next short swept increment is checked here. Cumulative scan
    // yaw is measured separately across IMU wrap; no 29-degree fatal segment.
    const double angle=wrap(target-pose_.yaw);
    // Match the independent raw FPGA direction floor as well as geometry;
    // otherwise a gated zero PWM could be misreported as encoder no-progress.
    if((angle>0&&*frame_->sonar_m[1]<=.20)||(angle<0&&*frame_->sonar_m[2]<=.20)){
        const unsigned ch=angle>0?1:2;local_guard_.blocked=true;local_guard_.reason="raw_direction_floor";
        local_guard_.channel=int(ch);local_guard_.range_m=*frame_->sonar_m[ch];local_guard_.range_required_m=.20;
        local_guard_.required_radius_m=radius;return false;
    }
    local_guard_.reason="fresh_sonar_rotation_clear_map_not_authority";
    return true;
}
double Navigator::bypass_front_clear_distance(double now)const{
    // Clearance must support the intended translation, not merely a zero-speed
    // reading; otherwise stop/resume cycles repeat at the braking threshold.
    const double speed=std::min(config_.max_wheel_mps,straight_assist_?config_.straight_speed_mps:config_.detour_mps);
    return std::max(config_.avoid_clear_front_m,config_.stop_clearance_m+
        speed*reaction_delay(now)+speed*speed/(2*config_.brake_mps2)+.04);
}
bool Navigator::bypass_forward_clear(double now)const{
    // Side ranges nominate the initial turn direction only. They do not
    // prevent leaving a turn once the FRONT beam certifies forward clearance.
    return *frame_->sonar_m[0]>bypass_front_clear_distance(now);
}
void Navigator::bypass_turn(double goal,Bypass phase,double now){
    (void)goal;(void)phase;
    bypass_target_=wrap(pose_.yaw+bypass_side_*config_.avoidance_step_rad);
    bypass_turn_start_=bypass_turn_last_=pose_.yaw;bypass_turn_travel_=0;
    bypass_turn_started_s_=now;bypass_turning_=true;bypass_=Bypass::TurnOut;
    progress_at_=now;progress_pose_=pose_;
    state_="AVOID_TURN_CLEAR";reason_="turn_until_new_front_clear";waiting_control();
}
Control Navigator::request_bypass_rejoin(double now){
    state_="AVOID_REJOIN_WAIT";reason_="avoid_finished_waiting_map_rejoin";
    if(straight_assist_){
        // Reactive straight assist continues in the newly observed direction.
        // It neither returns to the old heading nor invents a geometric route.
        straight_origin_=pose_;
        bypass_=Bypass::None;avoidance_active_=obstacle_stopping_=false;
        detour_origin_.reset();rejoin_point_.reset();path_.clear();original_route_.clear();
        progress_at_=now;progress_pose_=pose_;state_="STRAIGHT_ADVANCE";
        reason_="avoid_continue_new_heading";return waiting_control();
    }
    if(original_route_.size()<2){
        // Exploration may meet its first obstacle before any FPGA route exists.
        // Continue bounded exploration from here; never invent an old route.
        bypass_=Bypass::None;avoidance_active_=obstacle_stopping_=false;
        detour_origin_.reset();rejoin_point_.reset();path_.clear();
        if(exploration_){goal_.reset();initialize_bootstrap(now);return waiting_control();}
        return fail("avoid_no_saved_map_route");
    }
    const auto bits=grid_.planning_bitmap(now);
    if(!free_world(xy(pose_),bits)){reason_="avoid_rejoin_start_not_observed";return waiting_control();}
    // Only now choose a forward point on the saved route and ask the FPGA A*.
    const Point anchor=detour_origin_?xy(*detour_origin_):xy(pose_);
    size_t nearest=0;double best=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<original_route_.size();++i){double d=distance(original_route_[i],anchor);if(d<best){best=d;nearest=i;}}
    double along=0;
    for(size_t i=nearest+1;i<original_route_.size();++i){
        along+=distance(original_route_[i],original_route_[i-1]);
        if(along<config_.rejoin_ahead_m||!free_world(original_route_[i],bits))continue;
        rejoin_index_=unsigned(i);rejoin_point_=original_route_[i];
        const auto a=original_route_[i-1],b=original_route_[i];
        rejoin_heading_=std::atan2(b.second-a.second,b.first-a.first);
        path_.clear();bypass_=Bypass::None;obstacle_stopping_=false;
        request(*rejoin_point_,now,bits,"rejoin",bypass_side_);return waiting_control();
    }
    reason_="avoid_finished_no_observed_map_rejoin_point";return waiting_control();
}
Control Navigator::service_bypass(double now,double dt){
    if(bypass_motion_s_>120)return fail("avoid_local_sequence_timeout");
    if(bypass_==Bypass::Stop){
        if(!bypass_stopped(now))return waiting_control();
        bypass_=bypass_after_stop_;obstacle_stopping_=false;
        progress_at_=now;progress_pose_=pose_;
    }
    if(bypass_==Bypass::Select){
        state_="AVOID_SELECT";reason_="choose_smaller_side_after_stop";
        if(bypass_forward_clear(now)){
            // Confirm twice while stopped, using distinct source observations.
            // Packet sequence and repeated stale values are not observations.
            state_="AVOID_CONFIRM_CLEAR";reason_="waiting_two_new_stopped_front_observations";
            const double source=frame_->timestamp_s-frame_->sonar_age_s[0]-config_.sonar_latency_s;
            if(!stationary_since_||source<*stationary_since_)return waiting_control();
            const uint32_t id=frame_->sonar_observation_ids_valid?frame_->sonar_observation_id[0]:
                uint32_t(std::llround((frame_->timestamp_s-frame_->sonar_age_s[0])*1000));
            if(!bypass_clear_id_||id!=*bypass_clear_id_){bypass_clear_id_=id;++bypass_clear_samples_;}
            if(bypass_clear_samples_<2)return waiting_control();
            bypass_target_=pose_.yaw;progress_at_=now;progress_pose_=pose_;
            if(straight_assist_)return request_bypass_rejoin(now);
            bypass_=Bypass::MoveOut;bypass_leg_start_=total_distance_;
            bypass_departure_distance_=0;
        }else{
            bypass_clear_id_.reset();bypass_clear_samples_=0;
            const double left=*frame_->sonar_m[1],right=*frame_->sonar_m[2];
            // User policy: choose the SMALLER measured side once per obstacle.
            // Equal readings retain the clockwise/right fallback. A blocked chosen
            // side waits safely; never silently substitute the opposite side.
            if(!bypass_side_)bypass_side_=left<right?1:-1;
            const double step=std::max(.01,std::min(.08,config_.max_turn_rps*std::max(dt,.02)));
            if(bypass_rotation_clear(wrap(pose_.yaw+bypass_side_*step),now)){
                bypass_turn(pose_.yaw+bypass_side_*step,Bypass::TurnOut,now);return waiting_control();
            }
            state_="AVOID_WAIT";reason_="avoid_selected_side_physical_clearance_insufficient";
            progress_at_=now;progress_pose_=pose_;return waiting_control();
        }
    }
    if(bypass_==Bypass::TurnOut){
        const double travel=std::abs(wrap(pose_.yaw-bypass_turn_last_));bypass_turn_last_=pose_.yaw;
        bypass_turn_travel_+=travel;bypass_scan_+=travel;
        if(bypass_scan_>=4*PI)return fail("avoid_scan_exhausted");
        // A range acquired before this turn cannot nominate a clear direction.
        // Nomination only stops; forward motion additionally requires all new
        // post-stop ranges and two distinct clear front observations above.
        const double source=frame_->timestamp_s-frame_->sonar_age_s[0]-config_.sonar_latency_s;
        if(source>bypass_turn_started_s_&&bypass_forward_clear(now)){
            bypass_rotated_=true;bypass_clear_samples_=0;bypass_clear_id_.reset();
            bypass_stop(Bypass::Select,now,"avoid_stop_confirm_new_front_direction");return waiting_control();
        }
        const double step=std::max(.01,std::min(.08,config_.max_turn_rps*std::max(dt,.02)));
        bypass_target_=wrap(pose_.yaw+bypass_side_*step);
        if(!bypass_rotation_clear(bypass_target_,now)){
            state_="AVOID_WAIT";reason_="avoid_selected_side_physical_clearance_insufficient";
            progress_at_=now;progress_pose_=pose_;return waiting_control();
        }
        if(std::abs(wrap(pose_.yaw-progress_pose_.yaw))>.03){progress_at_=now;progress_pose_=pose_;}
        if(now-progress_at_>config_.no_progress_s)return fail("no_odometry_progress");
        state_="AVOID_TURN_CLEAR";reason_="turn_until_new_front_clear";
        return emit(0,bypass_side_*std::min(.35,config_.max_turn_rps),dt);
    }
    if(bypass_==Bypass::MoveOut){
        const double travelled=std::max(0.,total_distance_-bypass_leg_start_);
        bypass_departure_distance_=travelled;
        const double remaining=std::max(0.,config_.avoid_rejoin_distance_m-travelled);
        if(remaining<=.005){bypass_stop(Bypass::Rejoin,now,"avoid_050m_complete");return waiting_control();}
        const double err=wrap(bypass_target_-pose_.yaw);
        if(*frame_->sonar_m[0]<=front_stop_distance(now)){
            ++bypass_attempts_;bypass_side_=0;bypass_clear_id_.reset();bypass_clear_samples_=0;
            bypass_stop(Bypass::Select,now,"avoid_new_obstacle_stop_first");return waiting_control();
        }
        if(distance(xy(pose_),xy(progress_pose_))>.01){progress_pose_=pose_;progress_at_=now;}
        if(now-progress_at_>config_.no_progress_s)return fail("no_odometry_progress");
        const double b=config_.brake_mps2,reaction=reaction_delay(now),available=std::max(0.,*frame_->sonar_m[0]-config_.stop_clearance_m);
        const double safe=std::max(0.,-b*reaction+std::sqrt(b*b*reaction*reaction+2*b*available));
        const double speed=std::min({config_.detour_mps,config_.max_wheel_mps,safe*.8,std::sqrt(2*b*remaining)});
        const double ramp=std::min(speed,command_.linear_mps+config_.accel_mps2*dt);
        const double angular=bounded(err*1.8,-std::min(.12,ramp/config_.track_m),std::min(.12,ramp/config_.track_m));
        // The shared local .50m departure is gated by fresh FRONT sonar.
        // Keep map evidence intact for the later FPGA A* rejoin; it is not a
        // second collision authority during this explicitly sensor-only leg.
        state_="AVOID_ADVANCE_050";reason_="avoid_encoder_distance_050m_before_map_rejoin";
        return emit(speed,angular,dt);
    }
    if(bypass_==Bypass::Rejoin)return request_bypass_rejoin(now);
    return waiting_control();
}
