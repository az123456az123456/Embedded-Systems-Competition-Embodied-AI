// Included inside robot_nav: local obstacle response shared by heading assist
// and map navigation. Distances are measured, not time-based PWM estimates.
namespace {
int reactive_turn_side(const SensorFrame& frame,double clearance){
    const double left=*frame.sonar_m[1],right=*frame.sonar_m[2];
    const bool left_near=left<=clearance,right_near=right<=clearance;
    // Positive angular velocity is counterclockwise/left. A single close
    // flank always turns AWAY from it, even when the front beam is clear.
    if(left_near&&!right_near)return -1;
    if(right_near&&!left_near)return 1;
    // Front-only or both-flank obstacles select the measured larger opening.
    // Equal ranges use clockwise; the physical rotation guard still applies.
    return left>right?1:-1;
}
}
LocalAvoidanceDiagnostics Navigator::local_avoidance_diagnostics()const{
    return {bypass_!=Bypass::None,bypass_heading_,bypass_target_,bypass_turn_travel_,bypass_scan_,
            bypass_departure_distance_,bypass_parallel_distance_,bypass_attempts_,bypass_side_};
}
void Navigator::bypass_stop(Bypass next,double now,const std::string& why){
    bypass_=Bypass::Stop;bypass_after_stop_=next;bypass_turning_=false;
    bypass_clear_samples_=0;bypass_clear_id_.reset();bypass_min_turn_reached_s_.reset();
    stopped_requested_s_=now;stationary_since_.reset();
    state_="AVOID_STOP";reason_=why;waiting_control();
}
bool Navigator::bypass_stopped(double now){
    if(!motion_stopped(now))return false;
    // Overlap source arrival with actual braking. The fixed acoustic latency
    // belongs in stopping distance, not a second post-stop observation timer.
    for(unsigned i=0;i<3;++i)
        if(control_ranges().timestamp_s-control_ranges().sonar_age_s[i]+1e-9<stopped_requested_s_){
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
        if(*control_ranges().sonar_m[i]<=required){
            local_guard_.blocked=true;local_guard_.reason="range_rotation_radius_insufficient";
            local_guard_.channel=int(i);local_guard_.range_m=*control_ranges().sonar_m[i];local_guard_.range_required_m=required;
            local_guard_.required_radius_m=radius;return false;
        }
    }
    // Only the next short swept increment is checked here. Cumulative scan
    // yaw is measured separately across IMU wrap; no 29-degree fatal segment.
    const double angle=wrap(target-pose_.yaw);
    // Match the independent raw FPGA direction floor as well as geometry;
    // otherwise a gated zero PWM could be misreported as encoder no-progress.
    if((angle>0&&*control_ranges().sonar_m[1]<=.20)||(angle<0&&*control_ranges().sonar_m[2]<=.20)){
        const unsigned ch=angle>0?1:2;local_guard_.blocked=true;local_guard_.reason="raw_direction_floor";
        local_guard_.channel=int(ch);local_guard_.range_m=*control_ranges().sonar_m[ch];local_guard_.range_required_m=.20;
        local_guard_.required_radius_m=radius;return false;
    }
    local_guard_.reason="fresh_sonar_rotation_clear_map_not_authority";
    return true;
}
double Navigator::bypass_front_clear_distance(double now)const{
    // Clearance must support the intended translation, not merely a zero-speed
    // reading; otherwise stop/resume cycles repeat at the braking threshold.
    const double speed=std::min(config_.max_wheel_mps,straight_assist_?config_.straight_speed_mps:config_.detour_mps);
    return std::max(config_.avoid_clear_front_m,std::max(.30,config_.stop_clearance_m)+
        speed*reaction_delay(now)+speed*speed/(2*config_.brake_mps2)+.04);
}
bool Navigator::bypass_forward_clear(double now)const{
    // A clear front alone cannot resume straight into an unchanged flank
    // trigger. Two centimetres of release hysteresis prevent .30m chatter.
    const double side_release=std::max(.30,config_.side_clearance_m)+.02;
    return *control_ranges().sonar_m[0]>bypass_front_clear_distance(now)&&
           *control_ranges().sonar_m[1]>side_release&&*control_ranges().sonar_m[2]>side_release;
}
void Navigator::bypass_turn(double goal,Bypass phase,double now){
    (void)goal;(void)phase;
    bypass_target_=wrap(pose_.yaw+bypass_side_*config_.avoidance_step_rad);
    bypass_turn_start_=bypass_turn_last_=pose_.yaw;bypass_turn_travel_=0;
    bypass_min_turn_reached_s_.reset();bypass_clear_samples_=0;bypass_clear_id_.reset();
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
        straight_handoff_until_s_=now+.8;
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
        // The robot may wait here for seconds with PWM already zero. A proof
        // obtained before that wait is not permission to move after somebody
        // turns/drags the chassis: validate the *current* measured window.
        // Do not move stopped_requested_s_ forward on each retry, which would
        // make the required post-stop source observation impossible to obtain.
        if(!bypass_stopped(now)){
            bypass_clear_samples_=0;bypass_clear_id_.reset();bypass_min_turn_reached_s_.reset();
            return waiting_control();
        }
        state_="AVOID_SELECT";reason_="choose_away_from_close_side_after_stop";
        bypass_clear_id_.reset();bypass_clear_samples_=0;
        // A triggered obstacle is latched: a near->far jump with unchanged
        // heading cannot discharge it. First perform a measured short scan in
        // the safe selected direction, then observe that new direction.
        bypass_side_=reactive_turn_side(control_ranges(),std::max(.30,config_.side_clearance_m));
        const double step=std::max(.01,std::min(.08,config_.max_turn_rps*std::max(dt,.02)));
        if(bypass_rotation_clear(wrap(pose_.yaw+bypass_side_*step),now)){
            bypass_turn(pose_.yaw+bypass_side_*step,Bypass::TurnOut,now);
            // Stop was proved before Select. Continue into TurnOut in this
            // tick rather than adding an otherwise empty command frame.
        }else{
            state_="AVOID_WAIT";reason_="avoid_selected_side_physical_clearance_insufficient";
            progress_at_=now;progress_pose_=pose_;return waiting_control();
        }
    }
    if(bypass_==Bypass::TurnOut){
        const double travel=std::abs(wrap(pose_.yaw-bypass_turn_last_));bypass_turn_last_=pose_.yaw;
        bypass_turn_travel_+=travel;bypass_scan_+=travel;
        if(bypass_scan_>=4*PI)return fail("avoid_scan_exhausted");
        const double side_stop=std::max(.30,config_.side_clearance_m);
        const bool left_near=*control_ranges().sonar_m[1]<=side_stop,right_near=*control_ranges().sonar_m[2]<=side_stop;
        if(left_near!=right_near){
            const int away=left_near?-1:1;
            if(away!=bypass_side_){
                // A newly close flank may invalidate the initially safe
                // direction. Stop before reversing angular wheel setpoints.
                bypass_side_=away;bypass_clear_samples_=0;bypass_clear_id_.reset();
                bypass_stop(Bypass::Select,now,"avoid_new_side_obstacle_stop_before_reverse");
                return waiting_control();
            }
        }
        // Oscillatory yaw noise must not satisfy the new-scene requirement:
        // before the first crossing use signed NET displacement in the chosen
        // direction. Once crossed, retain its actual source timestamp across
        // yaw wrap and require new observations generated AFTER that crossing.
        const double minimum=bounded(config_.avoid_verify_turn_min_rad,1*PI/180,25*PI/180);
        if(!bypass_min_turn_reached_s_&&
           bypass_side_*wrap(pose_.yaw-bypass_turn_start_)>=minimum)
            bypass_min_turn_reached_s_=frame_->timestamp_s;
        // Confirm while turning, without a serial stop/dwell. Two distinct
        // observations of the newly scanned direction must clear front/flanks.
        // RK linear acceleration limiting and the FPGA PI/reversal deadtime
        // remain active; invalid/near observations reset this confirmation.
        const double source=control_ranges().timestamp_s-control_ranges().sonar_age_s[0]-config_.sonar_latency_s;
        if(bypass_min_turn_reached_s_&&source>*bypass_min_turn_reached_s_&&bypass_forward_clear(now)){
            const uint32_t id=control_ranges().sonar_observation_ids_valid?control_ranges().sonar_observation_id[0]:
                uint32_t(std::llround((control_ranges().timestamp_s-control_ranges().sonar_age_s[0])*1000));
            if(!bypass_clear_id_||id!=*bypass_clear_id_){bypass_clear_id_=id;++bypass_clear_samples_;}
            if(bypass_clear_samples_>=2){
                bypass_rotated_=true;bypass_turning_=false;bypass_target_=pose_.yaw;
                progress_at_=now;progress_pose_=pose_;
                if(straight_assist_){request_bypass_rejoin(now);return service_straight(now,dt);}
                bypass_=Bypass::MoveOut;bypass_leg_start_=total_distance_;bypass_departure_distance_=0;
                return service_bypass(now,dt); // one bounded phase handoff, no zero-command frame
            }
        }else{bypass_clear_samples_=0;bypass_clear_id_.reset();}
        const double step=std::max(.01,std::min(.08,config_.max_turn_rps*std::max(dt,.02)));
        bypass_target_=wrap(pose_.yaw+bypass_side_*step);
        if(!bypass_rotation_clear(bypass_target_,now)){
            // Re-enter a stopped boundary once. Keeping TurnOut while waiting
            // would allow old scan/stop evidence to resume after manual motion.
            bypass_stop(Bypass::Select,now,"avoid_selected_side_physical_clearance_insufficient");
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
        if(*control_ranges().sonar_m[0]<=front_stop_distance(now)||
           *control_ranges().sonar_m[1]<=std::max(.30,config_.side_clearance_m)||
           *control_ranges().sonar_m[2]<=std::max(.30,config_.side_clearance_m)){
            ++bypass_attempts_;bypass_side_=0;bypass_clear_id_.reset();bypass_clear_samples_=0;
            bypass_stop(Bypass::Select,now,"avoid_new_obstacle_stop_first");return waiting_control();
        }
        if(distance(xy(pose_),xy(progress_pose_))>.01){progress_pose_=pose_;progress_at_=now;}
        if(now-progress_at_>config_.no_progress_s)return fail("no_odometry_progress");
        const double b=config_.brake_mps2,reaction=reaction_delay(now),available=std::max(0.,*control_ranges().sonar_m[0]-std::max(.30,config_.stop_clearance_m));
        const double safe=std::max(0.,-b*reaction+std::sqrt(b*b*reaction*reaction+2*b*available));
        const double speed=std::min({config_.detour_mps,config_.max_wheel_mps,safe*.8,std::sqrt(2*b*remaining)});
        const double ramp=std::min(speed,command_.linear_mps+config_.accel_mps2*dt);
        const double angular=bounded(err*1.8,-std::min(.12,ramp/config_.track_m),std::min(.12,ramp/config_.track_m));
        // The shared local .50m departure is gated by all three fresh beams.
        // Keep map evidence intact for the later FPGA A* rejoin; it is not a
        // second collision authority during this explicitly sensor-only leg.
        state_="AVOID_ADVANCE_050";reason_="avoid_encoder_distance_050m_before_map_rejoin";
        return emit(speed,angular,dt);
    }
    if(bypass_==Bypass::Rejoin)return request_bypass_rejoin(now);
    return waiting_control();
}
