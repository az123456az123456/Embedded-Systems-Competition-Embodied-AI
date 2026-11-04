package cn.acg138.robotphone;

/** A destination bound to the exact displayed map, expressed at a cell centre in metres. */
public final class NavigationTarget {
    public final NavigationMap map;
    public final int cellX, cellY;
    public final double x, y;
    private NavigationTarget(NavigationMap map,int cellX,int cellY) {
        this.map=map;this.cellX=cellX;this.cellY=cellY;
        x=map.originX+(cellX+.5)*map.resolution;y=map.originY+(cellY+.5)*map.resolution;
    }
    public static NavigationTarget fromScreen(NavigationMap map,long now,double x,double y,
            double left,double top,double right,double bottom) {
        return fromScreen(map,now,x,y,left,top,right,bottom,
            map==null?0:map.viewOriginX,map==null?0:map.viewOriginY,map==null?0:map.viewSpan);
    }
    public static NavigationTarget fromScreen(NavigationMap map,long now,double x,double y,
            double left,double top,double right,double bottom,double viewX,double viewY,double viewSpan) {
        if(map==null||!map.fresh(now))throw new IllegalArgumentException("地图已过期，等待更新");
        if(!map.poseFresh(now))throw new IllegalArgumentException("定位未就绪，暂时无法导航");
        if(!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(left)||!Double.isFinite(top)||
                !Double.isFinite(right)||!Double.isFinite(bottom)||right<=left||bottom<=top||
                x<left||x>=right||y<=top||y>bottom)throw new IllegalArgumentException("请点击地图内的可通行区域");
        if(!Double.isFinite(viewX)||!Double.isFinite(viewY)||!Double.isFinite(viewSpan)||viewSpan<=0)throw new IllegalArgumentException("地图显示范围无效");
        double wx=viewX+(x-left)/(right-left)*viewSpan;
        int cx=(int)Math.floor((wx-map.originX)/map.resolution);
        double wy=viewY+(bottom-y)/(bottom-top)*viewSpan;
        int cy=(int)Math.floor((wy-map.originY)/map.resolution);
        if(cx<0||cy<0||cx>=map.width||cy>=map.height)throw new IllegalArgumentException("目标超出FPGA当前6.4米导航窗口");
        int index=cy*map.width+cx;char cell=map.cells.charAt(index);
        if(cell!='.')throw new IllegalArgumentException(cell=='#'?"这里是障碍物，请换一个位置":"这里尚未探明，请选择深灰色空闲区域");
        if(!map.navigable.isEmpty()&&map.navigable.charAt(index)!='.')
            throw new IllegalArgumentException("距离障碍物或未知区域太近，请换一个位置");
        return new NavigationTarget(map,cx,cy);
    }
    public boolean fresh(long now) { return map.fresh(now)&&map.poseFresh(now); }
    public static String reason(String reason) {
        if(reason==null)return "导航请求失败";
        switch(reason) {
            case "semantic_service_not_ready": return "RK 语义搜索服务尚未就绪";
            case "semantic_service_start_rejected": return "RK 未接受本次语义搜索，请查看状态后重试";
            case "unsupported_semantic_command": return "不支持该目标，请使用输入框列出的搜索口令";
            case "select_endpoint_first": return "请先选择终点并预览路线，再点击开始导航";
            case "PREVIEW_READY": return "路线预览已就绪，点击开始才行驶";
            case "CLAIM": case "WAIT_PERMIT": return "正在确认导航授权";
            case "WAIT_START_ACK": return "正在确认开始导航";
            case "RUNNING": case "FOLLOWING": return "正在前往目的地";
            case "AVOIDING": return "正在避开障碍";
            case "CANCELLED": case "FPGA_cancelled": return "导航已取消";
            case "FPGA_arrived": return "已到达目的地";
            case "FPGA_no_route": return "没有可通行路线，请重新选择终点";
            case "GOAL_OUTSIDE_WINDOW": return "目标超出当前导航窗口，请重新选择";
            case "BLOCKED_ENDPOINT": return "目标距离障碍物太近，请重新选择";
            case "map_changed_reselect_goal": return "地图已变化，请重新选择终点";
            case "command_timeout_30s": return "30秒未收到有效指令，电机已停用";
            case "motion_command_stale": return "控制指令暂缓，已暂停运动";
            case "avoid_both_sides_physical_clearance_insufficient": return "左右转动空间不足，停车等待";
            case "map_snapshot_expired": return "地图已更新，请重新点选目的地";
            case "map_session_changed": return "地图已重置，请重新点选目的地";
            case "goal_not_verified_free": return "目标附近空间不足，请选择更开阔的位置";
            case "goal_outside_local_map": return "目标已超出当前地图，请重新选择";
            case "stale_navigation_intent": case "stale_navigation_sequence": return "导航请求已取消，请重新选择";
            case "calibration_not_confirmed": case "motion_calibration_not_confirmed": return "请先在 RK 端确认车速及制动标定";
            case "observe_only": return "RK 端处于观察模式，请以 --drive 启动导航";
            case "map_not_ready": return "等待有效地图，请稍后再选择";
            case "start_not_verified_free": return "小车周围尚未确认安全空间，请先补充测距建图";
            case "odometry_not_ready": return "编码器定位尚未就绪";
            case "localization_lost_reset_map_required": return "定位已丢失，请在 RK 端重置地图";
            case "telemetry_stale": return "传感器数据已过期，请等待更新";
            case "imu_invalid_or_stale": case "imu_not_ready": return "IMU 尚未就绪，请等待校准完成";
            case "encoder_invalid": return "编码器数据无效，请检查接线";
            case "sonar_invalid": return "超声波数据无效，请检查传感器";
            case "wheel_mapping_mismatch": case "verified_wheel_layout_required": return "请核对已验证的轮组设置，并等待 FPGA 确认";
            case "motor_link_not_ready": return "电机控制连接未就绪，请检查 FPGA";
            case "dashboard_state_stale": return "RK 状态未更新，请稍后重试";
            case "arm_from_phone_first": return "请先使能电机，再选择目的地";
            case "phone_foreground_lease_required": case "phone_lease_expired":
            case "owner_not_ready": return "请保持手机在前台并重新使能电机";
            case "ARRIVED": case "arrived": case "goal_reached": return "已到达目的地";
            case "PLANNING": case "WAIT_PLAN": case "planning": return "正在规划路线";
            case "FOLLOW_ROUTE": case "tracking": return "正在前往目的地";
            case "OBSTACLE_STOP": case "blocked": return "前方受阻，已停车";
            case "waiting": return "等待导航数据";
            case "idle": return "点选深灰色空闲区域前往";
            case "DETOUR": return "正在绕过障碍";
            case "SCAN_STOP": case "TURN_STOP": return "已停车，准备小角度转向";
            case "SCAN_CLOCKWISE": return "正在顺时针扫描可通行方向";
            case "SCAN_OBSERVE": case "TURN_OBSERVE": return "已停车，等待最新测距";
            case "TURN_SEGMENT": return "正在分段调整方向";
            case "rotation_sweep_not_verified_free": case "rotation_sweep_blocked_or_unknown": return "转动空间尚未确认安全，已停车";
            case "clockwise_scan_exhausted_no_verified_corridor": return "扫描后未找到可通行方向，已停车";
            case "no_verified_frontier": return "暂无可安全前往的新区域";
            case "navigation_not_ready": return "导航尚未就绪，请检查 RK 状态";
            case "navigation_zero_intent_required": return "导航启动已取消，请确认连接后重新开始";
            case "REJOIN_HEADING": return "正在恢复原路线方向";
            case "STRAIGHT_STOP": return "已停车，观测直行避障空间";
            case "STRAIGHT_ADVANCE": case "STRAIGHT_DRIVE": case "STRAIGHT": return "按原方向保持直行";
            case "observe_before_straight_assist": return "直行前停车观测";
            case "straight_sonar_gated_heading_captured": return "按锁定航向低速直行";
            case "straight_heading_sweep_not_verified": return "航向修正区域尚未观测，已停车";
            case "straight_side_clearance_stop_first": return "侧方距离不足，先停车";
            case "known_obstacle_in_straight_sweep": return "前方行驶范围有障碍，已停车";
            case "straight_obstacle_cleared_reobserve": return "障碍已清除，重新观测";
            case "straight_planned_turn_sweep_not_observed": return "绕行／回归转向空间尚未观测，已停车";
            case "straight_detour_map_changed_stop_replan": return "绕行地图变化，停车后重新规划";
            case "straight_detour_replan_limit": return "绕行反复变化已停止，请检查环境";
            case "straight_avoidance_footprint_not_observed": return "绕行旋转区域尚未观测，已停车";
            case "straight_no_verified_rejoin_point": return "尚无可安全回归的原路线点";
            case "straight_detour_endpoints_not_observed": return "绕行起终点空间尚未观测，已停车";
            case "straight_scan_no_verified_corridor": return "扫描后尚未找到安全通道";
            case "straight_scan_sweep_not_observed": return "扫描旋转区域尚未观测，已停车";
            case "straight_original_heading_restored": return "已回归原方向，准备继续直行";
            case "STRAIGHT_WAIT": return "已停车，等待安全通路";
            case "straight_assist_not_supported": return "请先安装 RK v27 更新，再使用保持直行避障";
            case "AVOID_STOP": case "avoid_new_obstacle_stop_first": return "前方距离不足，先停车";
            case "AVOID_SELECT": return "避开近侧，优先较宽方向";
            case "AVOID_TURN_CLEAR": return "正在向所选侧转向，等待正前方距离足够";
            case "AVOID_CONFIRM_CLEAR": case "avoid_stop_confirm_new_front_direction": return "正前方已有空间，确认最新测距";
            case "AVOID_WAIT": case "avoid_selected_side_physical_clearance_insufficient": return "当前近距不足，停车等待安全转向";
            case "avoid_stop_dwell": case "avoid_waiting_encoder_stop": return "等待车轮停止";
            case "avoid_waiting_new_sonar": case "SENSOR_WAIT": return "超声波暂未更新，停车等待新数据";
            case "sonar_unavailable_timeout": return "超声波数据持续无效，已停车";
            case "avoid_continue_new_heading": return "按避障后的新方向继续直行";
            case "AVOID_REJOIN_WAIT": case "avoid_finished_waiting_map_rejoin": return "已绕行，等待规划回归原路线";
            case "avoid_finished_no_observed_map_rejoin_point": case "avoid_no_saved_map_route": case "avoid_rejoin_start_not_observed": return "暂无可用的地图回归路线，已停车";
            case "avoid_scan_exhausted": case "avoid_local_sequence_timeout": return "未找到足够通行距离，已停车";
            case "STOPPED": return "导航已停止";
            case "FAULT": return "导航异常，已停车";
            case "imu_stale": return "IMU 数据未更新，导航已停止";
            case "sonar_stale": case "sensors_stale": return "传感器数据未更新，导航已停止";
            case "reset-map_required_after_fpga_reset_or_imu_zero_or_wheel_mapping_change": case "reset-map_required_after_fpga_reset_or_imu_zero": return "位置参考已变化，请在 RK 端重置地图";
            case "goal_blocked": case "goal_occupied_or_unknown": return "目标未确认可通行，请换一个位置";
            case "operator_stop": case "manual_takeover": return "已停止自主导航";
            default: return FollowPresentation.reason(reason);
        }
    }
}
