`timescale 1ns/1ps
// Signed PWM feed-forward with optional bounded encoder balancing on straight motion.
// Sequence freshness and watchdog depend only on newly accepted MOT1/MOT2/MOT3 commands.
module motor_remote_control #(
    parameter integer CLK_HZ=50000000, PWM_HZ=20000,
    parameter integer WATCHDOG_MS=500, AUTHORIZATION_MS=30000, RAMP_STEP_MS=10,
    parameter integer REVERSAL_MS=1, STATUS_MS=100,
    parameter integer MAX_PERMILLE=600,
    parameter integer ENCODER_SYNC_ENABLE=0,
    // M1=right front, M2=left rear, M3=right rear, M4=left front.
    parameter [3:0] DEFAULT_LEFT_MASK=4'b1010, DEFAULT_INVERT_MASK=4'b1111
)(
    input wire clk, rst_n, command_valid,
    input wire [127:0] command_data,
    input wire encoder_valid,
    input wire [127:0] encoder_deltas,
    input wire [3:0] encoder_illegal,
    output wire [3:0] motor_pwm, motor_in1, motor_in2,
    output wire [3:0] status_flags,
    output reg status_valid,
    output wire [127:0] status_data,
    output reg [31:0] last_sequence,
    output wire signed [15:0] applied_left,applied_right,
    input wire [63:0] cpu_pwm,
    input wire cpu_ready,cpu_trap,
    input wire [47:0] raw_sonar_mm,
    input wire [2:0] raw_sonar_online,
    output wire [4:0] control_flags,
    output reg [31:0] control_epoch,
    output wire [31:0] control_sequence,control_motion,
    output wire [7:0] control_masks
);
    localparam integer MS_CYCLES=CLK_HZ/1000;
    localparam integer PWM_CYCLES=CLK_HZ/PWM_HZ;
    localparam integer PWM_BITS=$clog2(PWM_CYCLES);
    localparam integer DEAD_CYCLES=MS_CYCLES*REVERSAL_MS;
    reg configured,armed,watchdog,have_sequence;
    reg [2:0] protocol_mode;
    reg [31:0] accepted_motion;
    reg [3:0] left_mask,invert_mask;
    reg signed [15:0] target[0:1];
    reg [15:0] magnitude[0:1];
    reg direction[0:1];
    reg [31:0] dead_count[0:1];
    reg [31:0] watchdog_count;
    reg [31:0] ms_count;
    reg [15:0] ramp_ms,status_ms;
    reg accepted;
    reg [PWM_BITS-1:0] pwm_count[0:3];
    reg [PWM_BITS:0] threshold[0:3];
    wire ms_tick=(ms_count==MS_CYCLES-1);
    wire ramp_tick=ms_tick && ramp_ms==RAMP_STEP_MS-1;
    // One decode stage separates division from safety validation and PWM updates.
    // Every field and the valid strobe advance together; both protocols retain
    // the same shared sequence/watchdog policy at the pipeline output.
    reg decoded_valid;
    reg [127:0] decoded_data;
    reg [7:0] divided_angle;
    // Register the small unsigned multiplier separately from output validation.
    // Invalid wide inputs are rejected below; only bounded low bits feed DSP/LUTs.
    reg [18:0] arc_product;
    wire [15:0] incoming_curve_abs=command_data[31] ? -command_data[31:16] : command_data[31:16];
    wire [15:0] incoming_abs=command_data[47] ? -command_data[47:32] : command_data[47:32];
    wire [12:0] bounded_angle=incoming_abs>16'd5500 ? 13'd5500 : incoming_abs[12:0];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin decoded_valid<=0;decoded_data<=0;divided_angle<=0;arc_product<=0;end
        else begin
            decoded_valid<=command_valid;
            if(command_valid)begin
                decoded_data<=command_data;
                divided_angle<=bounded_angle/13'd25;
                arc_product<=incoming_abs[9:0]*incoming_curve_abs[8:0];
            end
        end
    end
    wire packet_armed=decoded_data[80];
    wire [31:0] command_sequence=decoded_data[79:48];
    wire mot1=decoded_data[127:96]==32'h4d4f5431 && decoded_data[95:88]==1;
    wire mot2=decoded_data[127:96]==32'h4d4f5432 && decoded_data[95:88]==2;
    wire mot3=decoded_data[127:96]==32'h4d4f5433 && decoded_data[95:88]==3;
    wire mot4=decoded_data[127:96]==32'h4d4f5434 && decoded_data[95:88]==4;
    wire signed [15:0] raw_left_or_angle=decoded_data[47:32];
    wire signed [15:0] raw_right_or_forward=decoded_data[31:16];
    wire [15:0] angle_abs=raw_left_or_angle[15] ? -raw_left_or_angle : raw_left_or_angle;
    // Continuous visual servo: the signed camera bearing is recomputed on RK.
    // Positive centidegrees means target to the RIGHT, not a one-shot rotation.
    // Clamp magnitude before the narrow unsigned division. Restoring its sign
    // after the register gives exact signed division truncated toward zero.
    wire signed [15:0] angle_steer_raw=raw_left_or_angle[15] ?
        -$signed({8'b0,divided_angle}) : $signed({8'b0,divided_angle});
    wire signed [15:0] angle_steer=angle_abs<=16'd200 ? 16'sd0 : angle_steer_raw;
    wire signed [15:0] angle_forward=angle_abs>=16'd2500 ? 16'sd0 : raw_right_or_forward;
    // MOT3 uses signed outer duty and path curvature Q8. Positive curvature
    // means the path goes RIGHT, also while reversing. It is deliberately
    // distinct from MOT1 physical signed wheel speeds and MOT2 yaw bearing.
    // reduction=floor(abs(speed)*abs(curve)/512), maximum 50% inner duty.
    wire [15:0] arc_inner_abs=angle_abs-{6'b0,arc_product[18:9]};
    wire signed [15:0] arc_inner=raw_left_or_angle[15] ? -$signed(arc_inner_abs) : $signed(arc_inner_abs);
    wire signed [15:0] arc_left=raw_right_or_forward[15] ? arc_inner : raw_left_or_angle;
    wire signed [15:0] arc_right=raw_right_or_forward[15] ? raw_left_or_angle : arc_inner;
    wire signed [15:0] packet_left=mot3 ? arc_left : (mot2 ? angle_forward+angle_steer : raw_left_or_angle);
    wire signed [15:0] packet_right=mot3 ? arc_right : (mot2 ? angle_forward-angle_steer : raw_right_or_forward);
    wire [3:0] packet_mask=decoded_data[11:8];
    wire [3:0] packet_invert=decoded_data[3:0];
    wire [31:0] sequence_delta=command_sequence-last_sequence;
    wire newer=!have_sequence || (sequence_delta!=0 && !sequence_delta[31]);
    wire stop_packet=!packet_armed && raw_left_or_angle==0 && raw_right_or_forward==0;
    wire values_ok=mot4 ? (raw_left_or_angle>=-16'sd300 && raw_left_or_angle<=16'sd300 &&
                          raw_right_or_forward>=-16'sd1000 && raw_right_or_forward<=16'sd1000) : mot3 ? (raw_left_or_angle>=-16'sd600 && raw_left_or_angle<=16'sd600 &&
                          raw_right_or_forward>=-16'sd256 && raw_right_or_forward<=16'sd256) : mot2 ? (raw_left_or_angle>=-16'sd9000 && raw_left_or_angle<=16'sd9000 &&
                          raw_right_or_forward>=0 && raw_right_or_forward<=16'sd220) :
                         (raw_left_or_angle>=-1000 && raw_left_or_angle<=1000 &&
                          raw_right_or_forward>=-1000 && raw_right_or_forward<=1000);
    wire layout_ok=packet_mask!=0 && packet_mask!=15 && decoded_data[15:12]==0 &&
                   decoded_data[7:4]==0;
    wire format_ok=(mot1 || mot2 || mot3 || mot4) && decoded_data[87:81]==0;
    // An armed packet cannot silently change which physical wheel is driven.
    wire config_matches=packet_mask==left_mask && packet_invert==invert_mask;
    wire accept=decoded_valid && newer && values_ok && layout_ok && format_ok &&
        (stop_packet || (packet_armed && configured && config_matches));
    wire running=configured && armed && !watchdog;
    wire mot4_mode=protocol_mode==4;
    wire all_ranges_ok=raw_sonar_online==3'b111;
    wire local_ready=cpu_ready && !cpu_trap;
    wire signed [15:0] v_cmd=accepted_motion[15:0];
    wire signed [15:0] w_cmd=accepted_motion[31:16];
    // These raw gates bypass every estimator and CPU task. A stopped obstacle
    // can still be scanned by turning; a positive drive cannot cross200mm. The source-age/braking guard
    // runs in the softcore; this raw floor is independent and immediate.
    wire raw_stop=(v_cmd>0 && raw_sonar_mm[15:0]<=16'd200) ||
                  (w_cmd>0 && raw_sonar_mm[31:16]<=16'd200) ||
                  (w_cmd<0 && raw_sonar_mm[47:32]<=16'd200);
    wire cpu_running=running && mot4_mode && local_ready && all_ranges_ok && !raw_stop && accepted_motion!=0;
    assign status_flags={mot4_mode && local_ready,configured,watchdog,armed};
    reg signed [15:0] cpu_applied_left,cpu_applied_right;
    assign applied_left=mot4_mode ? (cpu_running ? cpu_applied_left : 16'sd0) :
        (direction[0] ? -$signed(magnitude[0]) : $signed(magnitude[0]));
    assign applied_right=mot4_mode ? (cpu_running ? cpu_applied_right : 16'sd0) :
        (direction[1] ? -$signed(magnitude[1]) : $signed(magnitude[1]));
    assign status_data={mot4_mode ? 32'h41434b34 : (protocol_mode==3 ? 32'h41434b33 : (protocol_mode==2 ? 32'h41434b32 : 32'h41434b31)),
                        5'd0,protocol_mode,4'd0,status_flags,last_sequence,
                        applied_left,applied_right,4'd0,left_mask,4'd0,invert_mask};
    assign control_flags={cpu_trap,watchdog,armed,configured,mot4_mode};
    assign control_sequence=last_sequence;
    assign control_motion=accepted_motion;
    assign control_masks={invert_mask,left_mask};
    function signed [15:0] limited;
        input signed [15:0] value;
        begin
            if(value>MAX_PERMILLE) limited=MAX_PERMILLE;
            else if(value < -MAX_PERMILLE) limited=-MAX_PERMILLE;
            else limited=value;
        end
    endfunction
    // Real hardware: 2500 cycles * permille / 1000 = floor(2.5*permille).
    // Express this as a narrow shift/add, avoiding a 32-bit divider in the PWM path.
    function [PWM_BITS:0] duty_cycles;
        input [9:0] permille;
        begin
            if(PWM_CYCLES==2500) duty_cycles={permille,1'b0}+{1'b0,permille[9:1]};
            else if(PWM_CYCLES==100) duty_cycles=permille/10;
            else duty_cycles=(PWM_CYCLES*permille)/1000;
        end
    endfunction
    // Stable equal signed targets mean straight forward/reverse. A turn,
    // startup, reversal, stop or command loss immediately bypasses correction.
    wire [15:0] absolute_target=target[0][15] ? -target[0] : target[0];
    wire sync_enable=running && target[0]==target[1] && target[0]!=0 &&
        magnitude[0]>=200 && magnitude[0]==absolute_target && magnitude[1]==absolute_target &&
        direction[0]==target[0][15] && direction[1]==target[1][15] &&
        dead_count[0]==0 && dead_count[1]==0;
    wire [47:0] wheel_trims;
    wire sync_active,sync_fault;
    generate if(ENCODER_SYNC_ENABLE)begin: encoder_feedback
        motor_encoder_sync #(.CLK_HZ(CLK_HZ)) balance(
            .clk(clk),.rst_n(rst_n),.enable(sync_enable),.reverse(direction[0]),
            .base_duty(magnitude[0][9:0]),.sample_valid(encoder_valid),
            .deltas(encoder_deltas),.illegal(encoder_illegal),
            .trims(wheel_trims),.active(sync_active),.fault(sync_fault));
    end else begin: no_encoder_feedback
        assign wheel_trims=0;assign sync_active=0;assign sync_fault=0;
    end endgenerate
    function [9:0] corrected_duty;
        input [9:0] base;
        input signed [11:0] correction;
        reg signed [12:0] result;
        begin
            result=$signed({3'b0,base})+$signed({correction[11],correction});
            if(result<0)corrected_duty=0;
            else if(result>MAX_PERMILLE)corrected_duty=MAX_PERMILLE[9:0];
            else corrected_duty=result[9:0];
        end
    endfunction
    integer side,motor;
    // The short transport watchdog still stops PWM immediately. It pauses
    // motion, whereas the independent 30-second authority timeout disarms.
    // Only a newly accepted valid/sequence-checked command resets this count.
    wire expire=watchdog_count>=MS_CYCLES*WATCHDOG_MS-1;
    wire authorization_expired=watchdog_count>=MS_CYCLES*AUTHORIZATION_MS-1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            configured<=0;armed<=0;watchdog<=1;have_sequence<=0;protocol_mode<=1;
            last_sequence<=0;accepted_motion<=0;control_epoch<=0;left_mask<=DEFAULT_LEFT_MASK;invert_mask<=DEFAULT_INVERT_MASK;
            watchdog_count<=0;ms_count<=0;ramp_ms<=0;status_ms<=0;
            status_valid<=0;accepted<=0;
            for(side=0;side<2;side=side+1) begin
                target[side]<=0;magnitude[side]<=0;direction[side]<=0;dead_count[side]<=0;
            end
        end else begin
            accepted<=accept;
            status_valid<=accepted;
            if(ms_tick) begin
                ms_count<=0;
                if(ramp_ms==RAMP_STEP_MS-1) ramp_ms<=0;else ramp_ms<=ramp_ms+1'b1;
                if(status_ms==STATUS_MS-1) begin status_ms<=0;status_valid<=1;end
                else status_ms<=status_ms+1'b1;
            end else ms_count<=ms_count+1'b1;
            if(accept) begin
                watchdog_count<=0;watchdog<=0;have_sequence<=1;last_sequence<=command_sequence;
                armed<=packet_armed;configured<=1;protocol_mode<=mot4 ? 3'd4 : (mot3 ? 3'd3 : (mot2 ? 3'd2 : 3'd1));
                accepted_motion<={raw_right_or_forward,raw_left_or_angle};
                if(armed!=packet_armed || protocol_mode!=(mot4 ? 4 : mot3 ? 3 : mot2 ? 2 : 1) ||
                   !configured || (stop_packet && !config_matches)) control_epoch<=control_epoch+1'b1;
                if(stop_packet) begin left_mask<=packet_mask;invert_mask<=packet_invert;end
                target[0]<=limited(packet_left);target[1]<=limited(packet_right);
            end else begin
                if(!authorization_expired)watchdog_count<=watchdog_count+1'b1;
                if(expire)begin
                    watchdog<=1;target[0]<=0;target[1]<=0;
                    // Invalidate the CPU commit once on entry to a pause.
                    if(!watchdog)control_epoch<=control_epoch+1'b1;
                end
                if(authorization_expired)begin
                    armed<=0;configured<=0;have_sequence<=0;
                    if(armed || configured)control_epoch<=control_epoch+1'b1;
                end
            end
            for(side=0;side<2;side=side+1) begin
                if(!running || expire || (accept && (!packet_armed ||
                   (side==0 ? packet_left==0 : packet_right==0)))) begin
                    magnitude[side]<=0;dead_count[side]<=0;
                end else if(target[side]==0) begin
                    magnitude[side]<=0;dead_count[side]<=0;
                end else if(target[side][15]!=direction[side]) begin
                    magnitude[side]<=0;
                    if(dead_count[side]==0) dead_count[side]<=DEAD_CYCLES;
                    else if(dead_count[side]==1) begin
                        direction[side]<=target[side][15];dead_count[side]<=0;
                    end else dead_count[side]<=dead_count[side]-1'b1;
                end else begin
                    dead_count[side]<=0;
                    if(ramp_tick) begin
                        if(magnitude[side] < (target[side][15] ? -target[side] : target[side])) begin
                            if(magnitude[side]+10 >= (target[side][15] ? -target[side] : target[side]))
                                magnitude[side]<=target[side][15] ? -target[side] : target[side];
                            else magnitude[side]<=magnitude[side]+16'd10;
                        end else if(magnitude[side] > (target[side][15] ? -target[side] : target[side])) begin
                            if(magnitude[side] <= (target[side][15] ? -target[side] : target[side])+10)
                                magnitude[side]<=target[side][15] ? -target[side] : target[side];
                            else magnitude[side]<=magnitude[side]-16'd10;
                        end
                    end
                end
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(motor=0;motor<4;motor=motor+1) begin
                pwm_count[motor]<=motor*(PWM_CYCLES/4);threshold[motor]<=0;
            end
        end else begin
            for(motor=0;motor<4;motor=motor+1) begin
                if(pwm_count[motor]==PWM_CYCLES-1) begin
                    pwm_count[motor]<=0;
                    if(ENCODER_SYNC_ENABLE && sync_enable && sync_active && !sync_fault)
                        threshold[motor]<=duty_cycles(corrected_duty(magnitude[left_mask[motor] ? 0 : 1][9:0],
                            $signed(wheel_trims[motor*12 +:12])));
                    else threshold[motor]<=duty_cycles(magnitude[left_mask[motor] ? 0 : 1][9:0]);
                end else pwm_count[motor]<=pwm_count[motor]+1'b1;
            end
        end
    end
    // Four independent physical wheel actuators for the local PI. Electrical
    // inversion is applied only at the pins; encoder signs stay in firmware.
    reg [15:0] cpu_magnitude[0:3];
    reg [3:0] cpu_direction;
    reg [31:0] cpu_dead[0:3];
    reg [PWM_BITS:0] cpu_threshold[0:3];
    wire [3:0] cpu_active;
    genvar c;
    generate for(c=0;c<4;c=c+1)begin:local_wheel
        wire signed [15:0] desired=limited($signed(cpu_pwm[c*16 +:16]));
        wire [15:0] desired_abs=desired[15] ? -desired : desired;
        assign cpu_active[c]=cpu_running && desired!=0 && cpu_magnitude[c]!=0 &&
            cpu_dead[c]==0 && desired[15]==cpu_direction[c];
        always @(posedge clk or negedge rst_n)begin
            if(!rst_n)begin cpu_magnitude[c]<=0;cpu_direction[c]<=0;cpu_dead[c]<=0;cpu_threshold[c]<=0;end
            else begin
                if(!cpu_running || expire || (accept && (!packet_armed || !mot4)))begin
                    cpu_magnitude[c]<=0;cpu_dead[c]<=0;
                end else if(desired==0)begin cpu_magnitude[c]<=0;cpu_dead[c]<=0;end
                else if(desired[15]!=cpu_direction[c])begin
                    cpu_magnitude[c]<=0;
                    if(cpu_dead[c]==0)cpu_dead[c]<=DEAD_CYCLES;
                    else if(cpu_dead[c]==1)begin cpu_direction[c]<=desired[15];cpu_dead[c]<=0;end
                    else cpu_dead[c]<=cpu_dead[c]-1'b1;
                end else begin cpu_dead[c]<=0;cpu_magnitude[c]<=desired_abs;end
                if(pwm_count[c]==PWM_CYCLES-1)cpu_threshold[c]<=duty_cycles(cpu_magnitude[c][9:0]);
            end
        end
    end endgenerate
    // Telemetry arithmetic never shares a single path with the independent
    // CPU/IMU/raw-sonar safety gates. Capture each physical signed wheel,
    // reduce sides on the next clock, then divide on the following clock.
    // The public applied outputs are still forced zero immediately on a stop.
    reg signed [12:0] reported_wheel[0:3];
    reg [3:0] reported_mask;
    reg signed [13:0] sum_left,sum_right,saved_sum_left,saved_sum_right;
    reg [2:0] count_left,count_right,saved_count_left,saved_count_right;
    integer i,j;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            reported_mask<=0;
            for(j=0;j<4;j=j+1)reported_wheel[j]<=0;
        end else begin
            reported_mask<=left_mask;
            for(j=0;j<4;j=j+1)begin
                if(cpu_active[j])reported_wheel[j]<=cpu_direction[j] ?
                    -$signed({1'b0,cpu_magnitude[j][11:0]}) : $signed({1'b0,cpu_magnitude[j][11:0]});
                else reported_wheel[j]<=0;
            end
        end
    end
    always @* begin
        sum_left=0;sum_right=0;count_left=0;count_right=0;
        for(i=0;i<4;i=i+1)begin
            if(reported_mask[i])begin
                count_left=count_left+1'b1;sum_left=sum_left+reported_wheel[i];
            end else begin
                count_right=count_right+1'b1;sum_right=sum_right+reported_wheel[i];
            end
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            saved_sum_left<=0;saved_sum_right<=0;saved_count_left<=0;saved_count_right<=0;
            cpu_applied_left<=0;cpu_applied_right<=0;
        end else begin
            saved_sum_left<=sum_left;saved_sum_right<=sum_right;
            saved_count_left<=count_left;saved_count_right<=count_right;
            case(saved_count_left)1:cpu_applied_left<=saved_sum_left;2:cpu_applied_left<=saved_sum_left/2;3:cpu_applied_left<=saved_sum_left/3;default:cpu_applied_left<=0;endcase
            case(saved_count_right)1:cpu_applied_right<=saved_sum_right;2:cpu_applied_right<=saved_sum_right/2;3:cpu_applied_right<=saved_sum_right/3;default:cpu_applied_right<=0;endcase
        end
    end
    genvar n;
    generate for(n=0;n<4;n=n+1) begin:g_output
        wire which_side=left_mask[n] ? 1'b0 : 1'b1;
        wire active=running && magnitude[which_side]!=0 && target[which_side]!=0 &&
                    dead_count[which_side]==0 && target[which_side][15]==direction[which_side];
        wire backwards=direction[which_side]^invert_mask[n];
        assign motor_pwm[n]=mot4_mode ? (cpu_active[n] && pwm_count[n]<cpu_threshold[n]) : (active && pwm_count[n]<threshold[n]);
        assign motor_in1[n]=mot4_mode ? (cpu_active[n] && !(cpu_direction[n]^invert_mask[n])) : (active && !backwards);
        assign motor_in2[n]=mot4_mode ? (cpu_active[n] && (cpu_direction[n]^invert_mask[n])) : (active && backwards);
    end endgenerate
endmodule
