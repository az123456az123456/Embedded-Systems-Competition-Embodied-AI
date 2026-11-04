`timescale 1ns/1ps
// Signed PWM feed-forward with optional bounded encoder balancing on straight motion.
// Sequence freshness and watchdog depend only on newly accepted MOT1/MOT2 commands.
module motor_remote_control_v20_reference #(
    parameter integer CLK_HZ=50000000, PWM_HZ=20000,
    parameter integer WATCHDOG_MS=500, RAMP_STEP_MS=10,
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
    output wire signed [15:0] applied_left,applied_right
);
    localparam integer MS_CYCLES=CLK_HZ/1000;
    localparam integer PWM_CYCLES=CLK_HZ/PWM_HZ;
    localparam integer PWM_BITS=$clog2(PWM_CYCLES);
    localparam integer DEAD_CYCLES=MS_CYCLES*REVERSAL_MS;
    reg configured,armed,watchdog,have_sequence,angle_mode;
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
    wire [15:0] incoming_abs=command_data[47] ? -command_data[47:32] : command_data[47:32];
    wire [12:0] bounded_angle=incoming_abs>16'd5500 ? 13'd5500 : incoming_abs[12:0];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin decoded_valid<=0;decoded_data<=0;divided_angle<=0;end
        else begin
            decoded_valid<=command_valid;
            if(command_valid)begin
                decoded_data<=command_data;
                divided_angle<=bounded_angle/13'd25;
            end
        end
    end
    wire packet_armed=decoded_data[80];
    wire [31:0] command_sequence=decoded_data[79:48];
    wire mot1=decoded_data[127:96]==32'h4d4f5431 && decoded_data[95:88]==1;
    wire mot2=decoded_data[127:96]==32'h4d4f5432 && decoded_data[95:88]==2;
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
    wire signed [15:0] packet_left=mot2 ? angle_forward+angle_steer : raw_left_or_angle;
    wire signed [15:0] packet_right=mot2 ? angle_forward-angle_steer : raw_right_or_forward;
    wire [3:0] packet_mask=decoded_data[11:8];
    wire [3:0] packet_invert=decoded_data[3:0];
    wire [31:0] sequence_delta=command_sequence-last_sequence;
    wire newer=!have_sequence || (sequence_delta!=0 && !sequence_delta[31]);
    wire stop_packet=!packet_armed && raw_left_or_angle==0 && raw_right_or_forward==0;
    wire values_ok=mot2 ? (raw_left_or_angle>=-16'sd9000 && raw_left_or_angle<=16'sd9000 &&
                          raw_right_or_forward>=0 && raw_right_or_forward<=16'sd220) :
                         (raw_left_or_angle>=-1000 && raw_left_or_angle<=1000 &&
                          raw_right_or_forward>=-1000 && raw_right_or_forward<=1000);
    wire layout_ok=packet_mask!=0 && packet_mask!=15 && decoded_data[15:12]==0 &&
                   decoded_data[7:4]==0;
    wire format_ok=(mot1 || mot2) && decoded_data[87:81]==0;
    // An armed packet cannot silently change which physical wheel is driven.
    wire config_matches=packet_mask==left_mask && packet_invert==invert_mask;
    wire accept=decoded_valid && newer && values_ok && layout_ok && format_ok &&
        (stop_packet || (packet_armed && configured && config_matches));
    wire running=configured && armed && !watchdog;
    assign status_flags={1'b0,configured,watchdog,armed};
    assign applied_left=direction[0] ? -$signed(magnitude[0]) : $signed(magnitude[0]);
    assign applied_right=direction[1] ? -$signed(magnitude[1]) : $signed(magnitude[1]);
    assign status_data={angle_mode ? 32'h41434b32 : 32'h41434b31,
                        angle_mode ? 8'd2 : 8'd1,4'd0,status_flags,last_sequence,
                        applied_left,applied_right,4'd0,left_mask,4'd0,invert_mask};
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
    wire expire=watchdog_count>=MS_CYCLES*WATCHDOG_MS-1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            configured<=0;armed<=0;watchdog<=1;have_sequence<=0;angle_mode<=0;
            last_sequence<=0;left_mask<=DEFAULT_LEFT_MASK;invert_mask<=DEFAULT_INVERT_MASK;
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
                armed<=packet_armed;configured<=1;angle_mode<=mot2;
                if(stop_packet) begin left_mask<=packet_mask;invert_mask<=packet_invert;end
                target[0]<=limited(packet_left);target[1]<=limited(packet_right);
            end else if(expire) begin
                watchdog<=1;armed<=0;configured<=0;have_sequence<=0;
                target[0]<=0;target[1]<=0;
            end else watchdog_count<=watchdog_count+1'b1;
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
    genvar n;
    generate for(n=0;n<4;n=n+1) begin:g_output
        wire which_side=left_mask[n] ? 1'b0 : 1'b1;
        wire active=running && magnitude[which_side]!=0 && target[which_side]!=0 &&
                    dead_count[which_side]==0 && target[which_side][15]==direction[which_side];
        wire backwards=direction[which_side]^invert_mask[n];
        assign motor_pwm[n]=active && pwm_count[n]<threshold[n];
        assign motor_in1[n]=active && !backwards;
        assign motor_in2[n]=active && backwards;
    end endgenerate
endmodule
