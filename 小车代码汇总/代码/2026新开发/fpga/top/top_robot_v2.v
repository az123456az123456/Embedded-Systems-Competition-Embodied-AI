// ============================================================
// top_robot_v2.v — 协议 v2 双电机 FPGA 顶层
//   UART + CRC16 + ACK/STATUS + 看门狗
//   双编码器 + 双 PID + 差速运动 + 超声波避障 + IMU
// ============================================================
module top_robot_v2 #(
    parameter CLK_FREQ       = 50_000_000,
    parameter BAUD           = 115200,
    parameter TRIG_PERIOD    = 3_000_000,
    parameter REPORT_PERIOD  = 5_000_000,
    parameter WATCHDOG_MS    = 500,
    parameter OBSTACLE_CM    = 16'd20,
    parameter SLOW_CM        = 16'd40,
    parameter V_MAX_MM_S     = 3000,
    parameter TRACK_WIDTH_MM = 160,
    parameter TARGET_MAX     = 1000,
    parameter KP             = 8,
    parameter KI             = 1,
    parameter KD             = 2,
    parameter PID_DIV        = 100
)(
    input  wire               clk,
    input  wire               rst_n,
    // UART
    input  wire               uart_rx,
    output wire               uart_tx,
    // 双编码器
    input  wire               enc_left_a,
    input  wire               enc_left_b,
    input  wire               enc_right_a,
    input  wire               enc_right_b,
    // 双电机
    output wire               motor_left_pwm,
    output wire               motor_left_dir,
    output wire               motor_right_pwm,
    output wire               motor_right_dir,
    // 超声波 / IMU
    input  wire               echo,
    output wire               trig,
    inout  wire               imu_sda,
    output wire               imu_scl,
    // 调试
    output wire signed [15:0] o_target_left,
    output wire signed [15:0] o_target_right,
    output wire signed [15:0] o_speed_left,
    output wire signed [15:0] o_speed_right,
    output wire [15:0]        o_distance_cm,
    output wire [3:0]         o_mode,
    output wire [15:0]        o_flags,
    output wire               o_watchdog
);

    localparam CMD_SET_TWIST = 8'h01;
    localparam CMD_STOP      = 8'h02;
    localparam CMD_SET_MODE  = 8'h03;
    localparam CMD_CLR_FAULT = 8'h04;
    localparam CMD_HEARTBEAT = 8'h10;
    localparam CMD_STATUS    = 8'h81;
    localparam CMD_ACK       = 8'h82;

    localparam MODE_MANUAL   = 4'd0;
    localparam MODE_OBSTACLE = 4'd1;
    localparam MODE_VISION   = 4'd2;
    localparam MODE_IDLE     = 4'd3;

    // ---------------- UART ----------------
    wire [7:0] uart_rx_data;
    wire       uart_rx_valid;
    wire       proto_tx_start;
    wire [7:0] proto_tx_data;
    wire       uart_tx_busy;
    wire       proto_tx_busy;
    wire       proto_tx_done;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .rx_valid(uart_rx_valid), .rx_data(uart_rx_data)
    );

    // ---------------- 协议 RX ----------------
    wire [7:0]   rx_cmd;
    wire [7:0]   rx_seq;
    wire [7:0]   rx_len;
    wire [255:0] rx_payload;
    wire         rx_frame_valid;
    wire         rx_frame_error;

    protocol_rx u_proto_rx (
        .clk(clk), .rst_n(rst_n),
        .rx_valid(uart_rx_valid), .rx_data(uart_rx_data),
        .o_cmd(rx_cmd), .o_seq(rx_seq), .o_len(rx_len),
        .o_payload(rx_payload), .o_frame_valid(rx_frame_valid),
        .o_frame_error(rx_frame_error)
    );

    // ---------------- 运动目标与安全状态 ----------------
    reg signed [15:0] cmd_v_mm_s;
    reg signed [15:0] cmd_w_mrad_s;
    reg [3:0]  mode;
    reg        stop_latched;
    reg        imu_valid_latched;

    wire watchdog_timeout;
    comm_watchdog #(
        .CLK_FREQ(CLK_FREQ),
        .TIMEOUT_MS(WATCHDOG_MS)
    ) u_watchdog (
        .clk(clk), .rst_n(rst_n),
        .kick(rx_frame_valid),
        .timeout(watchdog_timeout)
    );

    // ---------------- 超声波 ----------------
    wire [15:0] distance_cm;
    wire        us_busy;
    wire        us_done;
    reg         us_measured;
    reg         us_trigger;
    reg  [31:0] trig_cnt;
    wire        us_valid = us_measured && (distance_cm != 16'hFFFF);
    wire        obstacle = us_valid && (distance_cm < OBSTACLE_CM);
    wire        near_obstacle = us_valid && (distance_cm < SLOW_CM);
    wire        hard_stop = stop_latched || watchdog_timeout;
    wire        mode_idle = (mode == MODE_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            us_trigger <= 1'b0;
            trig_cnt   <= 32'd0;
        end else begin
            us_trigger <= 1'b0;
            if (us_busy) begin
                trig_cnt <= 32'd0;
            end else if (trig_cnt == TRIG_PERIOD - 1) begin
                trig_cnt   <= 32'd0;
                us_trigger <= 1'b1;
            end else begin
                trig_cnt <= trig_cnt + 32'd1;
            end
        end
    end

    ultrasonic #(.CLK_FREQ(CLK_FREQ), .TRIG_US(10), .TIMEOUT_US(10_000)) u_us (
        .clk(clk), .rst_n(rst_n), .trigger(us_trigger), .echo(echo),
        .trig(trig), .distance_cm(distance_cm), .done(us_done), .busy(us_busy)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            us_measured <= 1'b0;
        else if (us_done)
            us_measured <= 1'b1;
    end

    // ---------------- IMU ----------------
    wire signed [15:0] accel_x, accel_y, accel_z;
    wire imu_data_valid;
    imu_mpu6050 #(.I2C_ADDR(7'h68)) u_imu (
        .clk(clk), .rst_n(rst_n), .sda(imu_sda), .scl(imu_scl),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .data_valid(imu_data_valid), .busy()
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            imu_valid_latched <= 1'b0;
        else if (imu_data_valid)
            imu_valid_latched <= 1'b1;
    end

    // ---------------- 下行命令 ----------------
    reg [7:0] ack_pending;
    reg [7:0] ack_cmd;
    reg [7:0] ack_code;
    reg [7:0] ack_detail;
    reg       ack_take;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_v_mm_s  <= 16'sd0;
            cmd_w_mrad_s <= 16'sd0;
            mode        <= MODE_IDLE;
            stop_latched <= 1'b0;
            ack_pending <= 8'd0;
            ack_cmd     <= 8'd0;
            ack_code    <= 8'd0;
            ack_detail  <= 8'd0;
        end else begin
            if (rx_frame_valid) begin
                ack_pending <= 8'd1;
                ack_cmd     <= rx_cmd;
                ack_code    <= 8'd0;
                ack_detail  <= rx_len;
                case (rx_cmd)
                    CMD_SET_TWIST: begin
                        if (rx_len == 8'd4) begin
                            cmd_v_mm_s   <= rx_payload[15:0];
                            cmd_w_mrad_s <= rx_payload[31:16];
                            stop_latched <= 1'b0;
                        end else begin
                            ack_code <= 8'd2;   // BAD_PAYLOAD
                        end
                    end
                    CMD_STOP: begin
                        cmd_v_mm_s   <= 16'sd0;
                        cmd_w_mrad_s <= 16'sd0;
                        if (rx_len != 8'd1) begin
                            ack_code <= 8'd2;
                        end else begin
                            if (rx_payload[7:0] == 8'd1 || rx_payload[7:0] == 8'd3)
                                stop_latched <= 1'b1;
                            else
                                stop_latched <= 1'b0;
                        end
                    end
                    CMD_SET_MODE: begin
                        if (rx_len == 8'd1 && rx_payload[7:0] <= MODE_IDLE)
                            mode <= rx_payload[3:0];
                        else
                            ack_code <= 8'd2;
                    end
                    CMD_CLR_FAULT: begin
                        if (rx_len == 8'd0)
                            stop_latched <= 1'b0;
                        else
                            ack_code <= 8'd2;
                    end
                    CMD_HEARTBEAT: begin
                        if (rx_len != 8'd0)
                            ack_code <= 8'd2;
                    end
                    default: begin
                        ack_code <= 8'd1;       // BAD_COMMAND
                    end
                endcase
            end
            if (rx_frame_error && !rx_frame_valid) begin
                ack_pending <= 8'd1;
                ack_cmd     <= 8'hFF;
                ack_code    <= 8'd4;           // CRC_ERROR
                ack_detail  <= 8'd0;
            end
            if (ack_take)
                ack_pending <= 8'd0;
        end
    end

    // 障碍分级：危险直接停并禁用 PID；接近时速度/转向减半。
    wire signed [15:0] safe_v = (hard_stop || mode_idle) ? 16'sd0 :
                               (obstacle ? 16'sd0 :
                               (near_obstacle ? (cmd_v_mm_s >>> 1) : cmd_v_mm_s));
    wire signed [15:0] safe_w = (hard_stop || mode_idle) ? 16'sd0 :
                               (obstacle ? 16'sd0 :
                               (near_obstacle ? (cmd_w_mrad_s >>> 1) : cmd_w_mrad_s));
    wire motion_enable = !hard_stop && !mode_idle && !obstacle;

    wire signed [15:0] target_left;
    wire signed [15:0] target_right;
    twist_to_wheel #(
        .V_MAX_MM_S(V_MAX_MM_S),
        .TRACK_WIDTH_MM(TRACK_WIDTH_MM),
        .TARGET_MAX(TARGET_MAX)
    ) u_twist (
        .v_mm_s(safe_v),
        .w_mrad_s(safe_w),
        .target_left(target_left),
        .target_right(target_right)
    );

    // ---------------- 双编码器 + 双 PID + 双 PWM ----------------
    wire left_pulse, left_dir, right_pulse, right_dir;
    wire signed [15:0] left_speed, right_speed;
    wire signed [15:0] left_duty, right_duty;

    quadrature_decoder #(.WIDTH(16)) u_enc_left (
        .clk(clk), .rst_n(rst_n), .enc_a(enc_left_a), .enc_b(enc_left_b),
        .position(), .pulse_tick(left_pulse), .direction(left_dir)
    );
    quadrature_decoder #(.WIDTH(16)) u_enc_right (
        .clk(clk), .rst_n(rst_n), .enc_a(enc_right_a), .enc_b(enc_right_b),
        .position(), .pulse_tick(right_pulse), .direction(right_dir)
    );
    encoder_velocity #(.WINDOW(16'd5000)) u_vel_left (
        .clk(clk), .rst_n(rst_n), .pulse_tick(left_pulse),
        .direction(left_dir), .speed(left_speed)
    );
    encoder_velocity #(.WINDOW(16'd5000)) u_vel_right (
        .clk(clk), .rst_n(rst_n), .pulse_tick(right_pulse),
        .direction(right_dir), .speed(right_speed)
    );
    motor_pid #(
        .KP(KP), .KI(KI), .KD(KD),
        .OUT_MAX(TARGET_MAX), .CONTROL_DIV(PID_DIV)
    ) u_pid_left (
        .clk(clk), .rst_n(rst_n), .enable(motion_enable),
        .target(target_left), .actual(left_speed), .duty(left_duty)
    );
    motor_pid #(
        .KP(KP), .KI(KI), .KD(KD),
        .OUT_MAX(TARGET_MAX), .CONTROL_DIV(PID_DIV)
    ) u_pid_right (
        .clk(clk), .rst_n(rst_n), .enable(motion_enable),
        .target(target_right), .actual(right_speed), .duty(right_duty)
    );
    motor_pwm #(.TOP(TARGET_MAX)) u_pwm_left (
        .clk(clk), .rst_n(rst_n), .duty(left_duty),
        .pwm(motor_left_pwm), .dir(motor_left_dir)
    );
    motor_pwm #(.TOP(TARGET_MAX)) u_pwm_right (
        .clk(clk), .rst_n(rst_n), .duty(right_duty),
        .pwm(motor_right_pwm), .dir(motor_right_dir)
    );

    // ---------------- STATUS / ACK 上报 ----------------
    reg [31:0] rep_cnt;
    reg        status_due;
    reg [7:0]  tx_seq;
    reg [7:0]  resp_cmd;
    reg [7:0]  resp_len;
    reg [255:0] resp_payload;
    reg        resp_start;

    wire moving = (left_speed != 16'sd0) || (right_speed != 16'sd0);
    reg [15:0] status_flags;
    always @(*) begin
        status_flags      = 16'd0;
        status_flags[0]   = ~watchdog_timeout;
        status_flags[1]   = moving;
        status_flags[2]   = obstacle;
        status_flags[3]   = stop_latched || watchdog_timeout;
        status_flags[4]   = imu_valid_latched;
        status_flags[5]   = us_valid;
        status_flags[6]   = watchdog_timeout;
        status_flags[7]   = stop_latched;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            resp_start <= 1'b0;
            resp_cmd   <= 8'd0;
            resp_len   <= 8'd0;
            resp_payload <= 256'd0;
            tx_seq     <= 8'd0;
            rep_cnt    <= 32'd0;
            status_due <= 1'b0;
            ack_take   <= 1'b0;
        end else begin
            resp_start <= 1'b0;
            ack_take   <= 1'b0;
            if (rep_cnt == REPORT_PERIOD - 1)
                rep_cnt <= 32'd0;
            else
                rep_cnt <= rep_cnt + 32'd1;

            if (!proto_tx_busy) begin
                if (status_due || rep_cnt == REPORT_PERIOD - 1) begin
                    status_due <= 1'b0;
                    resp_cmd   <= CMD_STATUS;
                    resp_len   <= 8'd12;
                    resp_payload <= 256'd0;
                    resp_payload[15:0]   <= left_speed;
                    resp_payload[31:16]  <= right_speed;
                    resp_payload[47:32]  <= distance_cm;
                    resp_payload[63:48]  <= 16'd0;      // yaw 暂未融合
                    resp_payload[79:64]  <= status_flags;
                    resp_payload[95:80]  <= 16'd0;
                    tx_seq     <= tx_seq + 8'd1;
                    resp_start <= 1'b1;
                end else if (ack_pending) begin
                    ack_take   <= 1'b1;
                    resp_cmd   <= CMD_ACK;
                    resp_len   <= 8'd3;
                    resp_payload <= 256'd0;
                    resp_payload[7:0]   <= ack_cmd;
                    resp_payload[15:8]  <= ack_code;
                    resp_payload[23:16] <= ack_detail;
                    resp_start <= 1'b1;
                end
            end else if (rep_cnt == REPORT_PERIOD - 1) begin
                status_due <= 1'b1;
            end
        end
    end

    protocol_tx u_proto_tx (
        .clk(clk), .rst_n(rst_n),
        .start(resp_start), .cmd(resp_cmd), .seq(tx_seq),
        .len(resp_len), .payload(resp_payload),
        .tx_start(proto_tx_start), .tx_data(proto_tx_data),
        .tx_busy(uart_tx_busy), .busy(proto_tx_busy), .done(proto_tx_done)
    );
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart_tx (
        .clk(clk), .rst_n(rst_n),
        .tx_start(proto_tx_start), .tx_data(proto_tx_data),
        .tx(uart_tx), .tx_busy(uart_tx_busy)
    );

    assign o_target_left  = target_left;
    assign o_target_right = target_right;
    assign o_speed_left   = left_speed;
    assign o_speed_right  = right_speed;
    assign o_distance_cm  = distance_cm;
    assign o_mode         = mode;
    assign o_flags        = status_flags;
    assign o_watchdog     = watchdog_timeout;

endmodule
