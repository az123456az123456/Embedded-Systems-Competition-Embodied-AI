// ============================================================
// top_robot.v — 具身智能机器人 FPGA 顶层（集成五大模块）
//   ① UART 收发(与RK3588通信)  ② 编码器解码  ③ 电机PWM+PID
//   ④ 超声波避障               ⑤ IMU(MPU6050)姿态
// 协议见 ../通信协议.md
// ============================================================
module top_robot #(
    parameter CLK_FREQ      = 50_000_000,
    parameter BAUD          = 115200,
    parameter TRIG_PERIOD   = 3_000_000,    // 超声波周期 60ms
    parameter REPORT_PERIOD = 5_000_000,    // 状态上报 100ms
    parameter OBSTACLE_CM   = 16'd20,       // 停车阈值
    parameter SLOW_CM       = 16'd40        // 减速阈值
)(
    input  wire               clk,
    input  wire               rst_n,
    // ---- UART ----
    input  wire               uart_rx,
    output wire               uart_tx,
    // ---- 编码器 ----
    input  wire               enc_a,
    input  wire               enc_b,
    // ---- 电机 ----
    output wire               motor_pwm,
    output wire               motor_dir,
    // ---- 超声波 ----
    input  wire               echo,
    output wire               trig,
    // ---- IMU ----
    inout  wire               imu_sda,
    output wire               imu_scl,
    // ---- 调试/状态输出(可接逻辑分析仪或数码管) ----
    output wire signed [15:0] o_target,
    output wire signed [15:0] o_speed,
    output wire [15:0]        o_distance,
    output wire [3:0]         o_mode
);

    // ================= 内部信号 =================
    wire [7:0]  rx_data;
    wire        rx_valid;
    reg  [7:0]  tx_data;
    reg         tx_start;
    wire        tx_busy;

    reg  signed [15:0] target_speed;
    reg  [3:0]  mode;

    wire signed [15:0] enc_speed;
    wire        enc_pulse, enc_dir;
    wire signed [15:0] duty;

    wire [15:0] distance_cm;
    wire        us_busy;
    reg         us_trigger;
    reg  [31:0] trig_cnt;

    // ================= UART =================
    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .rx_valid(rx_valid), .rx_data(rx_data)
    );
    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .tx_start(tx_start), .tx_data(tx_data),
        .tx(uart_tx), .tx_busy(tx_busy)
    );

    // ================= 下行命令解析 =================
    // 帧: AA CMD DH DL SUM
    reg [7:0] fb [0:4];
    reg [2:0] fi;
    wire [7:0] fsum = fb[0] ^ fb[1] ^ fb[2] ^ fb[3];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fi <= 3'd0;
            target_speed <= 16'sd0;
            mode <= 4'd0;
        end else begin
            if (rx_valid) begin
                if (fi == 3'd0) begin
                    if (rx_data == 8'hAA) begin fb[0] <= rx_data; fi <= 3'd1; end
                end else if (fi <= 3'd3) begin
                    fb[fi] <= rx_data;
                    fi <= fi + 3'd1;
                end else begin       // fi == 4, 校验字节
                    fi <= 3'd0;
                    if (rx_data == fsum) begin
                        case (fb[1])
                            8'h01: target_speed <= {fb[2], fb[3]};          // SET_SPEED
                            8'h02: target_speed <= 16'sd0;                   // STOP
                            8'h03: mode <= {4'b0, fb[3][3:0]};               // SET_MODE
                            default: ;                                       // GET_STATUS 预留
                        endcase
                    end
                end
            end
        end
    end

    // ================= 安全逻辑(避障) =================
    wire us_valid = (distance_cm != 16'hFFFF);
    wire obstacle = mode[0] && us_valid && (distance_cm < OBSTACLE_CM);
    wire near_obs = mode[0] && us_valid && (distance_cm < SLOW_CM);
    wire signed [15:0] eff_target = obstacle ? 16'sd0
                                 : (near_obs ? (target_speed >>> 1) : target_speed);

    // ================= 编码器 + 测速 =================
    quadrature_decoder #(.WIDTH(16)) u_enc (
        .clk(clk), .rst_n(rst_n), .enc_a(enc_a), .enc_b(enc_b),
        .position(), .pulse_tick(enc_pulse), .direction(enc_dir)
    );
    encoder_velocity #(.WINDOW(16'd5000)) u_vel (
        .clk(clk), .rst_n(rst_n), .pulse_tick(enc_pulse),
        .direction(enc_dir), .speed(enc_speed)
    );

    // ================= PID + 电机 =================
    // ⚠️ 关键: 障碍急停时必须 disable PID -> 清积分(否则 windup 导致停不下来)
    motor_pid #(.KP(8), .KI(1), .KD(2), .OUT_MAX(1000), .CONTROL_DIV(100)) u_pid (
        .clk(clk), .rst_n(rst_n), .enable(~obstacle),
        .target(eff_target), .actual(enc_speed), .duty(duty)
    );
    motor_pwm #(.TOP(16'd1000)) u_pwm (
        .clk(clk), .rst_n(rst_n), .duty(duty),
        .pwm(motor_pwm), .dir(motor_dir)
    );

    // ================= 超声波周期触发 =================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin trig_cnt <= 32'd0; us_trigger <= 1'b0; end
        else begin
            us_trigger <= 1'b0;
            if (us_busy) begin
                trig_cnt <= 32'd0;
            end else if (trig_cnt == TRIG_PERIOD - 1) begin
                trig_cnt <= 32'd0;
                us_trigger <= 1'b1;
            end else begin
                trig_cnt <= trig_cnt + 32'd1;
            end
        end
    end

    ultrasonic #(.CLK_FREQ(CLK_FREQ), .TRIG_US(10), .TIMEOUT_US(10_000)) u_us (
        .clk(clk), .rst_n(rst_n), .trigger(us_trigger), .echo(echo),
        .trig(trig), .distance_cm(distance_cm), .done(), .busy(us_busy)
    );

    // ================= IMU =================
    wire signed [15:0] accel_x, accel_y, accel_z;
    imu_mpu6050 #(.I2C_ADDR(7'h68)) u_imu (
        .clk(clk), .rst_n(rst_n), .sda(imu_sda), .scl(imu_scl),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .data_valid(), .busy()
    );

    // ================= 上行状态上报(100ms) =================
    reg [31:0] rep_cnt;
    reg [2:0]  si;
    reg        sending;
    reg [7:0]  sbuf [0:6];
    wire [7:0] stat_sum = 8'h55 ^ 8'h01 ^ enc_speed[15:8] ^ enc_speed[7:0]
                        ^ distance_cm[15:8] ^ distance_cm[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rep_cnt <= 32'd0; si <= 3'd0; sending <= 1'b0; tx_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            if (!sending) begin
                if (rep_cnt == REPORT_PERIOD - 1) begin
                    rep_cnt  <= 32'd0;
                    sbuf[0]  <= 8'h55;
                    sbuf[1]  <= 8'h01;
                    sbuf[2]  <= enc_speed[15:8];
                    sbuf[3]  <= enc_speed[7:0];
                    sbuf[4]  <= distance_cm[15:8];
                    sbuf[5]  <= distance_cm[7:0];
                    sbuf[6]  <= stat_sum;
                    sending  <= 1'b1;
                    si       <= 3'd0;
                end else begin
                    rep_cnt <= rep_cnt + 32'd1;
                end
            end else begin
                if (!tx_busy && !tx_start) begin
                    if (si <= 3'd6) begin
                        tx_data  <= sbuf[si];
                        tx_start <= 1'b1;
                        si       <= si + 3'd1;
                    end else begin
                        sending <= 1'b0;
                    end
                end
            end
        end
    end

    // ================= 调试输出 =================
    assign o_target   = eff_target;
    assign o_speed    = enc_speed;
    assign o_distance = distance_cm;
    assign o_mode     = mode;
endmodule
