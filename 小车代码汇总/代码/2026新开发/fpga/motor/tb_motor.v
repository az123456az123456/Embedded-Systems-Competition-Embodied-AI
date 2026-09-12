// ============================================================
// tb_motor.v — 电机控制模块测试
// 1) PWM 占空比测试  2) 编码器测速测试  3) PID 闭环收敛测试
// ============================================================
`timescale 1ns/1ps
module tb_motor;

    reg clk = 0;
    reg rst_n = 0;
    always #10 clk = ~clk;   // 50MHz

    integer err = 0;
    integer i;

    // ---- 1) PWM 测试 ----
    reg signed [15:0] pwm_duty;
    wire pwm, dir;
    motor_pwm #(.TOP(1000)) u_pwm(
        .clk(clk), .rst_n(rst_n), .duty(pwm_duty), .pwm(pwm), .dir(dir));

    // ---- 2) 测速测试 ----
    reg pulse = 0, enc_dir = 0;
    wire signed [15:0] vel;
    encoder_velocity #(.WINDOW(500)) u_vel(
        .clk(clk), .rst_n(rst_n), .pulse_tick(pulse), .direction(enc_dir), .speed(vel));

    // ---- 电机模型信号(先声明后使用) ----
    reg signed [15:0] speed = 0;
    reg [15:0] ctrl_cnt = 0;
    wire ctrl_tick = (ctrl_cnt == 16'd99);

    // ---- 3) PID 测试 ----
    reg enable = 0;
    reg signed [15:0] target = 0;
    wire signed [15:0] pid_duty;
    motor_pid #(.KP(8), .KI(1), .KD(2), .OUT_MAX(1000), .CONTROL_DIV(100)) u_pid(
        .clk(clk), .rst_n(rst_n), .enable(enable), .target(target), .actual(speed), .duty(pid_duty));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ctrl_cnt <= 16'd0;
        else if (ctrl_tick) ctrl_cnt <= 16'd0;
        else ctrl_cnt <= ctrl_cnt + 16'd1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) speed <= 16'sd0;
        else if (ctrl_tick) speed <= speed + ((pid_duty - speed) * 2) / 16;
    end

    // ================= 测试流程 =================
    initial begin
        $dumpfile("motor_wave.vcd");
        $dumpvars(1, tb_motor);

        #1000 rst_n = 1;
        #1000;

        // ---- 1) PWM: duty=500 -> 占空比约50% ----
        pwm_duty = 500;
        #100;
        begin : pwm_test
            integer high_cnt;
            high_cnt = 0;
            for (i = 0; i < 2000; i = i + 1) begin
                @(posedge clk);
                if (pwm) high_cnt = high_cnt + 1;
            end
            $display("PWM: duty=500 占空比=%0d%% dir=%b %s", (high_cnt*100)/2000, dir,
                     ((high_cnt >= 900) && (high_cnt <= 1100) && dir) ? "PASS" : "FAIL");
            if (!((high_cnt >= 900) && (high_cnt <= 1100) && dir)) err = err + 1;
        end

        // ---- 2a) 测速: 每10时钟1个脉冲(持续4000拍=8个窗口), 窗口500 -> 应约50 ----
        enc_dir = 1;
        begin : vel_test1
            integer k;
            for (k = 0; k < 4000; k = k + 1) begin
                @(posedge clk);
                pulse = (k % 10 == 0) ? 1 : 0;
            end
            pulse = 0;
            #10;   // 立即读(此时 speed 仍是最后锁存的 50)
            $display("测速(正转): speed=%0d %s", vel,
                     (vel >= 45 && vel <= 55) ? "PASS" : "FAIL");
            if (!(vel >= 45 && vel <= 55)) err = err + 1;
        end
        #1000;   // 让窗口清到 0, 再测反转

        // ---- 2b) 测速: 反转 -> 应约 -50 ----
        enc_dir = 0;
        begin : vel_test2
            integer k;
            for (k = 0; k < 4000; k = k + 1) begin
                @(posedge clk);
                pulse = (k % 10 == 0) ? 1 : 0;
            end
            pulse = 0;
            #10;
            $display("测速(反转): speed=%0d %s", vel,
                     (vel <= -45 && vel >= -55) ? "PASS" : "FAIL");
            if (!(vel <= -45 && vel >= -55)) err = err + 1;
        end

        // ---- 3a) PID: target=500, 应收敛到 500 ----
        enable = 1;
        target = 500;
        #6_000_000;   // 6ms = 3000 个控制周期
        $display("PID: target=500 实际=%0d %s", speed,
                 (speed >= 475 && speed <= 525) ? "PASS" : "FAIL");
        if (!(speed >= 475 && speed <= 525)) err = err + 1;

        // ---- 3b) PID: 重置后 target=-300, 应反向收敛 ----
        enable = 0;          // 清积分
        #1000;
        target = -300;
        enable = 1;
        #8_000_000;
        $display("PID: target=-300 实际=%0d %s", speed,
                 (speed <= -275 && speed >= -325) ? "PASS" : "FAIL");
        if (!(speed <= -275 && speed >= -325)) err = err + 1;

        if (err == 0) $display("\n===== 电机控制测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #30_000_000;
        $display("TIMEOUT"); $finish;
    end
endmodule
