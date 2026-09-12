// ============================================================
// tb_top.v — 顶层集成测试
//  1) 发 SET_SPEED  -> 目标速度生效 + 电机输出
//  2) 发 SET_MODE 1 + 注入 10cm 回波 -> 避障自动停车
//  3) 发 SET_MODE 0 -> 恢复目标速度
//  4) 发 STOP -> 速度归零
// ============================================================
`timescale 1ns/1ps
module tb_top;

    reg clk = 0;
    reg rst_n = 0;
    always #10 clk = ~clk;   // 50MHz

    reg  uart_rx_pin = 1;
    wire uart_tx_pin;
    reg  enc_a = 0, enc_b = 0;
    wire motor_pwm_w, motor_dir_w;
    reg  echo = 0;
    wire trig_w;
    wire imu_sda_w;
    wire imu_scl_w;
    pullup(imu_sda_w);

    wire signed [15:0] o_target, o_speed;
    wire [15:0] o_distance;
    wire [3:0]  o_mode;

    top_robot #(
        .CLK_FREQ(50_000_000),
        .BAUD(115200),
        .TRIG_PERIOD(60_000),
        .REPORT_PERIOD(50_000),
        .OBSTACLE_CM(16'd20),
        .SLOW_CM(16'd40)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .uart_rx(uart_rx_pin), .uart_tx(uart_tx_pin),
        .enc_a(enc_a), .enc_b(enc_b),
        .motor_pwm(motor_pwm_w), .motor_dir(motor_dir_w),
        .echo(echo), .trig(trig_w),
        .imu_sda(imu_sda_w), .imu_scl(imu_scl_w),
        .o_target(o_target), .o_speed(o_speed),
        .o_distance(o_distance), .o_mode(o_mode)
    );

    // ============ UART 发送(模拟上位机) ============
    localparam BIT_NS = 8680;
    task send_byte(input [7:0] b);
        integer i;
        begin
            uart_rx_pin = 1'b0; #(BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_pin = b[i]; #(BIT_NS);
            end
            uart_rx_pin = 1'b1; #(BIT_NS);
        end
    endtask

    task send_frame(input [7:0] cmd, input [7:0] dh, input [7:0] dl);
        begin
            send_byte(8'hAA);
            send_byte(cmd);
            send_byte(dh);
            send_byte(dl);
            send_byte(8'hAA ^ cmd ^ dh ^ dl);
        end
    endtask

    // ============ 超声波回波模拟 ============
    reg inject_obstacle = 0;
    always @(posedge trig_w) begin
        @(negedge trig_w);
        if (inject_obstacle) begin
            #2000;
            echo = 1;
            #583110;        // ~10cm (29155 时钟)
            echo = 0;
        end
    end

    // ============ 测试 ============
    integer err = 0;
    integer hi, i;
    integer pwm_hi;

    task count_pwm_time(output integer hi_cnt);
        integer k;
        begin
            hi_cnt = 0;
            for (k = 0; k < 2000; k = k + 1) begin
                @(posedge clk);
                if (motor_pwm_w) hi_cnt = hi_cnt + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("top_wave.vcd");
        $dumpvars(1, tb_top);

        #1000 rst_n = 1;
        #100000;

        // ---- 1) SET_SPEED 500 ----
        send_frame(8'h01, 8'h01, 8'hF4);
        #200000;
        $display("[1] SET_SPEED=500 -> o_target=%0d %s", o_target,
                 (o_target == 16'sd500) ? "PASS" : "FAIL");
        if (o_target != 16'sd500) err = err + 1;
        count_pwm_time(pwm_hi);
        $display("    电机PWM高电平 %0d/2000 (应大量输出) %s", pwm_hi,
                 (pwm_hi > 1000) ? "PASS" : "FAIL");
        if (!(pwm_hi > 1000)) err = err + 1;

        // ---- 2) SET_MODE 1 + 注入 10cm 障碍 ----
        send_frame(8'h03, 8'h00, 8'h01);
        inject_obstacle = 1;
        #3_000_000;       // 等超声波测几次
        $display("[2] 避障模式: o_mode=%0d o_distance=%0d cm", o_mode, o_distance);
        $display("    障碍<20cm -> o_target=%0d (应停车=0) %s", o_target,
                 (o_target == 16'sd0) ? "PASS" : "FAIL");
        if (o_target != 16'sd0) err = err + 1;
        count_pwm_time(pwm_hi);
        $display("    电机PWM高电平 %0d/2000 (应停) %s", pwm_hi,
                 (pwm_hi < 100) ? "PASS" : "FAIL");
        if (!(pwm_hi < 100)) err = err + 1;

        // ---- 3) SET_MODE 0 -> 恢复 ----
        send_frame(8'h03, 8'h00, 8'h00);
        inject_obstacle = 0;
        #2_000_000;
        $display("[3] 退出避障 -> o_mode=%0d o_target=%0d (应恢复500) %s",
                 o_mode, o_target, (o_target == 16'sd500) ? "PASS" : "FAIL");
        if (o_target != 16'sd500) err = err + 1;

        // ---- 4) STOP ----
        send_frame(8'h02, 8'h00, 8'h00);
        #200000;
        $display("[4] STOP -> o_target=%0d %s", o_target,
                 (o_target == 16'sd0) ? "PASS" : "FAIL");
        if (o_target != 16'sd0) err = err + 1;

        if (err == 0) $display("\n===== 顶层集成测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #30_000_000;
        $display("TIMEOUT"); $finish;
    end
endmodule
