`timescale 1ns/1ps
module tb_top_v2;
    reg clk = 0;
    reg rst_n = 0;
    always #10 clk = ~clk;   // 50MHz

    reg  uart_rx_pin = 1;
    wire uart_tx_pin;
    reg  enc_left_a = 0, enc_left_b = 0;
    reg  enc_right_a = 0, enc_right_b = 0;
    wire motor_left_pwm, motor_left_dir;
    wire motor_right_pwm, motor_right_dir;
    reg  echo = 0;
    wire trig;
    wire imu_sda;
    wire imu_scl;
    pullup(imu_sda);

    wire signed [15:0] o_target_left, o_target_right;
    wire signed [15:0] o_speed_left, o_speed_right;
    wire [15:0] o_distance_cm;
    wire [3:0]  o_mode;
    wire [15:0] o_flags;
    wire        o_watchdog;

    top_robot_v2 #(
        .CLK_FREQ(50_000_000),
        .BAUD(115200),
        .TRIG_PERIOD(60_000),
        .REPORT_PERIOD(250_000),   // 5ms，方便仿真
        .WATCHDOG_MS(10),
        .OBSTACLE_CM(16'd20),
        .SLOW_CM(16'd40),
        .V_MAX_MM_S(3000),
        .TRACK_WIDTH_MM(160),
        .TARGET_MAX(1000),
        .PID_DIV(100)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .uart_rx(uart_rx_pin), .uart_tx(uart_tx_pin),
        .enc_left_a(enc_left_a), .enc_left_b(enc_left_b),
        .enc_right_a(enc_right_a), .enc_right_b(enc_right_b),
        .motor_left_pwm(motor_left_pwm), .motor_left_dir(motor_left_dir),
        .motor_right_pwm(motor_right_pwm), .motor_right_dir(motor_right_dir),
        .echo(echo), .trig(trig), .imu_sda(imu_sda), .imu_scl(imu_scl),
        .o_target_left(o_target_left), .o_target_right(o_target_right),
        .o_speed_left(o_speed_left), .o_speed_right(o_speed_right),
        .o_distance_cm(o_distance_cm), .o_mode(o_mode),
        .o_flags(o_flags), .o_watchdog(o_watchdog)
    );

    // UART monitor: capture top's ACK/STATUS bytes
    wire mon_valid;
    wire [7:0] mon_data;
    uart_rx #(.CLK_FREQ(50_000_000), .BAUD(115200)) u_monitor (
        .clk(clk), .rst_n(rst_n), .rx(uart_tx_pin),
        .rx_valid(mon_valid), .rx_data(mon_data)
    );

    wire [7:0] mon_cmd, mon_seq, mon_len;
    wire [255:0] mon_payload;
    wire mon_frame_valid, mon_frame_error;
    protocol_rx u_proto_monitor (
        .clk(clk), .rst_n(rst_n),
        .rx_valid(mon_valid), .rx_data(mon_data),
        .o_cmd(mon_cmd), .o_seq(mon_seq), .o_len(mon_len),
        .o_payload(mon_payload), .o_frame_valid(mon_frame_valid),
        .o_frame_error(mon_frame_error)
    );
    reg mon_ack_seen = 0;
    reg mon_status_seen = 0;
    integer mon_crc_errors = 0;
    always @(posedge clk) begin
        if (rst_n && mon_frame_valid) begin
            if (mon_cmd == 8'h82) mon_ack_seen <= 1'b1;
            if (mon_cmd == 8'h81) mon_status_seen <= 1'b1;
        end
        if (rst_n && mon_frame_error)
            mon_crc_errors <= mon_crc_errors + 1;
    end

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

    function [15:0] crc_next;
        input [15:0] crc_in;
        input [7:0] data;
        integer i;
        reg [15:0] c;
        begin
            c = crc_in ^ {data, 8'h00};
            for (i = 0; i < 8; i = i + 1) begin
                if (c[15]) c = {c[14:0], 1'b0} ^ 16'h1021;
                else       c = {c[14:0], 1'b0};
            end
            crc_next = c;
        end
    endfunction

    task send_frame(
        input [7:0] cmd,
        input [7:0] seq,
        input [7:0] len,
        input [255:0] payload
    );
        integer i;
        reg [15:0] crc;
        reg [7:0] b;
        begin
            crc = 16'hFFFF;
            send_byte(8'hAA);
            send_byte(8'h55);
            send_byte(cmd);
            crc = crc_next(crc, cmd);
            send_byte(seq);
            crc = crc_next(crc, seq);
            send_byte(len);
            crc = crc_next(crc, len);
            for (i = 0; i < len; i = i + 1) begin
                b = payload[i*8 +: 8];
                send_byte(b);
                crc = crc_next(crc, b);
            end
            send_byte(crc[7:0]);
            send_byte(crc[15:8]);
        end
    endtask

    task send_twist(input signed [15:0] v, input signed [15:0] w);
        reg [255:0] payload;
        begin
            payload = 256'd0;
            payload[15:0]  = v;
            payload[31:16] = w;
            send_frame(8'h01, 8'h01, 8'd4, payload);
        end
    endtask

    integer hi, k;
    task count_left_pwm(output integer hi_cnt);
        begin
            hi_cnt = 0;
            for (k = 0; k < 2000; k = k + 1) begin
                @(posedge clk);
                if (motor_left_pwm) hi_cnt = hi_cnt + 1;
            end
        end
    endtask

    integer err = 0;

    // ============ 超声波回波模拟 ============
    reg inject_obstacle = 0;
    always @(posedge trig) begin
        @(negedge trig);
        if (inject_obstacle) begin
            #2000;
            echo = 1;
            #583110;      // 约 10cm
            echo = 0;
        end
    end

    initial begin
        $dumpfile("top_v2_wave.vcd");
        $dumpvars(1, tb_top_v2);

        #1000 rst_n = 1;
        #100000;

        // SET_MODE=2 (VISION)
        send_frame(8'h03, 8'h10, 8'd1, {255'd0, 8'd2});
        // SET_TWIST v=300,w=0 -> 双轮目标 100
        send_twist(16'sd300, 16'sd0);
        #200000;

        $display("[1] SET_MODE=%0d SET_TWIST=(300,0) -> L=%0d R=%0d %s",
                 o_mode, o_target_left, o_target_right,
                 (o_mode == 4'd2 && o_target_left == 16'sd100 &&
                  o_target_right == 16'sd100) ? "PASS" : "FAIL");
        if (!(o_mode == 4'd2 && o_target_left == 16'sd100 &&
              o_target_right == 16'sd100)) err = err + 1;

        count_left_pwm(hi);
        $display("    左电机PWM高电平 %0d/2000 %s", hi,
                 (hi > 1000) ? "PASS" : "FAIL");
        if (!(hi > 1000)) err = err + 1;

        // SET_TWIST v=300,w=500 -> L=86,R=113
        send_twist(16'sd300, 16'sd500);
        #100000;
        $display("[2] SET_TWIST=(300,500) -> L=%0d R=%0d %s",
                 o_target_left, o_target_right,
                 (o_target_left == 16'sd86 && o_target_right == 16'sd113)
                 ? "PASS" : "FAIL");
        if (!(o_target_left == 16'sd86 && o_target_right == 16'sd113))
            err = err + 1;

        // 等一次 STATUS（5ms）
        #5_100_000;
        $display("[3] UART 返回 ACK=%0d STATUS=%0d CRC错误=%0d %s",
                 mon_ack_seen, mon_status_seen, mon_crc_errors,
                 (mon_ack_seen && mon_status_seen && mon_crc_errors == 0)
                 ? "PASS" : "FAIL");
        if (!(mon_ack_seen && mon_status_seen && mon_crc_errors == 0))
            err = err + 1;

        // 恢复运动，然后停止心跳，验证 10ms 看门狗停车
        send_twist(16'sd300, 16'sd0);
        #200000;
        if (o_target_left != 16'sd100) begin
            $display("[4] 看门狗前置条件失败：target=%0d", o_target_left);
            err = err + 1;
        end
        #11_000_000;
        $display("[4] 无心跳 -> watchdog=%0d L=%0d R=%0d flags=0x%04X %s",
                 o_watchdog, o_target_left, o_target_right, o_flags,
                 (o_watchdog && o_target_left == 0 && o_target_right == 0)
                 ? "PASS" : "FAIL");
        if (!(o_watchdog && o_target_left == 0 && o_target_right == 0))
            err = err + 1;

        // 清除看门狗，进入避障模式，注入 10cm 障碍
        send_frame(8'h04, 8'h20, 8'd0, 256'd0);
        send_frame(8'h03, 8'h21, 8'd1, {255'd0, 8'd1});
        send_twist(16'sd300, 16'sd0);
        inject_obstacle = 1;
        #3_000_000;
        count_left_pwm(hi);
        $display("[5] 避障 10cm -> dist=%0d L=%0d R=%0d flags=0x%04X PWM=%0d %s",
                 o_distance_cm, o_target_left, o_target_right, o_flags, hi,
                 (o_distance_cm <= 12 && o_target_left == 0 &&
                  o_target_right == 0 && hi < 100) ? "PASS" : "FAIL");
        if (!(o_distance_cm <= 12 && o_target_left == 0 &&
              o_target_right == 0 && hi < 100))
            err = err + 1;

        if (err == 0) $display("\n===== top_robot_v2 集成测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #40_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
