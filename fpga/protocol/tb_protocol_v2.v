`timescale 1ns/1ps
module tb_protocol_v2;
    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;   // 100MHz，仅用于快速功能仿真

    reg        rx_valid = 0;
    reg  [7:0] rx_data = 0;
    wire [7:0] o_cmd, o_seq, o_len;
    wire [255:0] o_payload;
    wire       frame_valid, frame_error;

    protocol_rx dut_rx (
        .clk(clk), .rst_n(rst_n),
        .rx_valid(rx_valid), .rx_data(rx_data),
        .o_cmd(o_cmd), .o_seq(o_seq), .o_len(o_len),
        .o_payload(o_payload), .o_frame_valid(frame_valid),
        .o_frame_error(frame_error)
    );

    reg watchdog_kick = 0;
    wire watchdog_timeout;
    comm_watchdog #(
        .CLK_FREQ(100_000),
        .TIMEOUT_MS(1)
    ) dut_watchdog (
        .clk(clk), .rst_n(rst_n),
        .kick(watchdog_kick),
        .timeout(watchdog_timeout)
    );

    reg signed [15:0] v_in, w_in;
    wire signed [15:0] wheel_left, wheel_right;
    twist_to_wheel #(
        .V_MAX_MM_S(3000),
        .TRACK_WIDTH_MM(160),
        .TARGET_MAX(1000)
    ) dut_twist (
        .v_mm_s(v_in), .w_mrad_s(w_in),
        .target_left(wheel_left), .target_right(wheel_right)
    );

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

    task send_byte(input [7:0] b);
        begin
            @(posedge clk);
            rx_valid = 1'b1;
            rx_data  = b;
            @(posedge clk);
            rx_valid = 1'b0;
            rx_data  = 8'd0;
        end
    endtask

    task send_frame(
        input [7:0] cmd,
        input [7:0] seq,
        input [7:0] len,
        input [255:0] payload,
        input       corrupt_crc
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
            if (corrupt_crc)
                crc = crc ^ 16'h0001;
            send_byte(crc[7:0]);
            send_byte(crc[15:8]);
        end
    endtask

    integer err = 0;
    initial begin
        $dumpfile("protocol_v2_wave.vcd");
        $dumpvars(1, tb_protocol_v2);

        #100 rst_n = 1;
        #100;

        // SET_TWIST v=300, w=-250
        send_frame(8'h01, 8'h42, 8'd4, {240'd0, 16'hFF06, 16'h012C}, 0);
        #100;
        $display("[协议] cmd=0x%02X seq=0x%02X len=%0d payload0=0x%04X payload1=0x%04X %s",
                 o_cmd, o_seq, o_len, o_payload[15:0], o_payload[31:16],
                 (o_cmd == 8'h01 && o_seq == 8'h42 && o_len == 4 &&
                  o_payload[15:0] == 16'h012C && o_payload[31:16] == 16'hFF06)
                 ? "PASS" : "FAIL");
        if (!(o_cmd == 8'h01 && o_seq == 8'h42 && o_len == 4 &&
              o_payload[15:0] == 16'h012C && o_payload[31:16] == 16'hFF06))
            err = err + 1;

        // 零长度 HEARTBEAT
        send_frame(8'h10, 8'h43, 8'd0, 256'd0, 0);
        #100;
        $display("[协议] HEARTBEAT cmd=0x%02X len=%0d %s", o_cmd, o_len,
                 (o_cmd == 8'h10 && o_len == 0) ? "PASS" : "FAIL");
        if (!(o_cmd == 8'h10 && o_len == 0)) err = err + 1;

        // 坏 CRC，应产生 frame_error 且不产生 frame_valid
        send_frame(8'h01, 8'h44, 8'd4, 256'd0, 1);
        wait (frame_error == 1'b1);
        $display("[协议] 坏 CRC frame_error=%0d %s", frame_error,
                 (frame_error == 1'b1) ? "PASS" : "FAIL");
        if (frame_error != 1'b1) err = err + 1;

        // 看门狗 kick 后不超时，等待后超时
        watchdog_kick = 1'b1;
        @(posedge clk);
        watchdog_kick = 1'b0;
        #200;
        if (watchdog_timeout) begin
            $display("[看门狗] kick 后提前超时 FAIL");
            err = err + 1;
        end
        #1200;
        $display("[看门狗] timeout=%0d %s", watchdog_timeout,
                 (watchdog_timeout == 1'b1) ? "PASS" : "FAIL");
        if (watchdog_timeout != 1'b1) err = err + 1;

        // v=300,w=0 -> 双轮 +100
        v_in = 16'sd300; w_in = 16'sd0;
        #10;
        $display("[Twist] v=300,w=0 -> L=%0d R=%0d %s",
                 wheel_left, wheel_right,
                 (wheel_left == 16'sd100 && wheel_right == 16'sd100)
                 ? "PASS" : "FAIL");
        if (!(wheel_left == 16'sd100 && wheel_right == 16'sd100)) err = err + 1;

        // v=300,w=500 -> L=86, R=113（整数除法截断）
        v_in = 16'sd300; w_in = 16'sd500;
        #10;
        $display("[Twist] v=300,w=500 -> L=%0d R=%0d %s",
                 wheel_left, wheel_right,
                 (wheel_left == 16'sd86 && wheel_right == 16'sd113)
                 ? "PASS" : "FAIL");
        if (!(wheel_left == 16'sd86 && wheel_right == 16'sd113)) err = err + 1;

        // v=-300,w=0 -> 双轮 -100（验证有符号转换）
        v_in = -16'sd300; w_in = 16'sd0;
        #10;
        $display("[Twist] v=-300,w=0 -> L=%0d R=%0d %s",
                 wheel_left, wheel_right,
                 (wheel_left == -16'sd100 && wheel_right == -16'sd100)
                 ? "PASS" : "FAIL");
        if (!(wheel_left == -16'sd100 && wheel_right == -16'sd100))
            err = err + 1;

        if (err == 0) $display("\n===== 协议 v2 基础测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #1_000_000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
