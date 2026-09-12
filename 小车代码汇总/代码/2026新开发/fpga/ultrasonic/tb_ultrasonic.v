// ============================================================
// tb_ultrasonic.v — 超声波测距测试
// 距离(cm) = echo高电平时钟数 * 0.000343
// 1) echo 100000拍 -> 34cm   2) echo 200000拍 -> 68cm
// 3) 无回波 -> 超时 0xFFFF
// ============================================================
`timescale 1ns/1ps
module tb_ultrasonic;

    reg clk = 0;
    reg rst_n = 0;
    reg trigger = 0;
    reg echo = 0;
    wire trig;
    wire [15:0] distance_cm;
    wire done;
    wire busy;

    ultrasonic #(.CLK_FREQ(50_000_000), .TRIG_US(10), .TIMEOUT_US(10_000)) uut (
        .clk(clk), .rst_n(rst_n), .trigger(trigger), .echo(echo),
        .trig(trig), .distance_cm(distance_cm), .done(done), .busy(busy)
    );

    always #10 clk = ~clk;   // 50MHz

    integer err = 0;
    integer k;

    // 测一次: delay 拍后给 echo 高电平 hold 拍
    task meas(input integer delay_clks, input integer echo_hold, input integer expect);
        begin
            // 等上一次结束
            while (busy) @(posedge clk);
            #100;
            @(posedge clk); trigger = 1;
            @(posedge clk); trigger = 0;
            // 等触发脉冲结束
            while (trig) @(posedge clk);
            // 模拟回波延迟
            for (k = 0; k < delay_clks; k = k + 1) @(posedge clk);
            echo = 1;
            for (k = 0; k < echo_hold; k = k + 1) @(posedge clk);
            echo = 0;
            // 等完成
            while (!done) @(posedge clk);
            @(posedge clk);
            $display("echo=%0d拍 -> distance=%0d cm (期望~%0d) %s",
                     echo_hold, distance_cm, expect,
                     ((distance_cm >= expect-2) && (distance_cm <= expect+2)) ? "PASS" : "FAIL");
            if (!((distance_cm >= expect-2) && (distance_cm <= expect+2))) err = err + 1;
        end
    endtask

    initial begin
        $dumpfile("ultrasonic_wave.vcd");
        $dumpvars(1, tb_ultrasonic);
        #1000 rst_n = 1;
        #1000;

        // 1) 100000拍 -> 34.3cm
        meas(5_000, 100_000, 34);
        // 2) 200000拍 -> 68.6cm
        meas(5_000, 200_000, 68);
        // 3) 无回波 -> 超时 0xFFFF
        begin
            while (busy) @(posedge clk);
            #100;
            @(posedge clk); trigger = 1;
            @(posedge clk); trigger = 0;
            while (trig) @(posedge clk);
            // 不拉 echo, 等超时
            while (!done) @(posedge clk);
            @(posedge clk);
            $display("无回波 -> distance=%h %s", distance_cm,
                     (distance_cm == 16'hFFFF) ? "PASS" : "FAIL");
            if (distance_cm != 16'hFFFF) err = err + 1;
        end

        if (err == 0) $display("\n===== 超声波测距测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #50_000_000;
        $display("TIMEOUT"); $finish;
    end
endmodule
