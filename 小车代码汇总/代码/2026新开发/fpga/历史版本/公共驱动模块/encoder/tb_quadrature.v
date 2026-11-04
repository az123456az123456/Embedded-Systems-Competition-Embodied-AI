// ============================================================
// tb_quadrature.v — 编码器解码测试 (纯 Verilog-2001)
// 正转一圈(4跳变)->+4; 再正转->+8; 反转一圈->+4; 再反转->0
// ============================================================
`timescale 1ns/1ps
module tb_quadrature;

    reg clk = 0;
    reg rst_n = 0;
    reg enc_a = 0;
    reg enc_b = 0;
    wire signed [15:0] position;
    wire pulse_tick;
    wire direction;

    quadrature_decoder #(.WIDTH(16)) uut (
        .clk(clk), .rst_n(rst_n), .enc_a(enc_a), .enc_b(enc_b),
        .position(position), .pulse_tick(pulse_tick), .direction(direction)
    );

    always #10 clk = ~clk;   // 50MHz

    integer err = 0;
    integer i, j;

    // 走完整圈: 5 个状态(00->01->11->10->00) = 4 次跳变 = 1 圈
    task walk(input integer cycles, input integer forward, input integer hold);
        integer c, s;
        begin
            for (c = 0; c < cycles; c = c + 1) begin
                for (s = 0; s < 5; s = s + 1) begin
                    if (forward) begin
                        case (s)
                            0: {enc_a, enc_b} = 2'b00;
                            1: {enc_a, enc_b} = 2'b01;
                            2: {enc_a, enc_b} = 2'b11;
                            3: {enc_a, enc_b} = 2'b10;
                            4: {enc_a, enc_b} = 2'b00;
                        endcase
                    end else begin
                        case (s)
                            0: {enc_a, enc_b} = 2'b00;
                            1: {enc_a, enc_b} = 2'b10;
                            2: {enc_a, enc_b} = 2'b11;
                            3: {enc_a, enc_b} = 2'b01;
                            4: {enc_a, enc_b} = 2'b00;
                        endcase
                    end
                    for (j = 0; j < hold; j = j + 1) @(posedge clk);
                end
            end
        end
    endtask

    initial begin
        #1000 rst_n = 1;
        #1000;

        // 1) 正转一圈
        walk(1, 1, 10);
        #100;
        $display("正转一圈: position=%0d dir=%b %s", position, direction,
                 (position==16'sd4 && direction==1'b1) ? "PASS" : "FAIL");
        if (position != 16'sd4 || direction != 1'b1) err = err + 1;

        // 2) 再正转一圈
        walk(1, 1, 10);
        #100;
        $display("再正转一圈: position=%0d %s", position,
                 (position==16'sd8) ? "PASS" : "FAIL");
        if (position != 16'sd8) err = err + 1;

        // 3) 反转一圈
        walk(1, 0, 10);
        #100;
        $display("反转一圈: position=%0d dir=%b %s", position, direction,
                 (position==16'sd4 && direction==1'b0) ? "PASS" : "FAIL");
        if (position != 16'sd4 || direction != 1'b0) err = err + 1;

        // 4) 再反转一圈
        walk(1, 0, 10);
        #100;
        $display("再反转一圈: position=%0d %s", position,
                 (position==16'sd0) ? "PASS" : "FAIL");
        if (position != 16'sd0) err = err + 1;

        if (err == 0) $display("\n===== 编码器解码测试全部通过! =====");
        else          $display("\n===== 有 %0d 项失败 =====", err);
        $finish;
    end

    initial begin
        #10_000_000;
        $display("TIMEOUT"); $finish;
    end
endmodule
