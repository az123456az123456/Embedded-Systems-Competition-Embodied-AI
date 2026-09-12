// ============================================================
// tb_uart.v — UART 回环测试
// 用真实参数: 时钟 50MHz, 波特率 115200 (板卡同款)
// 发送 4 个字节 0x55 0xAA 0x0F 0xF0, 通过回环验证接收
// 注意: 时钟频率必须满足 16 倍过采样 (CLK/BAUD >= 16)
// ============================================================
`timescale 1ns/1ps
module tb_uart;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         tx_start = 0;
    reg  [7:0]  tx_data = 0;
    wire        tx;
    wire        tx_busy;
    wire        rx_valid;
    wire [7:0]  rx_data;

    // 待发送的测试数据
    reg [7:0] send_bytes [0:3];
    integer   i;

    // 回环: TX 直接接到 RX
    uart_tx #(.CLK_FREQ(50_000_000), .BAUD(115200)) u_tx (
        .clk(clk), .rst_n(rst_n), .tx_start(tx_start), .tx_data(tx_data),
        .tx(tx), .tx_busy(tx_busy)
    );
    uart_rx #(.CLK_FREQ(50_000_000), .BAUD(115200)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(tx),
        .rx_valid(rx_valid), .rx_data(rx_data)
    );

    always #10 clk = ~clk;   // 50MHz

    initial begin
        $dumpfile("uart_wave.vcd");
        $dumpvars(0, tb_uart);

        send_bytes[0] = 8'h55;   // 0101_0101
        send_bytes[1] = 8'hAA;   // 1010_1010
        send_bytes[2] = 8'h0F;   // 0000_1111
        send_bytes[3] = 8'hF0;   // 1111_0000

        #1000 rst_n = 1;
        #1000;

        // 依次发送 4 个字节, 每次等发送完成
        for (i = 0; i < 4; i = i + 1) begin
            tx_data  = send_bytes[i];
            @(posedge clk);
            tx_start = 1;
            @(posedge clk);
            tx_start = 0;
            // 等待发送完成 + 接收完成
            wait (tx_busy == 0);
            wait (rx_valid == 1);
            @(posedge clk);
            // 校验
            if (rx_data == send_bytes[i])
                $display("PASS: 发送 0x%02X -> 收到 0x%02X", send_bytes[i], rx_data);
            else begin
                $display("FAIL: 发送 0x%02X -> 收到 0x%02X", send_bytes[i], rx_data);
                $finish;
            end
            // 等接收忙结束
            #100;
        end

        $display("\n===== UART 回环测试全部通过! =====");
        $finish;
    end

    // 全局超时(10ms), 防止死循环挂死
    initial begin
        #10_000_000;
        $display("TIMEOUT: 仿真超时, 测试失败");
        $finish;
    end
endmodule
