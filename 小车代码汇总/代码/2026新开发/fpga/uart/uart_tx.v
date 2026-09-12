// ============================================================
// uart_tx.v — UART 发送模块 (8N1)
// 参数: CLK_FREQ 系统时钟频率, BAUD 波特率
// 用法: 拉高 tx_start 一拍, tx_data 会被按 起始位0+8数据位LSB先+停止位1 发出
// 信号: tx_busy 高表示正在发送, 发送完成后自动拉低
// ============================================================
module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,      // 发送使能(一拍脉冲)
    input  wire [7:0] tx_data,       // 要发送的字节
    output reg        tx,            // 串行输出
    output reg        tx_busy        // 忙标志
);

    localparam BAUD_DIV = CLK_FREQ / BAUD;   // 每个比特占用的时钟数

    reg [15:0] baud_cnt;                     // 波特率分频计数
    reg        baud_tick;                    // 每比特一次的脉冲
    reg [3:0]  bit_cnt;                      // 0=起始 1~8=数据 9=停止
    reg [7:0]  shift;                        // 移位寄存器

    // ---- 波特率分频 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt  <= 16'd0;
            baud_tick <= 1'b0;
        end else if (tx_busy) begin
            if (baud_cnt == BAUD_DIV - 1) begin
                baud_cnt  <= 16'd0;
                baud_tick <= 1'b1;
            end else begin
                baud_cnt  <= baud_cnt + 16'd1;
                baud_tick <= 1'b0;
            end
        end else begin
            baud_cnt  <= 16'd0;
            baud_tick <= 1'b0;
        end
    end

    // ---- 发送状态机 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx      <= 1'b1;      // 空闲为高
            tx_busy <= 1'b0;
            bit_cnt <= 4'd0;
            shift   <= 8'd0;
        end else if (tx_start && !tx_busy) begin
            tx_busy <= 1'b1;
            tx      <= 1'b0;      // 起始位
            shift   <= tx_data;
            bit_cnt <= 4'd1;      // 下一个比特是数据位0
        end else if (tx_busy && baud_tick) begin
            if (bit_cnt <= 4'd8) begin
                // 发送数据位 (LSB 先)
                tx      <= shift[0];
                shift   <= {1'b0, shift[7:1]};
                bit_cnt <= bit_cnt + 4'd1;
            end else begin
                // 停止位
                tx      <= 1'b1;
                bit_cnt <= 4'd0;
                tx_busy <= 1'b0;
            end
        end
    end
endmodule
