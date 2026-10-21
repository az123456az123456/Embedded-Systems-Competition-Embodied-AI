// ============================================================
// uart_rx.v — UART 接收模块 (8N1, 16倍过采样抗干扰)
// 参数: CLK_FREQ 系统时钟频率, BAUD 波特率
// 输出: rx_valid 一拍脉冲(收到一个字节), rx_data 数据
// ============================================================
module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,             // 串行输入
    output reg        rx_valid,       // 收到一字节(一拍脉冲)
    output reg  [7:0] rx_data         // 接收数据
);

    localparam OV_DIV = CLK_FREQ / BAUD / 16;   // 每个过采样点时钟数

    reg [15:0] ov_cnt;                // 过采样计数
    reg        ov_tick;               // 过采样脉冲
    reg [3:0]  s_cnt;                 // 0~15 过采样相位(4位,自动回绕)
    reg [3:0]  bit_cnt;               // 0=起始 1~8=数据 9=停止
    reg [7:0]  shift;
    reg        rx_busy;
    reg        rx_r, rx_rr;           // 同步打拍, 消抖

    // ---- 输入同步 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_r  <= 1'b1;
            rx_rr <= 1'b1;
        end else begin
            rx_r  <= rx;
            rx_rr <= rx_r;
        end
    end

    // ---- 过采样分频 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ov_cnt  <= 16'd0;
            ov_tick <= 1'b0;
        end else if (rx_busy) begin
            if (ov_cnt == OV_DIV - 1) begin
                ov_cnt  <= 16'd0;
                ov_tick <= 1'b1;
            end else begin
                ov_cnt  <= ov_cnt + 16'd1;
                ov_tick <= 1'b0;
            end
        end else begin
            ov_cnt  <= 16'd0;
            ov_tick <= 1'b0;
        end
    end

    // ---- 接收状态机 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_valid <= 1'b0;
            rx_data  <= 8'd0;
            rx_busy  <= 1'b0;
            s_cnt    <= 5'd0;
            bit_cnt  <= 4'd0;
            shift    <= 8'd0;
        end else begin
            rx_valid <= 1'b0;
            if (!rx_busy) begin
                // 检测起始位(下降沿)
                if (rx_rr == 1'b0) begin
                    rx_busy <= 1'b1;
                    s_cnt   <= 4'd0;
                    bit_cnt <= 4'd0;
                end
            end else if (ov_tick) begin
                if (s_cnt == 4'd7) begin
                    // 过采样中点采样
                    s_cnt <= s_cnt + 4'd1;
                    case (bit_cnt)
                        4'd0: begin
                            // 起始位应仍为低, 否则视为毛刺, 重新等待
                            if (rx_rr) rx_busy <= 1'b0;
                        end
                        4'd1, 4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8: begin
                            shift <= {rx_rr, shift[7:1]};
                        end
                        4'd9: begin
                            // 停止位后完成
                            rx_data  <= shift;
                            rx_valid <= 1'b1;
                            rx_busy  <= 1'b0;
                        end
                    endcase
                    bit_cnt <= bit_cnt + 4'd1;
                end else begin
                    s_cnt <= s_cnt + 4'd1;
                end
            end
        end
    end
endmodule
