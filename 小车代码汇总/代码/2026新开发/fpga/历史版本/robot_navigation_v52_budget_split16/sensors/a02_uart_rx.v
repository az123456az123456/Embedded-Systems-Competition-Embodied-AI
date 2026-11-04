`timescale 1ns/1ps
// ============================================================================
//  UART 接收：系统时钟计数，在各位中点采样并检查停止位
// ============================================================================
module a02_uart_rx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 9600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);
    localparam integer DIV = CLK_FREQ / BAUD;   // 50MHz/9600 = 5208

    reg [1:0]  rx_sync;
    reg [31:0] cnt;
    reg [2:0]  bit_idx;
    reg [1:0] bit_samples;
    wire sampled_bit = (bit_samples[1] & bit_samples[0]) |
        (bit_samples[1] & rx_sync[1]) | (bit_samples[0] & rx_sync[1]);
    reg [7:0]  shift;
    reg [1:0]  st;                              // 0空闲 1起始 2数据 3停止

    always @(posedge clk)
        if (!rst_n) rx_sync <= 2'b11;
        else rx_sync <= {rx_sync[0], rx};            // 两级同步，防亚稳态

    reg rx_prev;
    always @(posedge clk)
        if (!rst_n) rx_prev <= 1'b1;
        else rx_prev <= rx_sync[1];
    wire rx_fall = rx_prev & ~rx_sync[1];

    always @(posedge clk) begin
        if (!rst_n) begin
            st <= 2'd0; cnt <= 32'd0; bit_idx <= 3'd0;
            shift <= 8'd0; data <= 8'd0; valid <= 1'b0; bit_samples <= 2'b11;
        end else begin
            valid <= 1'b0;
            // Three samples ending at the original midpoint reject a one-clock
            // glitch without delaying bytes or changing the UART baud rate.
            if(st!=0 && cnt==2)bit_samples[1]<=rx_sync[1];
            if(st!=0 && cnt==1)bit_samples[0]<=rx_sync[1];
            case (st)
                2'd0: if (rx_fall) begin
                          st  <= 2'd1;
                          cnt <= DIV/2 - 1;             // 半位后采起始位中点
                      end
                2'd1: if (cnt == 32'd0) begin
                          if (!sampled_bit) begin    // 确认不是毛刺
                              st      <= 2'd2;
                              cnt     <= DIV - 1;
                              bit_idx <= 3'd0;
                          end else
                              st <= 2'd0;
                      end else
                          cnt <= cnt - 1'b1;
                2'd2: if (cnt == 32'd0) begin
                          shift <= {sampled_bit, shift[7:1]};   // LSB 先到
                          cnt   <= DIV - 1;
                          if (bit_idx == 3'd7)
                              st <= 2'd3;
                          else
                              bit_idx <= bit_idx + 1'b1;
                      end else
                          cnt <= cnt - 1'b1;
                2'd3: if (cnt == 32'd0) begin
                          if (sampled_bit) begin
                              data <= shift;
                              valid <= 1'b1;
                          end
                          st    <= 2'd0;
                      end else
                          cnt <= cnt - 1'b1;
                default: st <= 2'd0;
            endcase
        end
    end
endmodule


