`timescale 1ns/1ps
// ============================================================================
//  UART 发送
// ============================================================================
module a02_uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       send,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);
    localparam integer DIV = CLK_FREQ / BAUD;

    reg [31:0] cnt;
    reg [3:0]  bit_idx;
    reg [9:0]  shift;

    always @(posedge clk) begin
        if (!rst_n) begin
            tx      <= 1'b1;
            busy    <= 1'b0;
            cnt     <= 32'd0;
            bit_idx <= 4'd0;
            shift   <= 10'h3FF;
        end else if (!busy) begin
            tx <= 1'b1;
            if (send) begin
                tx      <= 1'b0;
                shift   <= {1'b1, 1'b1, data};  // 停止位 + 数据 + 起始位
                busy    <= 1'b1;
                bit_idx <= 4'd0;
                cnt     <= DIV - 1;
            end
        end else begin
            if (cnt == 32'd0) begin
                cnt    <= DIV - 1;
                tx     <= shift[0];
                shift  <= {1'b1, shift[9:1]};
                if (bit_idx == 4'd9) begin
                    busy <= 1'b0;
                    tx   <= 1'b1;
                end else
                    bit_idx <= bit_idx + 1'b1;
            end else
                cnt <= cnt - 1'b1;
        end
    end
endmodule
