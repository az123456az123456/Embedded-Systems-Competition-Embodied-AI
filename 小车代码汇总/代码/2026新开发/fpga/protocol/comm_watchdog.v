// ============================================================
// comm_watchdog.v — 独立通信看门狗
// 任意有效下行帧或 HEARTBEAT 都会 kick；超时输出锁存到下一次 kick。
// ============================================================
module comm_watchdog #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer TIMEOUT_MS = 500
)(
    input  wire clk,
    input  wire rst_n,
    input  wire kick,
    output reg  timeout
);
    localparam integer LIMIT = (CLK_FREQ / 1000) * TIMEOUT_MS;
    reg [31:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= 32'd0;
            timeout <= 1'b0;
        end else if (kick) begin
            cnt     <= 32'd0;
            timeout <= 1'b0;
        end else if (!timeout) begin
            if (cnt >= LIMIT - 1) begin
                cnt     <= 32'd0;
                timeout <= 1'b1;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
endmodule
