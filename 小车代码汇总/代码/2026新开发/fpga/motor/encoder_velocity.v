// ============================================================
// encoder_velocity.v — 编码器测速
// 在一个固定时间窗口内累计脉冲数(带符号) -> speed
// 窗口越小更新越快, 分辨率越低; 仿真用小窗口
// ============================================================
module encoder_velocity #(
    parameter WINDOW = 16'd5000      // 测速窗口(时钟数)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               pulse_tick,
    input  wire               direction,
    output reg  signed [15:0] speed
);
    reg [15:0] wcnt;
    reg signed [15:0] acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wcnt  <= 16'd0;
            acc   <= 16'sd0;
            speed <= 16'sd0;
        end else if (wcnt == WINDOW - 1) begin
            speed <= acc;          // 锁存本窗口测速结果
            acc   <= 16'sd0;
            wcnt  <= 16'd0;
        end else begin
            wcnt <= wcnt + 16'd1;
            if (pulse_tick)
                acc <= (direction) ? acc + 16'sd1 : acc - 16'sd1;
        end
    end
endmodule
