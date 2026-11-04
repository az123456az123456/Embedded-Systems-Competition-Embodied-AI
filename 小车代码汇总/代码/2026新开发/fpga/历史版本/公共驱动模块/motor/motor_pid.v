// ============================================================
// motor_pid.v — 电机速度 PID 控制器 (位置式, 带积分饱和/输出限幅)
// 控制周期: 每 CONTROL_DIV 个时钟计算一次
// 输出 duty: -OUT_MAX..+OUT_MAX (正=正转)
// ============================================================
module motor_pid #(
    parameter integer KP         = 8,
    parameter integer KI         = 1,
    parameter integer KD         = 2,
    parameter integer OUT_MAX    = 1000,
    parameter integer INT_LIMIT  = 16000,     // 积分饱和限制
    parameter integer CONTROL_DIV = 100       // 控制周期分频(时钟数)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire signed [15:0] target,
    input  wire signed [15:0] actual,
    output reg  signed [15:0] duty
);
    reg  [15:0] cdiv;
    reg  signed [31:0] integ;
    reg  signed [15:0] err_prev;

    wire signed [15:0] err = target - actual;
    wire signed [31:0] next_integ = integ + err * KI;
    wire signed [31:0] duty_sum   = err * KP + integ + (err - err_prev) * KD;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cdiv     <= 16'd0;
            integ    <= 32'sd0;
            err_prev <= 16'sd0;
            duty     <= 16'sd0;
        end else if (!enable) begin
            cdiv     <= 16'd0;
            integ    <= 32'sd0;
            err_prev <= 16'sd0;
            duty     <= 16'sd0;
        end else if (cdiv == CONTROL_DIV - 1) begin
            cdiv <= 16'd0;
            // 积分更新(抗饱和)
            if (next_integ > INT_LIMIT)      integ <= INT_LIMIT;
            else if (next_integ < -INT_LIMIT) integ <= -INT_LIMIT;
            else                              integ <= next_integ;
            err_prev <= err;
            // 输出限幅
            if (duty_sum > OUT_MAX)      duty <= OUT_MAX;
            else if (duty_sum < -OUT_MAX) duty <= -OUT_MAX;
            else                          duty <= duty_sum;
        end else begin
            cdiv <= cdiv + 16'd1;
        end
    end
endmodule
