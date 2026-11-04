// ============================================================
// motor_pwm.v — 电机 PWM 输出 + 方向
// duty: -TOP..+TOP  (正=正转, 负=反转), 占空比 = |duty|/TOP
// ============================================================
module motor_pwm #(
    parameter TOP = 16'd1000
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] duty,
    output reg                pwm,
    output reg                dir
);
    reg [15:0] cnt;
    reg [15:0] abs_duty;

    always @(*) abs_duty = (duty[15]) ? (-duty) : duty;   // 取绝对值

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 16'd0;
            pwm <= 1'b0;
            dir <= 1'b0;
        end else begin
            if (cnt >= TOP - 1) cnt <= 16'd0;
            else                cnt <= cnt + 16'd1;
            dir <= ~duty[15];                        // 1=正转 0=反转
            if (abs_duty >= TOP) pwm <= 1'b1;        // 满占空比
            else                 pwm <= (cnt < abs_duty) ? 1'b1 : 1'b0;
        end
    end
endmodule
