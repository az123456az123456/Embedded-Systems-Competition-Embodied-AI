// ============================================================
// twist_to_wheel.v — 线速度/角速度转换为左右轮目标速度
// v: mm/s, w: mrad/s, 正 w 表示逆时针左转
// 输出按 TARGET_MAX 归一化，并做限幅。
// ============================================================
module twist_to_wheel #(
    parameter integer V_MAX_MM_S      = 3000,
    parameter integer TRACK_WIDTH_MM  = 160,
    parameter integer TARGET_MAX      = 1000
)(
    input  wire signed [15:0] v_mm_s,
    input  wire signed [15:0] w_mrad_s,
    output reg  signed [15:0] target_left,
    output reg  signed [15:0] target_right
);
    wire signed [31:0] v_ext = v_mm_s;
    wire signed [31:0] w_ext = w_mrad_s;
    wire signed [31:0] wheel_left  = v_ext - (w_ext * TRACK_WIDTH_MM) / 2000;
    wire signed [31:0] wheel_right = v_ext + (w_ext * TRACK_WIDTH_MM) / 2000;
    wire signed [31:0] left_raw  = (wheel_left  * TARGET_MAX) / V_MAX_MM_S;
    wire signed [31:0] right_raw = (wheel_right * TARGET_MAX) / V_MAX_MM_S;

    always @(*) begin
        if (left_raw > TARGET_MAX)
            target_left = TARGET_MAX;
        else if (left_raw < -TARGET_MAX)
            target_left = -TARGET_MAX;
        else
            target_left = left_raw[15:0];

        if (right_raw > TARGET_MAX)
            target_right = TARGET_MAX;
        else if (right_raw < -TARGET_MAX)
            target_right = -TARGET_MAX;
        else
            target_right = right_raw[15:0];
    end
endmodule
