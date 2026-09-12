// ============================================================
// quadrature_decoder.v — 增量编码器正交解码 (4倍频)
// A/B 两路 90° 相位差信号 -> 位置计数 + 方向
// 每发生一次状态跳变计数 +1/-1 (4x 分辨率)
// ============================================================
module quadrature_decoder #(
    parameter WIDTH = 16
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 enc_a,        // 编码器 A 相
    input  wire                 enc_b,        // 编码器 B 相
    output reg  signed [WIDTH-1:0] position,  // 位置计数(带符号)
    output reg                  pulse_tick,   // 每计数一拍脉冲
    output reg                  direction     // 1=正转 0=反转
);

    // ---- 输入同步(打两拍, 消除亚稳态) ----
    reg a_r, a_rr, b_r, b_rr;
    reg [1:0] prev_state;
    wire [1:0] cur_state = {a_rr, b_rr};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_r  <= 1'b0;
            a_rr <= 1'b0;
            b_r  <= 1'b0;
            b_rr <= 1'b0;
            prev_state <= 2'b00;
        end else begin
            a_r  <= enc_a;
            a_rr <= a_r;
            b_r  <= enc_b;
            b_rr <= b_r;
            prev_state <= cur_state;
        end
    end

    // ---- 状态跳变 -> 计数 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position   <= {WIDTH{1'b0}};
            pulse_tick <= 1'b0;
            direction  <= 1'b1;
        end else begin
            pulse_tick <= 1'b0;    // 默认无脉冲
            case ({prev_state, cur_state})
                // 正转序列: 00->01->11->10->00
                4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                    position   <= position + {{WIDTH-1{1'b0}}, 1'b1};
                    direction  <= 1'b1;
                    pulse_tick <= 1'b1;
                end
                // 反转序列: 00->10->11->01->00
                4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                    position   <= position - {{WIDTH-1{1'b0}}, 1'b1};
                    direction  <= 1'b0;
                    pulse_tick <= 1'b1;
                end
                default: ;          // 无变化或非法跳变
            endcase
        end
    end
endmodule
