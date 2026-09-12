// ============================================================
// ultrasonic.v — HC-SR04 超声波测距模块
// 流程: 收到 trigger -> 发10us触发脉冲 -> 等echo高电平
//       高电平时长 -> 距离(cm) = 时长(us)*0.0343/2
// 输出: distance_cm (超时输出 0xFFFF), done(完成脉冲)
// ============================================================
module ultrasonic #(
    parameter CLK_FREQ    = 50_000_000,
    parameter TRIG_US     = 10,       // 触发脉冲宽度(us)
    parameter TIMEOUT_US  = 10_000    // 回波超时(us)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        trigger,       // 触发测量(一拍脉冲)
    input  wire        echo,          // 回波输入
    output reg         trig,          // 触发输出
    output reg  [15:0] distance_cm,   // 距离(cm), 超时=0xFFFF
    output reg         done,          // 完成(一拍脉冲)
    output reg         busy           // 忙标志
);

    localparam TRIG_CLKS     = TRIG_US    * (CLK_FREQ / 1_000_000);   // 500 @50MHz
    localparam TIMEOUT_CLKS  = TIMEOUT_US * (CLK_FREQ / 1_000_000);   // 500k @50MHz

    reg [1:0]  state;
    localparam S_IDLE = 0, S_TRIG = 1, S_ECHO = 2;

    reg [31:0] cnt;          // 触发/超时计数
    reg [31:0] echo_clks;    // echo 高电平时钟数
    reg echo_r, echo_rr;     // echo 同步打拍

    // ---- echo 同步 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin echo_r <= 1'b0; echo_rr <= 1'b0; end
        else begin echo_r <= echo; echo_rr <= echo_r; end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            trig        <= 1'b0;
            distance_cm <= 16'd0;
            done        <= 1'b0;
            busy        <= 1'b0;
            cnt         <= 32'd0;
            echo_clks   <= 32'd0;
        end else begin
            done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (trigger) begin
                        busy  <= 1'b1;
                        cnt   <= 32'd0;
                        state <= S_TRIG;
                    end
                end
                S_TRIG: begin
                    trig <= 1'b1;
                    if (cnt == TRIG_CLKS) begin
                        trig  <= 1'b0;
                        cnt   <= 32'd0;
                        state <= S_ECHO;
                    end else begin
                        cnt <= cnt + 32'd1;
                    end
                end
                S_ECHO: begin
                    if (echo_rr) begin
                        echo_clks <= echo_clks + 32'd1;
                    end else if (echo_clks != 0) begin
                        // 回波结束, 计算距离
                        distance_cm <= (echo_clks * 343) / 1_000_000;
                        done        <= 1'b1;
                        busy        <= 1'b0;
                        echo_clks   <= 32'd0;
                        state       <= S_IDLE;
                    end
                    // 超时保护
                    if (cnt == TIMEOUT_CLKS) begin
                        distance_cm <= 16'hFFFF;
                        done        <= 1'b1;
                        busy        <= 1'b0;
                        echo_clks   <= 32'd0;
                        state       <= S_IDLE;
                    end else begin
                        cnt <= cnt + 32'd1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
