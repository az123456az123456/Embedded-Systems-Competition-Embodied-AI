`timescale 1ns/1ps
// ACG138 / GW5AT-60，板载 50MHz 时钟，P10 第二组单电机测试。
// 配置完成后：等待 2s -> 方向稳定一个 PWM 周期 -> 运行 3s -> 停止。
// 默认 PWM：20kHz，25%；运行时 IN1=1、IN2=0，EN 高有效。
// 上电初值用于 Gowin FPGA 综合；重新配置 FPGA 可重新开始测试。
module motor_group2_test #(
    parameter integer CLK_HZ       = 50_000_000,
    parameter integer PWM_HZ       = 20_000,
    parameter integer DUTY_PERCENT = 25,
    parameter integer WAIT_MS      = 2_000,
    parameter integer RUN_MS       = 3_000
)(
    input  wire clk50m,
    output reg  motor_pwm,  // P10-4 / F16
    output reg  motor_in1,  // P10-5 / A16
    output wire motor_in2,  // P10-6 / A15
    output reg  motor_en    // P10-7 / F14，总使能
);
    localparam integer PWM_CYCLES = CLK_HZ / PWM_HZ;
    localparam integer HIGH_CYCLES = PWM_CYCLES * DUTY_PERCENT / 100;
    localparam integer WAIT_CYCLES = (CLK_HZ / 1000) * WAIT_MS;
    localparam integer RUN_CYCLES = (CLK_HZ / 1000) * RUN_MS;
    localparam [1:0] ST_WAIT = 2'd0, ST_ARM = 2'd1,
                     ST_RUN = 2'd2, ST_STOP = 2'd3;

    reg [1:0] state = ST_WAIT;
    reg [31:0] phase_count = 32'd0;
    reg [31:0] pwm_count = 32'd0;

    initial begin
        motor_en = 1'b0;
        motor_pwm = 1'b0;
        motor_in1 = 1'b0;
    end

    // 输出经寄存器送到引脚，避免组合计数比较产生窄脉冲。
    // 三路同步延迟一拍，不改变 PWM 周期、运行时长或方向建立时间。
    assign motor_in2 = 1'b0;

    always @(posedge clk50m) begin
        motor_en  <= (state == ST_RUN);
        motor_pwm <= (state == ST_RUN) && (pwm_count < HIGH_CYCLES);
        motor_in1 <= (state == ST_ARM) || (state == ST_RUN);
        case (state)
            ST_WAIT: begin
                pwm_count <= 32'd0;
                if ((WAIT_CYCLES == 0) || (phase_count >= WAIT_CYCLES - 1)) begin
                    phase_count <= 32'd0;
                    state <= ST_ARM;
                end else
                    phase_count <= phase_count + 1'b1;
            end
            ST_ARM: begin
                if (phase_count >= PWM_CYCLES - 1) begin
                    phase_count <= 32'd0;
                    state <= ST_RUN;
                end else
                    phase_count <= phase_count + 1'b1;
            end
            ST_RUN: begin
                if (phase_count >= RUN_CYCLES - 1) begin
                    phase_count <= 32'd0;
                    pwm_count <= 32'd0;
                    state <= ST_STOP;
                end else begin
                    phase_count <= phase_count + 1'b1;
                    if (pwm_count >= PWM_CYCLES - 1)
                        pwm_count <= 32'd0;
                    else
                        pwm_count <= pwm_count + 1'b1;
                end
            end
            default: begin
                state <= ST_STOP;
                phase_count <= 32'd0;
                pwm_count <= 32'd0;
            end
        endcase
    end

    // synthesis translate_off
    initial begin
        if ((CLK_HZ < 1000) || (PWM_HZ <= 0) || (PWM_HZ > CLK_HZ)
            || (DUTY_PERCENT < 0) || (DUTY_PERCENT > 100)
            || (WAIT_MS < 0) || (RUN_MS <= 0))
            $fatal(1, "Invalid motor test parameters");
    end
    // synthesis translate_on
endmodule
