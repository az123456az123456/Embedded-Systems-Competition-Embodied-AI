`timescale 1ns/1ps
module tb_motor_group2_test;
    reg clk50m = 1'b0;
    always #10 clk50m = ~clk50m;

    wire pwm, in1, in2, en;
    wire pwm_zero, en_zero, pwm_full, en_full;
    // 保留真实 50MHz / 20kHz 时序，仅把等待和运行缩短到 1ms。
    motor_group2_test #(.WAIT_MS(1), .RUN_MS(1)) dut (
        .clk50m(clk50m), .motor_pwm(pwm), .motor_in1(in1),
        .motor_in2(in2), .motor_en(en)
    );
    motor_group2_test #(.WAIT_MS(1), .RUN_MS(1), .DUTY_PERCENT(0)) zero_duty (
        .clk50m(clk50m), .motor_pwm(pwm_zero), .motor_in1(),
        .motor_in2(), .motor_en(en_zero)
    );
    motor_group2_test #(.WAIT_MS(1), .RUN_MS(1), .DUTY_PERCENT(100)) full_duty (
        .clk50m(clk50m), .motor_pwm(pwm_full), .motor_in1(),
        .motor_in2(), .motor_en(en_full)
    );

    integer cycle;
    integer run_samples = 0;
    integer high_samples = 0;
    integer armed_samples = 0;
    reg started = 1'b0;
    reg stopped = 1'b0;

    initial begin
        #1;
        if ({en, pwm, in1, in2} !== 4'b0000)
            $fatal(1, "Outputs must start disabled");
        // 多观察 3ms，验证完成后不会自动重新启动。
        for (cycle = 1; cycle <= 250000; cycle = cycle + 1) begin
            @(negedge clk50m);
            if (({en, pwm, in1, in2} === 4'bxxxx)
                || (in2 !== 1'b0))
                $fatal(1, "Invalid direction outputs at cycle %0d", cycle);
            if ((en_zero !== en) || (en_full !== en))
                $fatal(1, "Duty must not change enable timing");
            if (pwm_zero !== 1'b0)
                $fatal(1, "0 percent duty is not low");
            if (pwm_full !== en_full)
                $fatal(1, "100 percent duty does not follow enable");

            if (en === 1'b1) begin
                if (stopped || (in1 !== 1'b1))
                    $fatal(1, "Unexpected restart or direction");
                if (!started && ((cycle != 52501) || (armed_samples != 2500)))
                    $fatal(1, "Wrong startup delay or direction settling time");
                started = 1'b1;
                // 规范：每 2500 拍一周期，前 625 拍高，后 1875 拍低。
                if (pwm !== ((run_samples % 2500) < 625))
                    $fatal(1, "Wrong PWM period or duty at sample %0d", run_samples);
                run_samples = run_samples + 1;
                if (pwm) high_samples = high_samples + 1;
            end else if (en === 1'b0) begin
                if (pwm !== 1'b0)
                    $fatal(1, "PWM is active while disabled");
                if (started) begin
                    stopped = 1'b1;
                    if (in1 !== 1'b0)
                        $fatal(1, "Direction must return low after stopping");
                end else if (in1 === 1'b1)
                    armed_samples = armed_samples + 1;
                else if ((cycle >= 50001) || (in1 !== 1'b0))
                    $fatal(1, "Wrong waiting interval");
            end else
                $fatal(1, "Enable is unknown");
        end
        if (!stopped || (run_samples != 50000) || (high_samples != 12500))
            $fatal(1, "Wrong run duration or total duty");
        $display("PASS: startup, direction settling, 20kHz/25%% PWM, 0%%/100%% duty, one-shot stop");
        $finish;
    end
endmodule
