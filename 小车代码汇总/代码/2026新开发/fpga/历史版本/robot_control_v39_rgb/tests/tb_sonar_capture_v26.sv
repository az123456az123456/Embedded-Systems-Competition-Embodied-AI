`timescale 1ns/1ps
// Drive real serial pins through UART receivers, three independent filters and
// the unchanged 64-byte SNS1 producer. The emitted trace is consumable by RK.
module tb_sonar_capture_v26;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg [2:0] rx = 3'b111;
    wire valid;
    wire [511:0] packet;
    robot_sensor_capture #(.CLK_FREQ(100000), .BAUD(10000)) dut (
        .clk(clk), .rst_n(rst_n), .encoder_a(4'd0), .encoder_b(4'd0),
        .sonar_rx(rx), .sensor_valid(valid), .sensor_packet(packet)
    );
    wire [31:0] default_clock_checks, boundary_clock_checks;
    sonar_ma_clock_phase_probe #(.CLK_FREQ(50000000), .BAUD(9600)) default_clock (
        .clk(clk),.rst_n(rst_n),.phase_checks(default_clock_checks));
    sonar_ma_clock_phase_probe #(.CLK_FREQ(1000), .BAUD(100)) boundary_clock (
        .clk(clk),.rst_n(rst_n),.phase_checks(boundary_clock_checks));
    integer trace_fd, checks = 0, packets = 0, age_checks = 0, j, i;
    reg [511:0] latest = 0;
    bit trace_enabled = 1;
    reg [31:0] tick_stamp [0:2], last_source [0:2];
    reg [2:0] have_source = 0;
    reg [31:0] source, advance, first_source, left_source, right_source, last_sequence = 0;
    reg [15:0] sample_age;
    always @(posedge clk) if (rst_n && dut.tick_ms) begin
        // Capture the state actually visible to SNS1 at this same edge, before
        // nonblocking assignments can incorporate any coincident new frame.
        tick_stamp[0] = dut.sonar.channel[0].u_driver.u_sensor.distance_stamp_ms;
        tick_stamp[1] = dut.sonar.channel[1].u_driver.u_sensor.distance_stamp_ms;
        tick_stamp[2] = dut.sonar.channel[2].u_driver.u_sensor.distance_stamp_ms;
    end
    always @(negedge clk) begin
        if (!rst_n) begin have_source = 0; last_sequence = 0; end
        else if (valid) begin
            latest = packet; packets = packets + 1;
            if (packet[511:480] != 32'h534e5331 || packet[479:472] != 1 || packet[7:0] != 0)
                $fatal(1,"SNS1 magic/version/reserved bytes changed");
            if ((packet[471:464] | packet[463:456]) != 7 ||
                (packet[471:464] & packet[463:456]) != 0)
                $fatal(1,"SNS1 valid/fault masks are not complementary");
            if (packet[447:416] != last_sequence+1) $fatal(1,"SNS1 packet sequence skips/regresses");
            last_sequence = packet[447:416];
            if (packet[383:128] != 0 || packet[455:448] != 0 || packet[15:8] != 0)
                $fatal(1,"stationary encoders/inversion/illegal fields changed");
            if (packet[31:16] != 0 && packet[31:16] != 50)
                $fatal(1,"SNS1 event or periodic window format changed");
            for (j = 0; j < 3; j = j + 1) if (packet[464+j]) begin
                sample_age = packet[79-j*16 -: 16];
                source = packet[415:384]-sample_age;
                if (sample_age > 1000) $fatal(1,"online channel%0d carries stale source age%0d",j,sample_age);
                if (source != tick_stamp[j])
                    $fatal(1,"SNS1 source time falsified ch%0d expected%0d got%0d",j,tick_stamp[j],source);
                if (have_source[j]) begin
                    advance = source-last_source[j];
                    if (advance >= 32'h80000000) $fatal(1,"oldest source timestamp regressed on channel%0d",j);
                end
                have_source[j] = 1; last_source[j] = source; age_checks = age_checks + 1;
            end
            if (trace_enabled) $fdisplay(trace_fd,"%0128h",packet);
        end
    end
    task automatic ticks(input integer n); repeat(n) @(negedge clk); #1; endtask
    task automatic byte_in(input integer ch, input [7:0] value);
        integer bit_index;
        begin
            @(negedge clk); rx[ch] = 0; ticks(10);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index+1) begin
                rx[ch] = value[bit_index]; ticks(10);
            end
            rx[ch] = 1; ticks(10);
        end
    endtask
    task automatic frame(input integer ch, input [15:0] mm, input bit bad_crc);
        reg [7:0] checksum;
        begin
            checksum = 8'hff + mm[15:8] + mm[7:0];
            byte_in(ch,8'hff); byte_in(ch,mm[15:8]); byte_in(ch,mm[7:0]); byte_in(ch,checksum+bad_crc);
            ticks(110);
        end
    endtask
    task check(input bit condition, input string label);
        begin
            checks = checks+1;
            if (!condition) $fatal(1,"%s valid=%b fault=%b distances=%0d/%0d/%0d front_age=%0d",label,
                latest[471:464],latest[463:456],latest[127:112],latest[111:96],latest[95:80],latest[79:64]);
        end
    endtask
    initial begin
        trace_fd = $fopen("reports/sonar-v26-sns.hex","w");
        if (!trace_fd) $fatal(1,"cannot open SNS1 trace");
        ticks(5); rst_n = 1; ticks(30);
        fork frame(0,1000,0); frame(1,2000,0); frame(2,3000,0); join
        check(latest[471:464] == 7, "three simultaneous UART inputs become valid");
        check(latest[127:80] == {16'd1000,16'd2000,16'd3000}, "channel ordering remains front/left/right");
        first_source = latest[415:384]-latest[79:64];
        left_source = latest[415:384]-latest[63:48]; right_source = latest[415:384]-latest[47:32];
        ticks(10000); frame(0,1002,0);
        check(latest[127:112] == 1001 && latest[79:64] >= 100,
            "UART pair publishes rounded mean with oldest contributor age");
        check(latest[415:384]-latest[79:64] == first_source,
            "SNS1 averaging never stamps previous contributor as newly received");
        check(latest[111:80] == {16'd2000,16'd3000} &&
            latest[415:384]-latest[63:48] == left_source && latest[415:384]-latest[47:32] == right_source,
            "front updates preserve both independent channel values and source identities");
        ticks(9000); frame(0,1004,0); check(latest[127:112] == 1002, "UART warmup count3 mean");
        ticks(9000); frame(0,1006,0); check(latest[127:112] == 1003, "UART count4 mean");
        ticks(6000); frame(0,1008,0);
        check(latest[127:112] == 1005 && latest[415:384]-latest[79:64] > first_source,
            "expired first contributor leaves source window and oldest time advances");
        frame(2,250,0);
        check(latest[95:80] == 250 && latest[127:96] == {16'd1005,16'd2000},
            "right abrupt near acts immediately without changing front/left");
        frame(0,1,0);
        check(!latest[464] && latest[456] && latest[466:465] == 3,
            "front invalid numeric only faults its own channel");
        frame(0,1000,0); check(latest[464] && latest[127:112] == 1000, "ordinary recovery has fresh single-sample history");
        frame(0,250,0); first_source = latest[415:384]-latest[79:64];
        frame(0,4501,0);
        check(latest[127:112] == 250 && latest[415:384]-latest[79:64] == first_source,
            "single numeric high echo holds old near and old source timestamp");
        frame(0,65534,0); check(latest[464] && latest[127:112] == 4500, "numeric high far pair clamps to4500 in unchangedSNS1");
        frame(0,65535,0); check(!latest[464] && latest[456], "FFFF remains invalid in actual packet");
        frame(0,250,0); frame(0,2000,0); frame(0,3000,1); frame(0,4000,0);
        check(latest[127:112] == 250, "real UART badCRC breaks far confirmation");
        frame(0,4200,0); check(latest[127:112] == 4100, "fresh far pair mean follows badCRC");
        frame(0,250,0); frame(0,2000,0); byte_in(0,8'hff); byte_in(0,8'h01); ticks(2100);
        frame(0,4000,0); check(latest[127:112] == 250, "real UART partial timeout breaks far confirmation");
        frame(0,4200,0); check(latest[127:112] == 4100, "real UART partial timeout recovers with new pair");
        frame(0,250,0); frame(0,1200,0); ticks(50000); frame(0,2400,0);
        check(latest[464] && latest[127:112] == 2400 && latest[79:64] <= 2,
            "500ms sparse far pair confirms but excludes expired contributor");
        frame(0,250,0);
        for (i = 0; i < 20; i = i+1) begin
            ticks(9600); frame(0,i%2 ? 2400 : 1200,0);
            if (i > 0) check(latest[464] && latest[79:64] <= 305,
                "variable far UART readings sustain online data within source span");
        end
        ticks(100200); check(latest[471:464] == 0 && latest[463:456] == 7,
            "three disconnected serial channels expire independently to allfault");
        // The host replay harness derives host time from device uptime. Keep
        // that trace to one boot; still exercise the hardware reset below.
        trace_enabled = 0;
        ticks(5); rst_n = 0; ticks(5); rst_n = 1; ticks(20);
        fork frame(0,999,0); frame(1,1999,0); frame(2,2999,0); join
        check(latest[471:464] == 7 && latest[127:80] == {16'd999,16'd1999,16'd2999},
            "reset discards all channel histories and pending confirmations");
        $fclose(trace_fd);
        $display("PASS sonar MA v26 real UART -> SNS1: %0d checks, %0d packets, %0d oldest-source serialization/monotonicity checks; three-channel isolation, mean/warmup/span/near/far/clamp/invalid/CRC/partial/data timeout/reset",
            checks,packets,age_checks);
        $display("Clock phase: %0d cycles at default50MHz and %0d cycles at CLK_FREQ1000 agree with unchanged capture tick/uptime and independent reference",
            default_clock_checks,boundary_clock_checks);
        $finish;
    end
    initial begin #30000000; $fatal(1,"test timeout"); end
endmodule

// Compare the new predecoded decoder tick/shadow with the unchanged capture
// counter and an independent reference. CLK_FREQ1000 exercises one clock/ms.
module sonar_ma_clock_phase_probe #(
    parameter integer CLK_FREQ=50000000,
    parameter integer BAUD=9600
)(input wire clk,rst_n,output reg [31:0] phase_checks=0);
    localparam integer CYCLES_PER_MS=CLK_FREQ/1000;
    reg [31:0] reference_div=0, reference_uptime=0;
    robot_sensor_capture #(.CLK_FREQ(CLK_FREQ),.BAUD(BAUD)) capture (
        .clk(clk),.rst_n(rst_n),.encoder_a(4'd0),.encoder_b(4'd0),.sonar_rx(3'b111),
        .sensor_valid(),.sensor_packet());
    always @(posedge clk) begin
        if (!rst_n) begin reference_div=0; reference_uptime=0; end
        else if (reference_div == CYCLES_PER_MS-1) begin
            reference_div=0; reference_uptime=reference_uptime+1;
        end else reference_div=reference_div+1;
        #1;
        if (rst_n) begin
            if (capture.uptime_ms != reference_uptime || capture.ms_div != reference_div ||
                capture.tick_ms != (reference_div == CYCLES_PER_MS-1))
                $fatal(1,"unchanged capture clock differs from independent reference at CLK_FREQ%0d",CLK_FREQ);
            phase_checks=phase_checks+1;
        end
    end
    genvar channel_index;
    generate for (channel_index=0;channel_index<3;channel_index=channel_index+1) begin: clock_channel
        always @(posedge clk) begin
            #1;
            if (rst_n && (capture.sonar.channel[channel_index].u_driver.u_sensor.uptime_ms != reference_uptime ||
                capture.sonar.channel[channel_index].u_driver.u_sensor.uptime_next_ms != reference_uptime+32'd1 ||
                capture.sonar.channel[channel_index].u_driver.u_sensor.tick_ms != capture.tick_ms))
                $fatal(1,"decoder shadow/tick phase diverges from capture at CLK_FREQ%0d channel%0d",CLK_FREQ,channel_index);
        end
    end endgenerate
endmodule
