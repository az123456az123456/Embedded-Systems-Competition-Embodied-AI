`timescale 1ns/1ps
// Independent frame-level scoreboard. The model consumes the bytes sent by
// this test, not DUT history, sum, count, candidate or timestamp registers.
module tb_sonar_moving_average_v26;
    localparam integer CYCLES_PER_MS = 10;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg [7:0] rx_byte = 0;
    reg rx_valid = 0;
    wire [15:0] distance_mm, distance_age_ms;
    wire distance_valid, online, sensor_fault, obstacle, slow_down, stop_req;
    ultrasonic_sensor #(.CLK_FREQ(10000)) dut (
        .clk(clk), .rst_n(rst_n), .rx_byte(rx_byte), .rx_valid(rx_valid),
        .distance_mm(distance_mm), .distance_valid(distance_valid),
        .distance_age_ms(distance_age_ms), .online(online), .sensor_fault(sensor_fault),
        .obstacle(obstacle), .slow_down(slow_down), .stop_req(stop_req)
    );

    integer tb_div = 0, tb_cycles = 0, pulses = 0, clock_phase_checks = 0;
    reg [31:0] tb_ms = 0;
    always @(posedge clk) begin
        if (!rst_n) begin tb_div = 0; tb_ms = 0; tb_cycles = 0; end
        else begin
            tb_cycles = tb_cycles + 1;
            if (tb_div == CYCLES_PER_MS-1) begin tb_div = 0; tb_ms = tb_ms + 1; end
            else tb_div = tb_div + 1;
        end
        #1;
        if (rst_n) begin
            if (dut.uptime_ms != tb_ms || dut.uptime_next_ms != tb_ms+32'd1 ||
                dut.tick_ms != (tb_div == CYCLES_PER_MS-1))
                $fatal(1,"registered tick/shadow differs from independent clock: ref_ms%0d dut_ms%0d shadow%0d div%0d tick%b",
                    tb_ms,dut.uptime_ms,dut.uptime_next_ms,tb_div,dut.tick_ms);
            clock_phase_checks = clock_phase_checks+1;
            if (distance_valid) pulses = pulses+1;
        end
    end

    integer checks = 0, frames = 0, accepted = 0, random_frames = 0;
    integer history_mm [0:3];
    reg [31:0] history_stamp [0:3];
    integer history_count = 0, expected_mm = 0;
    reg [31:0] expected_stamp = 0;
    reg [31:0] expected_candidate_stamp = 0, expected_last_numeric_stamp = 0;
    bit expected_have = 0, expected_online = 0, expected_far = 0;
    bit expected_obstacle = 1, expected_slow = 1, expected_stop = 1, publish = 0;
    reg [31:0] rng = 32'h5a17c9e3;
    integer i, before_pulses, steady_min, steady_max, generated, gap_ms;
    reg [31:0] first_stamp;
    integer max_normal_latency = 0, queued_cancellations = 0;
    integer probe_before_mm, probe_before_pulses;

    task check(input bit condition, input string label);
        begin
            checks = checks + 1;
            if (!condition)
                $fatal(1, "%s frame=%0d t=%0d mm=%0d/%0d age=%0d stamp=%0d online=%b/%b hist=%0d far=%b",
                    label, frames, tb_ms, distance_mm, expected_mm, distance_age_ms,
                    expected_stamp, online, expected_online, history_count, expected_far);
        end
    endtask
    task ticks(input integer n); repeat(n) @(negedge clk); #1; endtask
    task clear_history;
        begin history_count = 0; expected_far = 0; end
    endtask
    task model_timeout;
        reg [31:0] elapsed;
        begin
            elapsed = tb_ms - expected_candidate_stamp;
            if (expected_far && elapsed >= 1000) clear_history();
            elapsed = tb_ms - expected_last_numeric_stamp;
            if (history_count != 0 && elapsed >= 1000) clear_history();
            elapsed = tb_ms - expected_stamp;
            if (expected_have && expected_online && elapsed >= 1000) begin
                expected_online = 0;
                expected_obstacle = 1; expected_slow = 1; expected_stop = 1;
                clear_history();
            end
        end
    endtask
    task verify_state(input string label);
        reg [31:0] elapsed;
        integer expected_age;
        begin
            model_timeout();
            elapsed = tb_ms - expected_stamp;
            expected_age = !expected_have || elapsed >= 65535 ? 65535 : elapsed;
            check(distance_mm == expected_mm, {label, ": distance"});
            check(distance_age_ms == expected_age, {label, ": oldest contributor age"});
            check(online == expected_online && sensor_fault == !expected_online,
                {label, ": online/fault"});
            check({obstacle,slow_down,stop_req} == {expected_obstacle,expected_slow,expected_stop},
                {label, ": hysteresis requests"});
        end
    endtask
    task idle_ms(input integer n);
        integer previous_pulses;
        begin
            previous_pulses = pulses;
            ticks(n*CYCLES_PER_MS);
            verify_state("idle");
            check(pulses == previous_pulses, "elapsed time alone never publishes a measurement");
        end
    endtask
    task append_sample(input integer mm);
        integer read_index, retained;
        reg [31:0] span;
        begin
            retained = 0;
            for (read_index = 0; read_index < history_count; read_index = read_index + 1) begin
                span = tb_ms - history_stamp[read_index];
                if (span <= 300) begin
                    history_mm[retained] = history_mm[read_index];
                    history_stamp[retained] = history_stamp[read_index];
                    retained = retained + 1;
                end
            end
            if (retained == 4) begin
                for (read_index = 0; read_index < 3; read_index = read_index + 1) begin
                    history_mm[read_index] = history_mm[read_index+1];
                    history_stamp[read_index] = history_stamp[read_index+1];
                end
                retained = 3;
            end
            history_mm[retained] = mm; history_stamp[retained] = tb_ms;
            history_count = retained + 1;
        end
    endtask
    task model_frame(input integer received, input bit bad_crc);
        integer mm, sum, k;
        begin
            publish = 0;
            model_timeout();
            if (bad_crc) clear_history();
            else if (received == 0) begin
                clear_history(); expected_mm = 4500; expected_stamp = tb_ms;
                expected_last_numeric_stamp = tb_ms; expected_have = 1; expected_online = 1;
                expected_obstacle = 0; expected_slow = 0; expected_stop = 0;
                publish = 1; accepted = accepted+1;
            end else if (received < 30 || received == 65535) begin
                clear_history(); expected_online = 0;
                expected_obstacle = 1; expected_slow = 1; expected_stop = 1;
            end else begin
                mm = received > 4500 ? 4500 : received;
                if (!expected_have || mm <= 350 || mm+50 <= expected_mm) begin
                    clear_history(); append_sample(mm); publish = 1;
                end else if (mm > expected_mm+200) begin
                    if (!expected_far) begin
                        clear_history(); append_sample(mm); expected_far = 1;
                        expected_candidate_stamp = tb_ms;
                    end else begin
                        append_sample(mm); expected_far = 0; publish = 1;
                    end
                end else begin
                    if (expected_far) clear_history();
                    append_sample(mm); publish = 1;
                end
                expected_last_numeric_stamp = tb_ms;
                if (publish) begin
                    sum = 0;
                    for (k = 0; k < history_count; k = k + 1) sum = sum + history_mm[k];
                    expected_mm = (sum + history_count/2) / history_count;
                    expected_stamp = history_stamp[0];
                    expected_have = 1; expected_online = 1;
                    if (expected_mm <= 1200) expected_obstacle = 1;
                    else if (expected_mm >= 1250) expected_obstacle = 0;
                    if (expected_mm <= 800) expected_slow = 1;
                    else if (expected_mm >= 850) expected_slow = 0;
                    if (expected_mm <= 300) expected_stop = 1;
                    else if (expected_mm >= 350) expected_stop = 0;
                    accepted = accepted + 1;
                end
            end
        end
    endtask
    task byte_in(input [7:0] value);
        begin @(negedge clk); rx_byte = value; rx_valid = 1;
            @(negedge clk); rx_valid = 0; #1; end
    endtask
    task frame(input [15:0] value, input bit bad_crc);
        reg [7:0] checksum;
        integer previous_pulses, latency, observed_latency, old_mm, clamped;
        reg [31:0] old_stamp;
        bit old_have, old_online, immediate, cancelled;
        begin
            previous_pulses = pulses;
            checksum = 8'hff + value[15:8] + value[7:0];
            byte_in(8'hff); byte_in(value[15:8]); byte_in(value[7:0]); byte_in(checksum+bad_crc);
            frames = frames + 1;
            old_mm = expected_mm; old_stamp = expected_stamp;
            old_have = expected_have;
            old_online = expected_online && tb_ms-old_stamp < 1000;
            clamped = value > 4500 ? 4500 : value;
            immediate = !bad_crc && (value == 0 || (value >= 30 && value != 65535 &&
                (clamped <= 350 || (old_have && clamped+50 <= old_mm))));
            model_frame(value, bad_crc);
            if (immediate) check(distance_valid, "safety near publishes on the checksum clock without pipeline delay");
            observed_latency = distance_valid ? 0 : -1;
            cancelled = 0;
            for (latency = 1; latency <= 4; latency = latency+1) begin
                ticks(1);
                // The existing publication remains the freshness baseline until
                // commit. Its online->offline transition cancels queued work.
                if (publish && observed_latency < 0 && old_have && old_online &&
                    tb_ms-old_stamp >= 1000) cancelled = 1;
                if (cancelled) check(!distance_valid, "source expiry cancels queued normal result, including same-clock commit");
                if (distance_valid && observed_latency < 0) observed_latency = latency;
                if (observed_latency < 0) check(distance_mm == old_mm, "queued normal result cannot change distance before its publish pulse");
            end
            if (cancelled) begin
                expected_mm = old_mm; expected_stamp = old_stamp; expected_have = old_have;
                expected_online = 0; expected_obstacle = 1; expected_slow = 1; expected_stop = 1;
                clear_history(); publish = 0; accepted = accepted-1; queued_cancellations = queued_cancellations+1;
            end
            check((observed_latency >= 0) == publish, "normal publication arrives within four clocks or is explicitly cancelled");
            check(pulses-previous_pulses == (publish ? 1 : 0), "exactly one pulse per accepted frame");
            if (publish && !immediate && observed_latency > max_normal_latency) max_normal_latency = observed_latency;
            verify_state("frame");
            ticks(2);
            check(!distance_valid, "publish pulse is one clock");
        end
    endtask
    task frame_at_stamp(input [15:0] value, input [31:0] target);
        integer wait_cycles;
        begin
            wait_cycles = (target-tb_ms)*CYCLES_PER_MS - tb_div - 8;
            check(wait_cycles >= 0, "test can schedule exact boundary stamp");
            ticks(wait_cycles); frame(value,0);
            check(tb_ms == target || tb_ms == target+1, "boundary frame clock schedule");
        end
    endtask
    // Cancellation probes deliberately leave a normal result queued. No model
    // update is performed; each probe resets before rejoining the scoreboard.
    task raw_frame_without_wait(input [15:0] value);
        reg [7:0] checksum;
        begin
            checksum = 8'hff + value[15:8] + value[7:0];
            byte_in(8'hff); byte_in(value[15:8]); byte_in(value[7:0]); byte_in(checksum);
            frames = frames+1;
        end
    endtask
    task burst_frame(input [15:0] value, input bit bad_crc, output integer before_crc_mm, output integer before_crc_pulses);
        reg [7:0] checksum;
        begin
            checksum = 8'hff + value[15:8] + value[7:0];
            rx_byte = 8'hff; rx_valid = 1;
            @(negedge clk); rx_byte = value[15:8];
            @(negedge clk); rx_byte = value[7:0];
            @(negedge clk); #1; before_crc_mm = distance_mm; before_crc_pulses = pulses; rx_byte = checksum+bad_crc;
            @(negedge clk); rx_valid = 0; #1; frames = frames+1;
        end
    endtask
    task reset_sensor;
        begin
            @(negedge clk); rst_n = 0; rx_valid = 0;
            ticks(3);
            expected_mm = 0; expected_stamp = 0; expected_have = 0; expected_online = 0;
            expected_candidate_stamp = 0; expected_last_numeric_stamp = 0;
            expected_obstacle = 1; expected_slow = 1; expected_stop = 1; clear_history();
            @(negedge clk); rst_n = 1; ticks(2);
            verify_state("reset");
        end
    endtask
    task next_random;
        begin
            rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
        end
    endtask

    initial begin
        reset_sensor(); idle_ms(25);
        frame(1000,0); check(distance_mm == 1000, "first valid sample has no artificial warmup");
        frame(1001,0); check(distance_mm == 1001, "count2 halves round up");
        frame(1002,0); check(distance_mm == 1001, "count3 uses actual contributor count");
        frame(1003,0); check(distance_mm == 1002, "count4 halves round up");
        frame(1004,0); check(distance_mm == 1003, "fifth sample evicts oldest of four");
        steady_min = 65535; steady_max = 0;
        for (i = 0; i < 80; i = i + 1) begin
            idle_ms(50); frame(i%2 ? 1020 : 980,0);
            if (i >= 4) begin
                if (distance_mm < steady_min) steady_min = distance_mm;
                if (distance_mm > steady_max) steady_max = distance_mm;
            end
        end
        check(steady_min == 1000 && steady_max == 1000,
            "40mm raw peak-to-peak jitter becomes steady1000mm over four frames");
        frame(950,0); check(distance_mm == 950, "exact50mm drop bypasses mean immediately");
        frame(951,0); check(distance_mm == 951, "near override restarts count with raw sample");
        frame(350,0); check(distance_mm == 350 && !stop_req, "350mm emergency band publishes raw");
        idle_ms(20); frame(349,0); check(distance_mm == 349 && distance_age_ms <= 1,
            "emergency band repeats always restart history/current timestamp");
        frame(250,0); first_stamp = expected_stamp; idle_ms(100); frame(1200,0);
        check(distance_mm == 250 && expected_stamp == first_stamp, "single far echo never refreshes retained near age");
        first_stamp = tb_ms; idle_ms(100); frame(2400,0);
        check(distance_mm == 1800 && distance_age_ms >= 100,
            "far pair averages new scene and retains first contributor age");
        for (i = 0; i < 30; i = i + 1) begin
            idle_ms(90); frame(i%2 ? 2400 : 1200,0);
            check(online, "variable far scene cannot starve online measurement");
        end
        frame(250,0); frame(1200,0); idle_ms(500); frame(2400,0);
        check(distance_mm == 2400 && distance_age_ms <= 1,
            "500ms gap excludes old far contributor but allows two-observation clearance");
        reset_sensor(); frame(1000,0); first_stamp = expected_stamp;
        frame_at_stamp(1002, first_stamp+300);
        check(distance_mm == 1001 && distance_age_ms >= 300,
            "exact300ms source span retains oldest sample");
        reset_sensor(); frame(1000,0); first_stamp = expected_stamp;
        frame_at_stamp(1002, first_stamp+301);
        check(distance_mm == 1002 && distance_age_ms <= 1,
            "301ms source span excludes stale contributor");
        frame(1,0); frame(1000,0); check(distance_mm == 1000, "invalid resets mean history");
        for (i = 1; i < 30; i = i + 1) frame(i,0);
        frame(65535,0); frame(30,0); check(distance_mm == 30 && online, "30mm is valid boundary");
        frame(4501,0); frame(65534,0); check(distance_mm == 4500 && online, "numeric high readings clamp before averaging");
        frame(6000,0); frame(65534,0); frame(4501,0); check(distance_mm == 4500, "four4500mm sum cannot overflow");
        frame(250,0); frame(2000,0); frame(3000,1); frame(4000,0);
        check(distance_mm == 250, "badCRC breaks old far confirmation");
        frame(4200,0); check(distance_mm == 4100, "new far pair after badCRC averages new values");
        frame(250,0); frame(2000,0); byte_in(8'hff); byte_in(8'h01); idle_ms(21); clear_history();
        frame(4000,0); check(distance_mm == 250, "partial frame timeout breaks old far confirmation");
        frame(4200,0); check(distance_mm == 4100, "partial timeout recovery uses fresh pair");
        frame(250,0); idle_ms(900); frame(2000,0); idle_ms(101); frame(3000,0);
        check(distance_mm == 250 && !online, "source timeout transition cannot reuse pretimeout candidate");
        frame(3500,0); check(distance_mm == 3250 && online, "two posttimeout far readings recover");
        reset_sensor(); frame(250,0); first_stamp = expected_stamp;
        idle_ms(900); frame(2000,0); frame_at_stamp(3000,first_stamp+1000);
        check(distance_mm == 250 && !online,
            "checksum exactly at1000ms source age cannot reuse preexpiry far candidate");
        frame(3500,0); check(distance_mm == 3250 && online,
            "same-cycle source ageout starts a new confirmation pair");
        frame(250,0); frame(1,0); frame(1000,0); first_stamp = expected_candidate_stamp;
        frame_at_stamp(2000,first_stamp+1000);
        check(distance_mm == 250 && !online, "candidate exactly1000ms old cannot confirm while already offline");
        frame(3000,0); check(distance_mm == 2500 && online,
            "candidate expiration while offline requires two new readings");
        for (i = 0; i < 4; i = i+1) begin
            reset_sensor(); frame(1000,0); raw_frame_without_wait(1002);
            check(!distance_valid && distance_mm == 1000, "normal mean is queued before reset probe");
            ticks(i); rst_n = 0; reset_sensor(); idle_ms(2);
            check(distance_mm == 0 && !online, "reset at every pipeline stage cancels mean without late resurrection");
        end
        frame(1000,0); raw_frame_without_wait(1002);
        burst_frame(1, 0, probe_before_mm, probe_before_pulses);
        check(!distance_valid && !online && distance_mm == probe_before_mm && pulses == probe_before_pulses,
            "invalid CRC coincident with queued commit wins without a publish pulse");
        before_pulses = pulses; ticks(8);
        check(!distance_valid && !online && pulses == before_pulses,
            "invalid numeric cannot be followed by a quiet late queued resurrection");
        reset_sensor(); frame(1000,0); raw_frame_without_wait(1002);
        burst_frame(1004, 1, probe_before_mm, probe_before_pulses);
        check(!distance_valid && online && distance_mm == probe_before_mm && pulses == probe_before_pulses,
            "bad checksum coincident with queued commit cancels it while retaining baseline online state");
        before_pulses = pulses; ticks(8);
        check(distance_mm == probe_before_mm && online && pulses == before_pulses,
            "bad checksum cannot be followed by a quiet late queued resurrection");
        reset_sensor(); frame(1000,0); raw_frame_without_wait(1002);
        burst_frame(250, 0, probe_before_mm, probe_before_pulses);
        check(distance_valid && distance_mm == 250 && online && pulses == probe_before_pulses+1,
            "fast near coincident with queued mean commit publishes only raw near");
        before_pulses = pulses; ticks(8);
        check(distance_mm == 250 && online && pulses == before_pulses,
            "queued old mean cannot overwrite a newer immediate near");
        reset_sensor(); frame(1000,0); first_stamp = expected_stamp;
        // Schedule checksum one system clock before the retained source expires.
        ticks((first_stamp+1000-tb_ms)*CYCLES_PER_MS-tb_div-9);
        frame(1002,0);
        check(distance_mm == 1000 && !online, "data timeout cancels a queued result before its commit");
        idle_ms(2); check(distance_mm == 1000 && !online, "cancelled timeout result never resurrects during silence");
        reset_sensor();
        @(negedge clk); dut.uptime_ms = 32'hffffffb0; dut.uptime_next_ms = 32'hffffffb1; tb_ms = 32'hffffffb0;
        frame(1000,0); idle_ms(100); frame(1002,0); check(distance_mm == 1001 && distance_age_ms >= 100,
            "uint32 uptime wrap preserves oldest contributor age and mean");
        @(negedge clk); dut.distance_stamp_ms = tb_ms-32'd65534; expected_stamp = tb_ms-32'd65534;
        idle_ms(3); check(distance_age_ms == 65535, "age saturates atFFFF rather than wrapping");
        reset_sensor();
        for (i = 0; i < 1200; i = i + 1) begin
            next_random(); gap_ms = rng % 151;
            if (i%47 == 0) gap_ms = 305;
            if (i%131 == 0) gap_ms = 1002;
            idle_ms(gap_ms);
            next_random();
            case (rng % 16)
                0: generated = 0;
                1: generated = 65535;
                2: generated = 1 + (rng % 29);
                3: generated = 4501 + (rng % 61033);
                4: generated = 30 + (rng % 321);
                5,6,7,8: generated = 980 + (rng % 41);
                default: generated = 351 + (rng % 4150);
            endcase
            frame(generated, (i%37 == 0)); random_frames = random_frames + 1;
            if (i%113 == 0) reset_sensor();
        end
        $display("PASS sonar MA v26: %0d checks, %0d frames (%0d seeded random), %0d publications; rounded recent4/warmup/300ms span, immediate near, confirmed variablefar, oldest-source age, clamp/invalid/CRC/partial/data timeout/reset/wrap",
            checks, frames, random_frames, accepted);
        $display("Pipeline contract: normal latency max%0d clocks (limit4); near immediate; %0d timeout cancellations",max_normal_latency,queued_cancellations);
        $display("Registered tick/shadow: %0d independent cycle-phase checks including uint32 wrap",clock_phase_checks);
        $finish;
    end
    initial begin #50000000; $fatal(1,"test timeout"); end
endmodule
