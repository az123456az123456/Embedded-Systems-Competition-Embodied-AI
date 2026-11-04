`timescale 1ns/1ps
// A02YYUW decoder with a bounded sliding mean. A published mean is a filtered
// estimate, not a raw point observation: its source age conservatively follows
// the OLDEST contributing frame. A sharp near change bypasses the mean; a sharp
// far change starts a new scene and needs two consecutive valid far frames.
// A checksum-valid zero frame is the configured no-echo observation4500mm;
// it publishes immediately, clears MA history, and uses its real arrival age.
module ultrasonic_sensor #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer FRAME_TIMEOUT_MS = 20,
    parameter integer DATA_TIMEOUT_MS = 1000,
    parameter integer MIN_MM = 30,
    parameter integer MAX_MM = 4500,
    parameter integer STOP_MM = 300,
    parameter integer SLOW_MM = 800,
    parameter integer OBSTACLE_MM = 1200,
    parameter integer HYST_MM = 50,
    // Kept for source compatibility; no adjacent-far-distance gate is used.
    parameter integer FAR_STEP_MM = 120,
    parameter integer FAR_CONFIRM_MM = 200,
    // Supported window sizes are 1, 2 and 4; other settings select 4.
    parameter integer RAW_MODE = 0,
    parameter integer MA_WINDOW = 4,
    parameter integer MA_MAX_SPAN_MS = 300,
    parameter integer FAST_NEAR_DELTA_MM = 50,
    parameter integer FAST_NEAR_MM = 350
)(
    input wire clk, rst_n,
    input wire [7:0] rx_byte,
    input wire rx_valid,
    output reg [15:0] distance_mm,
    output reg distance_valid,
    output wire [15:0] distance_age_ms,
    output reg online,
    output wire sensor_fault,
    output reg obstacle,
    output reg slow_down,
    output reg stop_req
);
    localparam integer FRAME_TICKS = (CLK_FREQ / 1000) * FRAME_TIMEOUT_MS;
    localparam integer MS_CYCLES = CLK_FREQ / 1000;
    localparam integer MS_BITS = MS_CYCLES > 1 ? $clog2(MS_CYCLES) : 1;
    localparam integer MS_PRE_TICK = MS_CYCLES - 2;
    localparam [2:0] WINDOW_SIZE = MA_WINDOW == 1 ? 3'd1 : MA_WINDOW == 2 ? 3'd2 : 3'd4;
    reg [MS_BITS-1:0] ms_div;
    reg tick_ms;
    reg [31:0] uptime_ms, uptime_next_ms, distance_stamp_ms, candidate_stamp_ms;
    reg [31:0] last_numeric_stamp_ms;
    reg [1:0] state;
    reg [7:0] hi, lo;
    reg far_pending, have_distance;
    reg [31:0] frame_age;
    reg [2:0] ma_count;
    // Entries are chronological, oldest first. A count of zero invalidates all
    // entries, so an error cannot resurrect a previous scene's contributors.
    reg [15:0] ma_mm [0:3];
    reg [31:0] ma_stamp_ms [0:3];
    reg [15:0] next_mm [0:3];
    reg [31:0] next_stamp_ms [0:3];
    reg [2:0] next_count, retained_count;
    // Normal estimates publish four clocks after CRC acceptance. Emergency
    // near readings bypass every stage and publish in the CRC acceptance cycle.
    reg sum_pending, upper_pending, lower_pending, commit_pending;
    reg [17:0] sample_sum;
    reg [2:0] sum_count, divide_count;
    reg [31:0] sum_stamp_ms, divide_stamp_ms, result_stamp_ms;
    reg [8:0] div3_upper_q;
    reg [10:0] div3_lower_n;
    reg [15:0] simple_mean_mm, result_mm;
    integer j, k;

    // Predecode the terminal clock and keep uptime+1 in a shadow register. The
    // current-frame timestamp stays exact at a millisecond boundary without a
    // late tick decode feeding a 32-bit increment before every age comparison.
    wire [31:0] current_ms = tick_ms ? uptime_next_ms : uptime_ms;
    // Unsigned differences also cover a wrapping 32-bit millisecond counter.
    // Configured timeouts/spans must remain below half the counter range.
    wire [31:0] source_age_ms = uptime_ms - distance_stamp_ms;
    wire [31:0] candidate_age_ms = current_ms - candidate_stamp_ms;
    assign distance_age_ms = !have_distance || source_age_ms >= 32'd65535 ?
        16'hffff : source_age_ms[15:0];
    wire [7:0] checksum = 8'hff + hi + lo;
    wire [15:0] received_mm = {hi,lo};
    wire zero_no_echo = received_mm == 16'd0;
    wire usable_numeric = zero_no_echo ||
        ({16'd0,received_mm} >= MIN_MM && received_mm != 16'hffff);
    wire [15:0] raw_mm = zero_no_echo ? 16'd4500 : RAW_MODE ? received_mm :
        {16'd0,received_mm} > MAX_MM ? MAX_MM[15:0] : received_mm;
    wire fast_near = RAW_MODE || {16'd0,raw_mm} <= FAST_NEAR_MM ||
        (have_distance && raw_mm <= distance_mm &&
         ({16'd0,distance_mm} - {16'd0,raw_mm}) >= FAST_NEAR_DELTA_MM);
    wire immediate_observation = fast_near || zero_no_echo;
    wire sudden_far = have_distance && raw_mm > distance_mm &&
        ({16'd0,raw_mm} - {16'd0,distance_mm}) > FAR_CONFIRM_MM;
    wire range_expiring = online && have_distance &&
        current_ms - distance_stamp_ms > DATA_TIMEOUT_MS;
    wire history_gap = ma_count != 0 &&
        current_ms - last_numeric_stamp_ms > DATA_TIMEOUT_MS;
    // Ageout in the same cycle as a frame must break the old confirmation pair.
    wire previous_ready = far_pending && candidate_age_ms < DATA_TIMEOUT_MS &&
        !range_expiring && !history_gap;
    wire restart_window = !have_distance || immediate_observation || range_expiring ||
        history_gap || (sudden_far && !previous_ready) ||
        (far_pending && !sudden_far);
    wire publish_observation = immediate_observation || !sudden_far || previous_ready;
    wire frame_timed_out = state != 0 && frame_age >= FRAME_TICKS - 1;
    wire checksum_complete = rx_valid && state == 3 && !frame_timed_out;
    // Cancellation has priority over queued publication, including an error or
    // fast-near CRC arriving exactly when an older estimate would commit.
    wire cancel_estimate = range_expiring || history_gap || frame_timed_out ||
        (checksum_complete && (rx_byte != checksum || !usable_numeric || immediate_observation));
    // Split the count-three constant division across two bounded stages. For
    // N = upper*512 + lower, divide upper by 3 first, then carry its remainder
    // into the lower division. No variable divisor or timing exception is used.
    wire [17:0] rounded_div3 = sample_sum + 18'd1;
    wire [8:0] upper_div3_q = rounded_div3[17:9] / 9'd3;
    wire [8:0] upper_div3_rem = rounded_div3[17:9] - upper_div3_q * 9'd3;
    wire [10:0] lower_div3_q = div3_lower_n / 11'd3;
    wire [17:0] divided_mean = {div3_upper_q,lower_div3_q[8:0]};
    wire [17:0] rounded_div2 = (sample_sum + 18'd1) >> 1;
    wire [17:0] rounded_div4 = (sample_sum + 18'd2) >> 2;
    assign sensor_fault = !online;

    // Compact at most four recent contributors, then register them at CRC.
    // The warm-up mean uses the real count; no zero-filled entries are averaged.
    always @* begin
        for (j = 0; j < 4; j = j + 1) begin
            next_mm[j] = 0;
            next_stamp_ms[j] = 0;
        end
        retained_count = 0;
        if (!restart_window) begin
            for (j = 0; j < 4; j = j + 1) begin
                if (j < ma_count &&
                    current_ms - ma_stamp_ms[j] <= MA_MAX_SPAN_MS) begin
                    next_mm[retained_count[1:0]] = ma_mm[j];
                    next_stamp_ms[retained_count[1:0]] = ma_stamp_ms[j];
                    retained_count = retained_count + 1'b1;
                end
            end
        end
        if (retained_count >= WINDOW_SIZE) begin
            for (j = 0; j < 3; j = j + 1) begin
                next_mm[j] = next_mm[j+1];
                next_stamp_ms[j] = next_stamp_ms[j+1];
            end
            retained_count = WINDOW_SIZE - 3'd1;
        end
        next_mm[retained_count[1:0]] = raw_mm;
        next_stamp_ms[retained_count[1:0]] = current_ms;
        next_count = retained_count + 1'b1;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            ms_div <= 0; tick_ms <= MS_CYCLES <= 1;
            uptime_ms <= 0; uptime_next_ms <= 1; distance_stamp_ms <= 0;
            candidate_stamp_ms <= 0; last_numeric_stamp_ms <= 0;
            state <= 0; hi <= 0; lo <= 0; far_pending <= 0; have_distance <= 0;
            ma_count <= 0;
            sum_pending <= 0; upper_pending <= 0; lower_pending <= 0; commit_pending <= 0;
            sample_sum <= 0; sum_count <= 0; divide_count <= 0;
            sum_stamp_ms <= 0; divide_stamp_ms <= 0; result_stamp_ms <= 0;
            div3_upper_q <= 0; div3_lower_n <= 0; simple_mean_mm <= 0; result_mm <= 0;
            for (k = 0; k < 4; k = k + 1) begin
                ma_mm[k] <= 0; ma_stamp_ms[k] <= 0;
            end
            frame_age <= 0; distance_mm <= 0; distance_valid <= 0; online <= 0;
            obstacle <= 1; slow_down <= 1; stop_req <= 1;
        end else begin
            distance_valid <= 0;
            sum_pending <= 0; upper_pending <= 0; lower_pending <= 0; commit_pending <= 0;
            if (!cancel_estimate) begin
                if (sum_pending) begin
                    sample_sum <= {2'b0,ma_mm[0]} + {2'b0,ma_mm[1]} +
                                  {2'b0,ma_mm[2]} + {2'b0,ma_mm[3]};
                    sum_count <= ma_count; sum_stamp_ms <= ma_stamp_ms[0];
                    upper_pending <= 1;
                end
                if (upper_pending) begin
                    div3_upper_q <= upper_div3_q;
                    div3_lower_n <= {upper_div3_rem[1:0],rounded_div3[8:0]};
                    case (sum_count)
                        1: simple_mean_mm <= sample_sum[15:0];
                        2: simple_mean_mm <= rounded_div2[15:0];
                        default: simple_mean_mm <= rounded_div4[15:0];
                    endcase
                    divide_count <= sum_count; divide_stamp_ms <= sum_stamp_ms;
                    lower_pending <= 1;
                end
                if (lower_pending) begin
                    result_mm <= divide_count == 3 ? divided_mean[15:0] : simple_mean_mm;
                    result_stamp_ms <= divide_stamp_ms;
                    commit_pending <= 1;
                end
                if (commit_pending) begin
                    distance_mm <= result_mm; distance_valid <= 1;
                    have_distance <= 1; distance_stamp_ms <= result_stamp_ms;
                    online <= 1;
                    if ({16'd0,result_mm} <= OBSTACLE_MM) obstacle <= 1;
                    else if ({16'd0,result_mm} >= OBSTACLE_MM + HYST_MM) obstacle <= 0;
                    if ({16'd0,result_mm} <= SLOW_MM) slow_down <= 1;
                    else if ({16'd0,result_mm} >= SLOW_MM + HYST_MM) slow_down <= 0;
                    if ({16'd0,result_mm} <= STOP_MM) stop_req <= 1;
                    else if ({16'd0,result_mm} >= STOP_MM + HYST_MM) stop_req <= 0;
                end
            end
            tick_ms <= MS_CYCLES <= 1 || ms_div == MS_PRE_TICK[MS_BITS-1:0];
            if (tick_ms) begin
                ms_div <= 0; uptime_ms <= uptime_next_ms;
                uptime_next_ms <= uptime_next_ms + 32'd1;
            end
            else ms_div <= ms_div+1'b1;
            if (far_pending && candidate_age_ms >= DATA_TIMEOUT_MS) begin
                far_pending <= 0; ma_count <= 0;
            end
            if (history_gap) begin ma_count <= 0; far_pending <= 0; end
            if (range_expiring) begin
                // Clear once on the online->offline transition. Retain the
                // published baseline so recovery to a far scene still needs
                // two NEW frames, even if the source emits only every 500 ms.
                far_pending <= 0; ma_count <= 0;
                online <= 0; obstacle <= 1; slow_down <= 1; stop_req <= 1;
            end
            if (state == 0) frame_age <= 0;
            else if (frame_age >= FRAME_TICKS - 1) begin
                state <= 0; frame_age <= 0; far_pending <= 0; ma_count <= 0;
            end else frame_age <= frame_age + 1;

            if (rx_valid) begin
                frame_age <= 0;
                if (state != 0 && frame_age >= FRAME_TICKS - 1) begin
                    state <= (rx_byte == 8'hff) ? 1 : 0;
                end else case (state)
                    0: if (rx_byte == 8'hff) state <= 1;
                    1: begin hi <= rx_byte; state <= 2; end
                    2: begin lo <= rx_byte; state <= 3; end
                    3: begin
                        state <= 0;
                        if (rx_byte == checksum) begin
                            if (usable_numeric) begin
                                // Never mix a no-echo marker with numeric MA contributors.
                                ma_count <= zero_no_echo ? 3'd0 : next_count;
                                last_numeric_stamp_ms <= current_ms;
                                for (k = 0; k < 4; k = k + 1) begin
                                    ma_mm[k] <= zero_no_echo ? 16'd0 : next_mm[k];
                                    ma_stamp_ms[k] <= zero_no_echo ? 32'd0 : next_stamp_ms[k];
                                end
                                if (immediate_observation) begin
                                    distance_mm <= raw_mm; distance_valid <= 1;
                                    have_distance <= 1;
                                    distance_stamp_ms <= current_ms;
                                    online <= 1;
                                    if ({16'd0,raw_mm} <= OBSTACLE_MM) obstacle <= 1;
                                    else if ({16'd0,raw_mm} >= OBSTACLE_MM + HYST_MM) obstacle <= 0;
                                    if ({16'd0,raw_mm} <= SLOW_MM) slow_down <= 1;
                                    else if ({16'd0,raw_mm} >= SLOW_MM + HYST_MM) slow_down <= 0;
                                    if ({16'd0,raw_mm} <= STOP_MM) stop_req <= 1;
                                    else if ({16'd0,raw_mm} >= STOP_MM + HYST_MM) stop_req <= 0;
                                end else if (publish_observation) sum_pending <= 1;
                                // Confirmation uses the held published baseline,
                                // not adjacency between far readings. The second
                                // confirms even when the first contributor has
                                // exceeded MA_MAX_SPAN_MS and was evicted above.
                                far_pending <= sudden_far && !immediate_observation && !previous_ready;
                                if (sudden_far && !immediate_observation && !previous_ready)
                                    candidate_stamp_ms <= current_ms;
                            end else begin
                                online <= 0; far_pending <= 0; ma_count <= 0;
                                obstacle <= 1; slow_down <= 1; stop_req <= 1;
                            end
                        end else begin
                            far_pending <= 0; ma_count <= 0;
                            // A bad checksum byte belongs to the rejected frame.
                            // Reusing FF here can permanently misalign FF0000FF.
                            // state remains0; only the NEXT byte may open a frame.
                        end
                    end
                endcase
            end
        end
    end
endmodule
