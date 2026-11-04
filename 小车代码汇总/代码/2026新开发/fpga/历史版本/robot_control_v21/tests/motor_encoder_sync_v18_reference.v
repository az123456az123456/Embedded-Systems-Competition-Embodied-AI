// Frozen v18 behavioral reference, copied before the v20 timing-only rewrite.
// Original SHA256: 2598217a63605196f3fd7ddeff087e62a9ed7809ab2f682861262498b50f1e36
`timescale 1ns/1ps
// Relative wheel-speed feedback for four identical encoders and wheels.
// The existing joystick still sets the common feed-forward PWM. This loop
// balances four absolute encoder rates only during settled straight motion.
// It does not infer metres/second, heading or encoder polarity.
module motor_encoder_sync_v18_reference #(
    parameter integer CLK_HZ=50000000,
    parameter integer STALE_MS=300,
    parameter integer SETTLE_SAMPLES=4,
    parameter integer TRIM_LIMIT=100
)(
    input wire clk,rst_n,enable,reverse,
    input wire [9:0] base_duty,
    input wire sample_valid,
    input wire [127:0] deltas, // signed32, M1 in bits31:0
    input wire [3:0] illegal,
    output wire [47:0] trims, // signed12 per motor, permille
    output reg active,fault
);
    localparam integer STALE_CYCLES=(CLK_HZ/1000)*STALE_MS;
    localparam signed [11:0] ABS_LIMIT=TRIM_LIMIT[11:0];
    reg [31:0] age;
    reg [9:0] previous_base;
    reg previous_reverse;
    reg [7:0] settling;
    reg [1:0] samples;
    reg [1:0] pending;
    reg [17:0] saved_mean,saved_band;
    reg saved_usable;
    reg [17:0] accumulated[0:3];
    reg signed [11:0] trim[0:3];
    wire [31:0] absolute_delta[0:3];
    wire invalid_sample=(illegal!=0) || (|absolute_delta[0][31:16]) ||
        (|absolute_delta[1][31:16]) || (|absolute_delta[2][31:16]) || (|absolute_delta[3][31:16]);
    wire [19:0] total={2'b0,accumulated[0]}+{2'b0,accumulated[1]}+
        {2'b0,accumulated[2]}+{2'b0,accumulated[3]};
    wire [17:0] mean=total[19:2];
    wire [17:0] band=(mean>>6)<2 ? 18'd2 : (mean>>6);
    wire usable=accumulated[0]>=8 && accumulated[1]>=8 && accumulated[2]>=8 && accumulated[3]>=8;
    wire [9:0] relative_limit=base_duty>>2;
    wire signed [11:0] limit=({2'b0,relative_limit}<$unsigned(ABS_LIMIT)) ? $signed({2'b0,relative_limit}) : ABS_LIMIT;
    function signed [11:0] adjust;
        input signed [11:0] old_trim;
        input [17:0] count,average,deadband;
        input signed [11:0] bound;
        reg signed [19:0] error;
        reg signed [12:0] change,next_trim;
        begin
            error=$signed({2'b0,average})-$signed({2'b0,count});
            change=0;
            if(error>$signed({2'b0,deadband}))
                change=error>$signed({2'b0,(average>>2)}) ? 13'sd8 : 13'sd4;
            else if(error<-$signed({2'b0,deadband}))
                change=error<-$signed({2'b0,(average>>2)}) ? -13'sd8 : -13'sd4;
            next_trim=$signed({old_trim[11],old_trim})+change;
            if(next_trim>$signed({bound[11],bound})) adjust=bound;
            else if(next_trim<-$signed({bound[11],bound})) adjust=-bound;
            else adjust=next_trim[11:0];
        end
    endfunction
    genvar n;
    generate for(n=0;n<4;n=n+1)begin: outputs
        wire [31:0] raw=deltas[n*32 +:32];
        assign absolute_delta[n]=raw[31] ? (~raw+32'd1) : raw;
        assign trims[n*12 +:12]=trim[n];
    end endgenerate
    integer i;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            age<=0;previous_base<=0;previous_reverse<=0;settling<=0;
            samples<=0;pending<=0;active<=0;fault<=0;
            saved_mean<=0;saved_band<=0;saved_usable<=0;
            for(i=0;i<4;i=i+1)begin accumulated[i]<=0;trim[i]<=0;end
        end else begin
            previous_base<=base_duty;previous_reverse<=reverse;
            if(!enable || base_duty!=previous_base || reverse!=previous_reverse)begin
                age<=0;settling<=0;samples<=0;pending<=0;active<=0;fault<=0;
                for(i=0;i<4;i=i+1)begin accumulated[i]<=0;trim[i]<=0;end
            end else if((!sample_valid && age>=STALE_CYCLES-1) || (sample_valid && invalid_sample))begin
                age<=sample_valid ? 0 : age;settling<=0;samples<=0;pending<=0;active<=0;fault<=1;
                for(i=0;i<4;i=i+1)begin accumulated[i]<=0;trim[i]<=0;end
            end else begin
                if(sample_valid)age<=0;else age<=age+1'b1;
                // Separate four-wheel mean reduction from error/limit arithmetic.
                // Two extra 50MHz cycles do not change the 200ms control cadence.
                if(pending==1)begin
                    saved_mean<=mean;saved_band<=band;saved_usable<=usable;pending<=2;
                end else if(pending==2)begin
                    pending<=0;active<=saved_usable;fault<=!saved_usable;
                    for(i=0;i<4;i=i+1)begin
                        trim[i]<=saved_usable ? adjust(trim[i],accumulated[i],saved_mean,saved_band,limit) : 12'sd0;
                        accumulated[i]<=0;
                    end
                end else if(sample_valid)begin
                    if(settling<SETTLE_SAMPLES[7:0])settling<=settling+1'b1;
                    else begin
                        for(i=0;i<4;i=i+1)accumulated[i]<=accumulated[i]+{2'b0,absolute_delta[i][15:0]};
                        if(samples==3)begin samples<=0;pending<=1;end
                        else samples<=samples+1'b1;
                    end
                end
            end
        end
    end
endmodule
