`timescale 1ns/1ps
module tb_imu_filter;
    reg clk=0,rst_n=0,clear=0,valid=0;
    always #10 clk=~clk;
    reg signed [15:0] a=0,g=0;
    reg [15:0] temp=0;
    wire out_valid;
    wire signed [15:0] ax,ay,az,gx,gy,gz;
    wire [15:0] t;
    imu_sample_filter dut(.clk(clk),.rst_n(rst_n),.clear(clear),.in_valid(valid),
        .ax_in(a),.ay_in(a),.az_in(a),.temp_in(temp),.gx_in(g),.gy_in(g),.gz_in(g),
        .out_valid(out_valid),.ax_out(ax),.ay_out(ay),.az_out(az),.temp_out(t),.gx_out(gx),.gy_out(gy),.gz_out(gz));
    task push(input signed [15:0] av,gv);
        begin
            @(negedge clk);a=av;g=gv;temp=temp+1;valid=1;
            @(posedge clk);#1;
            if(!out_valid || t!==temp || ax!==ay || ax!==az || gx!==gy || gx!==gz)
                $fatal(1,"sample/temperature/valid not atomic");
            @(negedge clk);valid=0;
            @(posedge clk);#1;if(out_valid)$fatal(1,"duplicate output without sample");
        end
    endtask
    task restart;
        begin @(negedge clk);clear=1;@(negedge clk);clear=0;end
    endtask
    integer i,area,ng,na;
    reg signed [15:0] held_a,held_g;
    initial begin
        #41;rst_n=1;
        push(16384,-131);if(ax!==16384 || gx!==-131)$fatal(1,"first sample must seed exactly");
        held_a=ax;held_g=gx;
        repeat(25)begin @(posedge clk);#1;if(out_valid || ax!==held_a || gx!==held_g)$fatal(1,"idle changes sample");end
        restart;push(-32768,32767);if(ax!==-32768 || gx!==32767)$fatal(1,"signed limits seed");
        for(i=0;i<80;i=i+1)push(32767,-32768);
        if(ax!==32767 || gx!==-32768)$fatal(1,"signed limits converge %d %d",ax,gx);
        restart;push(0,0);
        for(i=0;i<80;i=i+1)push(1,-1);
        if(ax!==1 || gx!==-1)$fatal(1,"tiny constant DC swallowed");
        for(i=0;i<80;i=i+1)push(-1,1);
        if(ax!==-1 || gx!==1)$fatal(1,"tiny reversed DC swallowed");
        restart;push(1000,1000);push(2000,2000);
        if(ax!==1250 || gx!==1500)$fatal(1,"wrong filter coefficient %d %d",ax,gx);
        restart;push(1000,1000);ng=0;na=0;
        for(i=0;i<200;i=i+1)begin
            push(i[0]?1200:800,i[0]?1200:800);
            if(i>100)begin
                if(gx<930 || gx>1070 || ax<968 || ax>1032)$fatal(1,"noise not attenuated %d %d",ax,gx);
                if(gx<1000)ng=1000-gx;else ng=gx-1000;
                if(ax<1000)na=1000-ax;else na=ax-1000;
            end
        end
        $display("PASS alternating noise: raw amplitude 200 codes; filtered gyro %0d, accel %0d",ng,na);
        restart;push(0,0);area=0;
        for(i=0;i<100;i=i+1)begin push(0,1310);area=area+gz;end
        for(i=0;i<80;i=i+1)begin push(0,0);area=area+gz;end
        if(area<130984 || area>131016)$fatal(1,"positive turn area lost %d",area);
        $display("PASS +10deg turn: integrated filtered codes %0d; expected 131000",area);
        restart;push(0,0);area=0;
        for(i=0;i<100;i=i+1)begin push(0,-1310);area=area+gz;end
        for(i=0;i<80;i=i+1)begin push(0,0);area=area+gz;end
        if(area< -131016 || area> -130984)$fatal(1,"negative turn area lost %d",area);
        $display("PASS -10deg turn: integrated filtered codes %0d; expected -131000",area);
        restart;push(0,0);area=0;
        for(i=0;i<100;i=i+1)begin push(0,11790);area=area+gz;end
        for(i=0;i<80;i=i+1)begin push(0,0);area=area+gz;end
        if(area<1178984 || area>1179016)$fatal(1,"90deg turn area lost %d",area);
        $display("PASS 90deg turn: integrated filtered codes %0d; expected 1179000",area);
        restart;push(0,0);area=0;
        for(i=0;i<10000;i=i+1)begin push(0,13);area=area+gz;end
        for(i=0;i<80;i=i+1)begin push(0,0);area=area+gz;end
        if(area<129984 || area>130016)$fatal(1,"0.1deg/s slow turn area lost %d",area);
        $display("PASS ~0.1deg/s slow turn for 100s: integrated filtered codes %0d; expected 130000",area);
        // Recovery seeds a fresh value rather than blending pre-fault history.
        restart;push(2345,-2345);if(ax!==2345 || gx!==-2345)$fatal(1,"recovery stale history");
        $display("PASS signed extremes, unity DC including +/-1 code, atomic valid/temperature, sample-only update and reinit");
        $finish;
    end
endmodule
