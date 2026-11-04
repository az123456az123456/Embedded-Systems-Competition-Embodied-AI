`timescale 1ns/1ps
// Compare the new algebraic thresholds to the frozen v18 expression, then
// compare complete feedback outputs on every cycle (including pending/faults).
module tb_encoder_threshold_equivalence;
    reg clk=0,rst_n=0,enable=0,reverse=0,sample_valid=0;
    reg [9:0] base_duty=400;
    reg [127:0] deltas=0;
    reg [3:0] illegal=0;
    wire [47:0] trims,reference_trims;
    wire active,fault,reference_active,reference_fault;
    motor_encoder_sync #(.CLK_HZ(1000)) dut(.*);
    motor_encoder_sync_v18_reference #(.CLK_HZ(1000)) reference_dut(
        .clk(clk),.rst_n(rst_n),.enable(enable),.reverse(reverse),.sample_valid(sample_valid),
        .base_duty(base_duty),.deltas(deltas),.illegal(illegal),
        .trims(reference_trims),.active(reference_active),.fault(reference_fault));
    always #5 clk=~clk;
    function signed [11:0] original_adjust;
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
            if(next_trim>$signed({bound[11],bound})) original_adjust=bound;
            else if(next_trim<-$signed({bound[11],bound})) original_adjust=-bound;
            else original_adjust=next_trim[11:0];
        end
    endfunction
    integer vectors=0,cycles=0,updates=0,fault_cycles=0;
    task compare_adjust(input [17:0] average,deadband,count,
                        input signed [11:0] old_trim,bound);
        reg signed [19:0] band_low,band_high,quarter_low,quarter_high;
        reg signed [11:0] expected,actual;
        begin
            band_low=$signed({2'b0,average})-$signed({2'b0,deadband});
            band_high=$signed({2'b0,average})+$signed({2'b0,deadband});
            quarter_low=$signed({2'b0,average})-$signed({2'b0,(average>>2)});
            quarter_high=$signed({2'b0,average})+$signed({2'b0,(average>>2)});
            expected=original_adjust(old_trim,count,average,deadband,bound);
            actual=dut.adjust(old_trim,count,band_low,band_high,quarter_low,quarter_high,bound);
            if(actual!==expected)$fatal(1,"threshold mismatch avg=%0d band=%0d count=%0d old=%0d bound=%0d got=%0d expected=%0d",
                average,deadband,count,old_trim,bound,actual,expected);
            vectors=vectors+1;
        end
    endtask
    reg [31:0] random_state=32'h691ad44b;
    function [31:0] step(input [31:0] old);
        begin step={old[30:0],old[31]^old[21]^old[1]^old[0]};end
    endfunction
    integer averages[0:20],bands[0:9],counts[0:15],olds[0:10],bounds[0:10];
    integer a,b,c,o,l,k,n,value,mean_value,band_value;
    reg [17:0] random_average,random_band,random_count;
    reg signed [11:0] random_old,random_bound;
    reg [47:0] previous_trims=0;
    always @(negedge clk)if(rst_n)begin
        if({trims,active,fault}!=={reference_trims,reference_active,reference_fault})
            $fatal(1,"feedback changed at cycle%0d pending=%0d: got%h/%b/%b expected%h/%b/%b",
                cycles,dut.pending,trims,active,fault,reference_trims,reference_active,reference_fault);
        if(trims!=previous_trims)updates=updates+1;
        if(fault)fault_cycles=fault_cycles+1;
        previous_trims=trims;
        cycles=cycles+1;
    end
    initial begin
        averages='{0,1,2,3,4,7,8,63,64,65,127,128,129,255,256,1023,65535,65536,131071,262142,262143};
        olds='{-2048,-101,-100,-8,-1,0,1,8,100,101,2047};
        bounds='{0,1,4,8,50,100,255,1023,2047,-1,-2048};
        for(a=0;a<21;a=a+1)begin
            mean_value=averages[a];
            bands='{0,1,2,3,4,8,(mean_value>>6),(mean_value>>2),mean_value,262143};
            for(b=0;b<10;b=b+1)begin
                band_value=bands[b];
                counts='{0,1,2,262143,mean_value,
                    (mean_value-band_value-1),(mean_value-band_value),(mean_value-band_value+1),
                    (mean_value+band_value-1),(mean_value+band_value),(mean_value+band_value+1),
                    (mean_value-(mean_value>>2)-1),(mean_value-(mean_value>>2)),
                    (mean_value+(mean_value>>2)),(mean_value+(mean_value>>2)+1),131072};
                for(c=0;c<16;c=c+1)if(counts[c]>=0 && counts[c]<=262143)
                    for(o=0;o<11;o=o+1)for(l=0;l<11;l=l+1)
                        compare_adjust(mean_value,band_value,counts[c],olds[o],bounds[l]);
            end
        end
        for(k=0;k<200000;k=k+1)begin
            random_state=step(random_state);random_average=random_state[17:0];
            random_state=step(random_state);random_band=random_state[17:0];
            random_state=step(random_state);random_count=random_state[17:0];
            random_state=step(random_state);random_old=random_state[11:0];
            random_state=step(random_state);random_bound=random_state[11:0];
            compare_adjust(random_average,random_band,random_count,random_old,random_bound);
        end
        repeat(3)@(negedge clk);rst_n=1;enable=1;
        for(k=0;k<150000;k=k+1)begin
            @(negedge clk);#1;
            sample_valid=0;illegal=0;
            if(k%3000==0)base_duty=200+(k/3000)%800;
            if(k%9000==0)reverse=~reverse;
            enable=k%7000>=3;
            if(k%50==0 && !(k%10000>=5000 && k%10000<5400))begin
                sample_valid=1;
                for(n=0;n<4;n=n+1)begin
                    random_state=step(random_state);value=8+random_state%800;
                    if(random_state[20])value=-value;
                    deltas[n*32+:32]=value;
                end
            end
            // Deliberate burst samples in the two pending stages preserve the
            // original input priority, including invalid/stale resets.
            if(k%1111==0)begin sample_valid=1;illegal=4'b0100;end
            if(k%2003==0)begin sample_valid=1;deltas[31:0]=32'h80000000;end
            if(k%2017==0)begin sample_valid=1;deltas[63:32]=32'd65536;end
            if(k%3011==0)begin sample_valid=1;deltas[95:64]=0;end
            if(k%4001==0)begin sample_valid=1;deltas={4{32'd65535}};end
        end
        @(negedge clk);#1;
        if(updates<100 || fault_cycles<1000)$fatal(1,"insufficient sequential coverage updates=%0d faults=%0d",updates,fault_cycles);
        $display("PASS encoder threshold equivalence: %0d boundary/random adjust vectors; %0d cycle-identical outputs, %0d trim changes, %0d fault cycles",vectors,cycles,updates,fault_cycles);
        $finish;
    end
    initial begin #2000000;$fatal(1,"timeout");end
endmodule
