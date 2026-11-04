`timescale 1ns/1ps
module tb_sensor_capture;
    reg clk=0;always #5 clk=~clk;
    reg rst_n=0;
    reg [3:0] ea=4'b1000,eb=4'b1000;
    reg [2:0] rx=3'b111;
    wire valid;wire [511:0] packet;
    robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(10000)) dut(
        .clk(clk),.rst_n(rst_n),.encoder_a(ea),.encoder_b(eb),.sonar_rx(rx),
        .sensor_valid(valid),.sensor_packet(packet));
    integer report_count=0;reg [511:0] latest;
    reg boundary_seen=0;
    always @(posedge clk)if(rst_n && dut.tick_ms && dut.report_ms==49 &&
        dut.encoders.wheel[0].stable==15 && dut.encoders.wheel[0].candidate==2'b10 &&
        dut.encoders.wheel[0].previous==0)boundary_seen=1;
    reg [31:0] boundary_delta;
    always @(negedge clk)if(valid && packet[31:16]==50)begin report_count=report_count+1;latest=packet;end
    task automatic wait_cycles;input integer n;repeat(n)@(negedge clk);endtask
    task automatic step;input integer ch;input [1:0] ab;
        begin @(negedge clk);ea[ch]=ab[1];eb[ch]=ab[0];wait_cycles(24);end
    endtask
    task automatic positive;input integer ch;begin step(ch,2'b10);step(ch,2'b11);step(ch,2'b01);step(ch,2'b00);end endtask
    task automatic negative;input integer ch;begin step(ch,2'b01);step(ch,2'b11);step(ch,2'b10);step(ch,2'b00);end endtask
    task automatic uart_byte;input integer ch;input [7:0] value;integer j;
        begin
            @(negedge clk);rx[ch]=0;wait_cycles(10);
            for(j=0;j<8;j=j+1)begin rx[ch]=value[j];wait_cycles(10);end
            rx[ch]=1;wait_cycles(10);
        end
    endtask
    task automatic frame;input integer ch;input [15:0] mm;input bad;
        reg [7:0] sum;
        begin sum=8'hff+mm[15:8]+mm[7:0];uart_byte(ch,8'hff);uart_byte(ch,mm[15:8]);uart_byte(ch,mm[7:0]);uart_byte(ch,sum+bad);end
    endtask
    task automatic next_report;
        integer previous;
        begin previous=report_count;while(report_count==previous)@(negedge clk);end
    endtask
    initial begin
        wait_cycles(5);rst_n=1;wait_cycles(40);
        if(dut.positions!==0)$fatal(1,"initial stable levels produced phantom counts");
        // Short glitches must not qualify as state changes.
        ea[0]=1;wait_cycles(3);ea[0]=0;wait_cycles(30);
        if(dut.positions!==0)$fatal(1,"short pulse escaped filter");
        positive(0);positive(1);positive(1);negative(2);
        fork frame(0,16'd1234,0);frame(1,16'd2000,0);frame(2,16'd4500,0);join
        next_report();
        if(latest[511:480]!==32'h534e5331||latest[479:472]!==8'd1)$fatal(1,"SNS1 header");
        if(latest[471:464]!==8'd7||latest[463:456]!==8'd0)$fatal(1,"three independent UARTs not valid");
        if(latest[383:352]!==32'd4||latest[351:320]!==32'd8||latest[319:288]!==-32'd4||latest[287:256]!==0)$fatal(1,"wheel positions/wire order");
        if(latest[255:224]!==32'd4||latest[223:192]!==32'd8||latest[191:160]!==-32'd4||latest[159:128]!==0)$fatal(1,"50ms delta counts");
        if(latest[127:112]!==16'd1234||latest[111:96]!==16'd2000||latest[95:80]!==16'd4500)$fatal(1,"sonar byte order");
        if(latest[31:16]!==16'd50||latest[7:0]!==0)$fatal(1,"window/reserved fields");
        next_report();if(latest[255:128]!==0)$fatal(1,"stationary delta is not zero");
        negative(0);next_report();
        if(latest[383:352]!==0||latest[255:224]!==-32'd4)$fatal(1,"signed reverse delta");
        // Simultaneous A/B jump is diagnosed, not converted into fake motion.
        step(0,2'b11);next_report();
        if(latest[383:352]!==0||latest[15:8]!==8'd1)$fatal(1,"illegal transition diagnosis");
        frame(0,16'd999,1);next_report();
        if(latest[127:112]!==16'd1234)$fatal(1,"bad checksum refreshed range");
        frame(1,16'd0,0);next_report();
        if(latest[465]!==0||latest[457]!==1)$fatal(1,"out-of-range did not invalidate left only");
        wait_cycles(100100);next_report();
        if(latest[471:464]!==0||latest[463:456]!==7)$fatal(1,"stale sonar not faulted");
        fork frame(0,16'd30,0);frame(1,16'd1024,0);frame(2,16'd2048,0);join
        next_report();if(latest[471:464]!==7||latest[463:456]!==0)$fatal(1,"sensor recovery");
        // Align a genuinely filtered A edge with the exact report clock.
        // Both count and delta belong to one coherent window, with no lost edge.
        rst_n=0;ea=0;eb=0;rx=7;wait_cycles(10);rst_n=1;
        while(!(dut.report_ms==49 && dut.ms_div==81))@(negedge clk);
        ea[0]=1;
        next_report();boundary_delta=latest[255:224];
        next_report();boundary_delta=boundary_delta+latest[255:224];
        if(!boundary_seen||latest[383:352]!==1||boundary_delta!==1)$fatal(1,"edge at report boundary lost/duplicated");
        $display("PASS exact report-boundary encoder edge counted once across adjacent windows");
        $display("PASS SNS1 capture: 4x counts, signed deltas, no startup/glitch counts, invalid transitions, concurrent A02 UARTs, checksum/range/expiry/recovery, field ordering");
        $finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
