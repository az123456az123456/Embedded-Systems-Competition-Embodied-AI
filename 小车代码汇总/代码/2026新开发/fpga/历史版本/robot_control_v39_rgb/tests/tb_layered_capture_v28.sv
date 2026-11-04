`timescale 1ns/1ps
module tb_layered_capture_v28;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg [2:0] rx=7;reg [3:0] a=0,b=0;reg [47:0] filtered={16'd2900,16'd1900,16'd900};reg [95:0] fseq=0;
 reg [2:0] fv=7;reg ready=1;wire valid;wire [511:0] packet;wire [127:0] positions;wire [3:0] illegal;
 wire [47:0] raw;wire [95:0] seqs,stamps;wire [2:0] online;
 robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(10000),.ENCODER_FILTER_CYCLES(2),.CPU_FILTER_ENABLE(1)) dut(
 .clk(clk),.rst_n(rst_n),.encoder_a(a),.encoder_b(b),.sonar_rx(rx),.sensor_valid(valid),.sensor_packet(packet),
 .filter_mm(filtered),.filter_seq(fseq),.filter_valid(fv),.filter_ready(ready),.encoder_positions(positions),.encoder_illegal(illegal),
 .raw_sonar_mm(raw),.raw_sonar_seq(seqs),.raw_sonar_stamp(stamps),.raw_sonar_online(online));
 reg [511:0] latest=0;integer packets=0,j;reg [31:0] source0,source1;reg [95:0] heldstamp,heldseq;
 always @(negedge clk)if(valid)begin
 latest=packet;packets++;
 if(packet[511:480]!==32'h534e5331 || packet[479:472]!=1)$fatal(1,"SNS1 incompatible");
 if((packet[471:464]|packet[463:456])!=7 || (packet[471:464]&packet[463:456])!=0)$fatal(1,"rawfaultmaskwrong");
 end
 task automatic ticks(input integer n);begin repeat(n)@(negedge clk);#1;end endtask
 task automatic byte_in(input integer ch,input [7:0] v);integer bitno;begin
 @(negedge clk);rx[ch]=0;ticks(10);for(bitno=0;bitno<8;bitno++)begin rx[ch]=v[bitno];ticks(10);end rx[ch]=1;ticks(10);end endtask
 task automatic frame(input integer ch,input [15:0] mm,input bit bad);reg [7:0] crc;begin
 crc=8'hff+mm[15:8]+mm[7:0];byte_in(ch,255);byte_in(ch,mm[15:8]);byte_in(ch,mm[7:0]);byte_in(ch,crc+bad);ticks(110);end endtask
 task automatic check(input bit cond,input string why);begin if(!cond)$fatal(1,"%s raw%h seq%h SNS=%h",why,raw,seqs,latest);end endtask
 initial begin ticks(5);rst_n=1;ticks(10);
 fork frame(0,1000,0);frame(1,2000,0);frame(2,3000,0);join
 check(online==7 && seqs=={32'd1,32'd1,32'd1},"rawCRC event seq independent channels");
 check(latest[127:80]=={16'd1000,16'd2000,16'd3000} && latest[471:464]==7,"pendingCPUestimate rawfallback without invalidgap");
 heldstamp=stamps;heldseq=seqs;source0=latest[415:384]-latest[79:64];
 fseq=seqs;ticks(5100);check(latest[127:80]=={16'd900,16'd1900,16'd2900},"matchingKFoutputs published");
 check(seqs==heldseq && stamps==heldstamp,"commit mustnotrefreshraweventtime");
 check(latest[415:384]-latest[79:64]==source0,"KFcommit preserves true source timestamp");
 ready=0;ticks(5100);check(latest[127:112]==1000,"CPUstallrawfallback retainsvalid");ready=1;
 frame(0,1050,0);check(raw[15:0]==1050 && seqs[31:0]==2 && latest[127:112]==1050 && latest[464],"each newCRC rawfallback noMA/farhold");
 source1=latest[415:384]-latest[79:64];check(source1>source0,"newsourceadvances");
 fseq=seqs;filtered[15:0]=1025;ticks(5100);check(latest[127:112]==1025,"newmatchedKF");
 frame(0,200,0);check(latest[127:112]==200,"nearrawbypass laggingKF");
 fseq=seqs;filtered[15:0]=1000;ticks(5100);check(latest[127:112]==200,"nearrawbypass matchingKF");
 frame(0,4000,0);check(latest[127:112]==4000 && latest[464],"near-to-far source immediatelypublished noextraMA/confirmation");
 heldseq=seqs;heldstamp=stamps;frame(0,1234,1);check(seqs==heldseq && stamps==heldstamp,"badCRCcannotrefreshsource");
 frame(0,1,0);check(!online[0] && !latest[464] && latest[456],"invalidnumericimmediatefault nofakefar");
 check(seqs==heldseq && stamps==heldstamp,"invalidnumericcannotrefreshsource");
 frame(0,65535,0);check(!online[0],"FFFFinvalid");frame(0,4501,0);check(online[0] && raw[15:0]==4501 && latest[127:112]==4501,"numericfarremainsactualnumeric notfabricated4m");
 // Local absolute counters update before the next50ms SNS periodic event.
 a[0]=1;ticks(10);b[0]=1;ticks(10);a[0]=0;ticks(10);b[0]=0;ticks(10);
 check($signed(positions[31:0])==4 || $signed(positions[31:0])==-4,"directencoderpositionsnotSNSdelta");
 ticks(100200);check(online==0 && latest[471:464]==0 && latest[463:456]==7,"rawdisconnects expires independentCPUcommit");
 fseq=seqs;ticks(5100);check(latest[471:464]==0,"KFcannotresurrectexpiredraw");
 $display("PASS v28 actualthreeUART->rawCRCseq/timestamps->SNS1 KFmatch/rawfallback/nearpriority/invalid/CRC/expiration; directlocalencoder snapshot; %0dpackets",packets);$finish;end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
