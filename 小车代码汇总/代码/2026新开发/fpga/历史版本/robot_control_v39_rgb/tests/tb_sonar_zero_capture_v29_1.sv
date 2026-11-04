`timescale 1ns/1ps
module tb_sonar_zero_capture_v29_1;
 reg clk=0,rst_n=0;always #5 clk=~clk;
 reg[2:0]rx=7;reg[47:0]filtered=0;reg[95:0]fseq=0;reg[2:0]fv=7;reg use_c=0;
 wire valid;wire[511:0]packet;wire[47:0]raw;wire[95:0]seqs,stamps;wire[2:0]online;
 robot_sensor_capture #(.CLK_FREQ(100000),.BAUD(10000),.ENCODER_FILTER_CYCLES(2),.CPU_FILTER_ENABLE(1)) dut(
 .clk(clk),.rst_n(rst_n),.encoder_a(4'd0),.encoder_b(4'd0),.sonar_rx(rx),.sensor_valid(valid),.sensor_packet(packet),
 .filter_mm(filtered),.filter_seq(fseq),.filter_valid(fv),.filter_ready(1'b1),.encoder_positions(),.encoder_illegal(),
 .raw_sonar_mm(raw),.raw_sonar_seq(seqs),.raw_sonar_stamp(stamps),.raw_sonar_online(online));
 import "DPI-C" function void kf_step(input int now_ms,input int mm0,mm1,mm2,input int seq0,seq1,seq2,input int stamp0,stamp1,stamp2,input int mask,output int f0,f1,f2,output int fs0,fs1,fs2,output int fmask);
 integer cycles=0,packets=0,checks=0,calls=0,f0,f1,f2,fs0,fs1,fs2,fmask;
 reg[511:0]latest=0;reg[95:0]heldseq,heldstamp;integer before_packets;
 always @(negedge clk)if(rst_n)begin
 cycles++;
 if(use_c && cycles%200==0)begin
 kf_step(dut.uptime_ms,raw[15:0],raw[31:16],raw[47:32],seqs[31:0],seqs[63:32],seqs[95:64],stamps[31:0],stamps[63:32],stamps[95:64],online,f0,f1,f2,fs0,fs1,fs2,fmask);
 filtered={f2[15:0],f1[15:0],f0[15:0]};fseq={fs2[31:0],fs1[31:0],fs0[31:0]};fv=fmask[2:0];calls++;
 end
 if(valid)begin latest=packet;packets++;
 $display("SNS1 t_ms=%0d packet=%h",dut.uptime_ms,packet);
 if(packet[511:480]!=32'h534e5331 || packet[479:472]!=1)$fatal(1,"SNS1 format");
 if(packet[471:464]!=online || (packet[471:464]|packet[463:456])!=7)$fatal(1,"masks");end
 end
 task ticks(input integer n);begin repeat(n)@(negedge clk);#1;end endtask
 task byte_in(input integer ch,input[7:0]v);begin @(negedge clk);rx[ch]=0;ticks(10);for(integer b=0;b<8;b++)begin rx[ch]=v[b];ticks(10);end rx[ch]=1;ticks(10);end endtask
 task frame(input integer ch,input[15:0]v,input[7:0]cs);begin byte_in(ch,255);byte_in(ch,v[15:8]);byte_in(ch,v[7:0]);byte_in(ch,cs);ticks(110);end endtask
 task sample(input integer ch,input[15:0]v);begin frame(ch,v,8'hff+v[15:8]+v[7:0]);end endtask
 task check(input bit cond,input string why);begin checks++;if(!cond)$fatal(1,"%s raw%h seq%h filter%h valid%b SNS%h",why,raw,seqs,filtered,fv,latest);end endtask
 initial begin ticks(5);rst_n=1;ticks(10);
 fork sample(0,0);sample(1,0);sample(2,0);join
 check(raw=={3{16'd4500}} && online==7 && seqs=={3{32'd1}},"startupzero actual3UART source events");
 check(latest[127:80]=={3{16'd4500}} && latest[471:464]==7,"startup SNS4500 rawfallback");
 fseq=seqs;filtered={3{16'd100}};ticks(110);check(latest[127:80]=={3{16'd4500}},"matchingKF near values cannot overridezero");
 heldseq=seqs;heldstamp=stamps;filtered={3{16'd4490}};ticks(5100);check(latest[127:80]=={3{16'd4500}},"matchingKF4490 cannot turnzero into intermediate");
 check(seqs==heldseq && stamps==heldstamp,"CPU commit notsourceevent");
 use_c=1;ticks(600);check(fseq==seqs && fv==7 && filtered=={3{16'd4500}},"actualC initialKFs");
 sample(0,200);check(latest[127:112]==200 && raw[15:0]==200,"actual near immediately visible");
 sample(0,0);ticks(400);check(latest[127:112]==4500 && raw[15:0]==4500,"actualC firstfar pending rawfallback");check(!fv[0],"actualC firstfar estimate explicitly unavailable");
 sample(0,0);ticks(400);check(fv[0] && filtered[15:0]==4500 && latest[127:112]==4500,"secondzero actualKFconfirmed");
 sample(0,4490);ticks(400);check(fv[0] && filtered[15:0]>4490 && filtered[15:0]<4500,"actualC smooths realnear-maxnumeric");
 sample(0,0);ticks(400);check(fv[0] && filtered[15:0]<4500 && fseq==seqs && latest[127:112]==4500,"actualC matched intermediate cannot replacezero4500");
 sample(0,180);check(latest[127:112]==180,"newnear bypass matchedfarC");
 heldseq=seqs;heldstamp=stamps;frame(0,1,255);check(seqs==heldseq && stamps==heldstamp,"bad checksumFF cannot update source");
 sample(0,0);check(seqs[31:0]==heldseq[31:0]+1 && latest[127:112]==4500,"nextzero recovers checksumFF exactonce");
 heldseq=seqs;heldstamp=stamps;frame(0,0,254);check(seqs==heldseq && stamps==heldstamp,"corruptzero nofresh source");
 sample(0,0);check(seqs[31:0]==heldseq[31:0]+1,"nextzero recovers checksum nonFF");
 heldseq=seqs;heldstamp=stamps;sample(0,1);check(!online[0] && seqs==heldseq && stamps==heldstamp,"1 invalid unchanged source");
 sample(0,0);sample(0,29);check(!online[0],"29 invalid");sample(0,0);sample(0,65535);check(!online[0],"FFFF invalid");
 sample(0,0);heldseq=seqs;heldstamp=stamps;before_packets=packets;ticks(100200);
 check(online==0 && latest[471:464]==0 && latest[463:456]==7,"UART silence realexpiry even ongoingSNS/Ccommits");
 check(seqs==heldseq && stamps==heldstamp && packets>before_packets+15,"SNS packet freshness cannot rejuvenate source");
 check(latest[79:64]>=1000 && fv==0,"C/source true ages expire");
 $display("PASS v29.1 actualthreeUART->SNS1 sourceage/seq -> productionC Kalman -> actualcapturepublication zero4500/noextraWAIT;matchingnear/4490;near-zero-near;badchecksumFF/nonFF;numericinvalid/silence checks=%0d calls=%0d packets=%0d",checks,calls,packets);$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
