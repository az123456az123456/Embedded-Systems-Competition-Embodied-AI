`timescale 1ns/1ps
module tb_sonar_zero_uart_v29_1;
 reg clk=0,rst_n=0,rx=1;always #5 clk=~clk;
 wire[7:0]byte_data;wire byte_valid;
 a02_uart_rx #(.CLK_FREQ(100000),.BAUD(10000)) uart(.clk(clk),.rst_n(rst_n),.rx(rx),.data(byte_data),.valid(byte_valid));
 wire[15:0]mm[0:3],age[0:3];wire[3:0]valid,online;integer count[0:3];integer cycle=0,last_valid[0:3],i,j,checks=0;
 genvar g;generate for(g=0;g<4;g++)begin:mode
 ultrasonic_sensor #(.CLK_FREQ(100000),.DATA_TIMEOUT_MS(40),.RAW_MODE(g==3),.MA_WINDOW(g==0?1:g==1?2:4)) decoder(
 .clk(clk),.rst_n(rst_n),.rx_byte(byte_data),.rx_valid(byte_valid),.distance_mm(mm[g]),.distance_age_ms(age[g]),.distance_valid(valid[g]),.online(online[g]),.sensor_fault(),.obstacle(),.slow_down(),.stop_req());
 end endgenerate
 always @(posedge clk)begin cycle=cycle+1;#1;for(integer n=0;n<4;n++)if(rst_n&&valid[n])begin count[n]++;last_valid[n]=cycle;end end
 task ticks(input integer n);begin repeat(n)@(negedge clk);#1;end endtask
 task byte_in(input[7:0]v);begin @(negedge clk);rx=0;ticks(10);for(integer b=0;b<8;b++)begin rx=v[b];ticks(10);end rx=1;ticks(10);end endtask
 task frame(input[15:0]v,input[7:0]cs);begin byte_in(255);byte_in(v[15:8]);byte_in(v[7:0]);byte_in(cs);ticks(8);end endtask
 task sample(input[15:0]v);begin frame(v,8'hff+v[15:8]+v[7:0]);end endtask
 task check_all(input[15:0]expected,input bit on,input string why);begin for(integer n=0;n<4;n++)begin checks++;if(mm[n]!=expected||online[n]!=on)$fatal(1,"%s mode%0d mm%0d expected%0d online%b",why,n,mm[n],expected,online[n]);end end endtask
 integer reference_state=0,zero_immediate_checks=0,reference_last_byte=0;reg[7:0]reference_hi=0,reference_lo=0;reg expect_zero=0;
 // Independent byte-level parser: every rejected checksum finishes its frame.
 // Check all RAW/MA outputs on the exact real UART checksum acceptance clock.
 always @(posedge clk)begin
 expect_zero=0;
 if(!rst_n)begin reference_state=0;reference_last_byte=0;end
 else begin
 if(reference_state!=0 && cycle-reference_last_byte>=2000)reference_state=0;
 if(byte_valid)begin reference_last_byte=cycle;case(reference_state)
 0:if(byte_data==255)reference_state=1;
 1:begin reference_hi=byte_data;reference_state=2;end
 2:begin reference_lo=byte_data;reference_state=3;end
 3:begin expect_zero=reference_hi==0&&reference_lo==0&&byte_data==255;reference_state=0;end
 endcase end end
 #1;if(expect_zero)begin zero_immediate_checks++;if(valid!==4'hf)$fatal(1,"zero not same checksum clock in all modes");for(integer n=0;n<4;n++)if(mm[n]!=4500||!online[n]||age[n]>1)$fatal(1,"zero immediate value/age mode%0d",n);end
 end
 integer saved_count[0:3],saved_cycle[0:3];
 task save;begin for(integer n=0;n<4;n++)begin saved_count[n]=count[n];saved_cycle[n]=last_valid[n];end end endtask
 task no_publish(input string why);begin for(integer n=0;n<4;n++)begin checks++;if(count[n]!=saved_count[n]||last_valid[n]!=saved_cycle[n])$fatal(1,"%s published mode%0d",why,n);end end endtask
 initial begin for(i=0;i<4;i++)begin count[i]=0;last_valid[i]=0;end ticks(5);rst_n=1;ticks(10);
 sample(0);check_all(4500,1,"startup zero");for(i=0;i<4;i++)if(count[i]!=1)$fatal(1,"startup zero notsingle");
 // Several different normal contributors must not leak into or out of zero.
 sample(200);check_all(200,1,"zero->near immediate");sample(0);check_all(4500,1,"near->zero immediate no confirmation/mean");sample(180);check_all(180,1,"zero->near cannot average4500");
 save();frame(1,8'hff);no_publish("bad checksum FF");
 for(j=0;j<6;j++)begin sample(0);check_all(4500,1,"next zero must recover from badchecksumFF");end
 save();frame(0,8'hfe);no_publish("corrupt zero nonFF");sample(0);check_all(4500,1,"next zero recovers nonFF checksum");
 save();frame(0,8'hfe);no_publish("corrupt zero cannot rejuvenate source");ticks(2500);no_publish("idle cannot create range");
 for(i=0;i<4;i++)if(age[i]<25)$fatal(1,"badchecksum refreshedage mode%0d age%0d",i,age[i]);
 ticks(1700);check_all(4500,0,"silence expires last zero");no_publish("expiration notmeasurement");
 sample(0);check_all(4500,1,"new actual zero recovers expired");
 sample(1);check_all(4500,0,"1 remains numeric invalid");sample(0);sample(29);check_all(4500,0,"29 remains numeric invalid");sample(0);sample(65535);check_all(4500,0,"FFFF remains numeric invalid");
 sample(0);sample(210);check_all(210,1,"final real near overrideszero");
 // Fill actual different-size mean windows, then verify zero is not a
 // contributor even when4490 is too close to4500 for fast-near bypass.
 sample(800);sample(820);sample(840);sample(860);
 if(mm[0]!=860||mm[1]!=850||mm[2]!=830||mm[3]!=860)$fatal(1,"MA windows not seeded %0d/%0d/%0d/%0d",mm[0],mm[1],mm[2],mm[3]);checks++;
 sample(0);check_all(4500,1,"filled MA -> zero bypass");sample(4490);check_all(4490,1,"zero cleared MA: real4490 not averaged with4500");
 // A02 has an inter-frame gap; incomplete/noisy frames recover after20ms.
 save();byte_in(255);byte_in(0);ticks(2100);no_publish("missing low/checksum cannot publish");sample(0);check_all(4500,1,"20ms partial frame timeout recovers");
 save();byte_in(255);byte_in(0);byte_in(0);ticks(2100);no_publish("missing checksum cannot publish");sample(0);check_all(4500,1,"droppedchecksum timeout recovers");
 save();byte_in(255);ticks(2100);no_publish("extra FF is not a range");sample(0);check_all(4500,1,"extraheader timeout recovers");
 $display("PASS v29.1 actual UART10bit bytes RAW+MA1/2/4 startupzero4500;near-zero-near nofalsemean;badchecksumFF/nonFF nextframe recovery;continuouszero;corruptzero noevent/age;silenceexpire;1..29/FFFF unchanged checks=%0d checksum_clock_zero_checks=%0d",checks,zero_immediate_checks);$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
