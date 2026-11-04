`timescale 1ns/1ps
module setup_case #(parameter MODE=0)(input clk,rst_n,output reg finished=0);
 wire wr,rd,locked;wire [15:0]addr,failed;wire [7:0]data,error,stage;wire [511:0]snap;
 reg done=0,nack=0;reg [7:0]rddata=0;reg [7:0]regs[0:65535];
 reg pending=0,iswrite;reg [15:0]savedaddr;reg [7:0]saveddata;integer waitcount=0,transactions=0,i;
 raw_camera_setup dut(.cfg_clk(clk),.cam2_rst_n_d(rst_n),.read0(rddata),.done0(done),.nack0(nack),
 .wr0(wr),.rd0(rd),.addr0(addr[14:0]),.data0(data),.led_d(locked),.error0(error[2:0]),.stage0(stage[3:0]),.failed0(failed[14:0]),.snapshot0(snap),.n33_3(),.n30_6());
 assign addr[15]=0;assign failed[15]=0;assign error[7:3]=0;assign stage[7:4]=0;
 // Skip only long idle delay counters, not state, command, comparison or RAM logic.
 always @(negedge clk)begin
  if(rst_n && !finished)begin
   if(stage==0)force dut.timer=32'd1049999;
   else if(stage==2)force dut.timer=32'd249999999;
   else if(stage==7)force dut.timer=32'd24999999;
   else if(stage==1 && dut.phase==3)force dut.timer=32'd249999;
   else if(MODE==2 && dut.phase==2)force dut.timer=32'd999999;
   else release dut.timer;
  end else release dut.timer;
 end

 initial begin
  for(i=0;i<65536;i=i+1)regs[i]=0;
  regs['h300a]=MODE==3?8'h55:8'h56;regs['h300b]=8'h40;
  regs['h3501]=8'h21;regs['h3502]=8'ha0;regs['h350b]=8'h10;
  regs['h3400]=4;regs['h3402]=4;regs['h3404]=4;
 end
 always @(posedge clk)begin
  done<=0;nack<=0;
  if(rst_n)begin
   if(wr || rd)begin
    if(pending)$fatal(1,"overlap");pending<=1;waitcount<=3;savedaddr<=addr;saveddata<=data;iswrite<=wr;transactions<=transactions+1;
   end
   if(pending && MODE!=2)begin
    if(waitcount!=0)waitcount<=waitcount-1;
    else begin
     pending<=0;done<=1;
     if(MODE==1)nack<=1;
     else if(iswrite)regs[savedaddr]<=saveddata;
     else if(MODE==4 && savedaddr=='h3503)rddata<=0;
     else if(MODE==5 && stage==9 && savedaddr=='h3501)rddata<=regs[savedaddr]^1;
     else rddata<=regs[savedaddr];
    end
   end
   if(!finished && (locked || error!=0))begin
    if(MODE==0)begin
     if(!locked || error || transactions!=391)$fatal(1,"normal transactions/status %d %d %d",transactions,locked,error);
     if(snap[511-2*8 -:8]!=3 || snap[511-8*8 -:8]!=1 || snap[511-4*8 -:8]!=8'h21)$fatal(1,"snapshot values");
     if(regs['h3503]!=3 || regs['h3406]!=1)$fatal(1,"not locked");
    end else begin
     if(locked)$fatal(1,"locked despite fault");
     if(MODE==1 && error!=1 || MODE==2 && error!=6 || MODE==3 && error!=2 || MODE==4 && error!=3 || MODE==5 && error!=4)$fatal(1,"incorrect error %d mode %d",error,MODE);
    end
    finished<=1;$display("PASS setup mode=%0d transactions=%0d error=%0d failed=%h",MODE,transactions,error,failed);
   end
  end
 end
endmodule
module tb_setup;
 reg clk=0,rst_n=0;always #10 clk=~clk;wire [5:0]done;
 genvar n;generate for(n=0;n<6;n=n+1)begin:g setup_case #(.MODE(n)) c(clk,rst_n,done[n]);end endgenerate
 initial begin #100;rst_n=1;wait(&done);repeat(50)@(posedge clk);$display("PASS all startup/lock/readback/fault cases");$finish;end
 initial begin #3000000;$fatal(1,"setup test timeout");end
endmodule
