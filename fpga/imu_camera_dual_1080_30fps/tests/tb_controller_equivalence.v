`timescale 1ns/1ps
module tb_controller_equivalence;
 reg clk=0,rst_n=0;always #4 clk=~clk;
 reg [7:0] mem1[0:4095],mem2[0:4095];
 integer rp1=0,rp2=0,wp1=0,wp2=0,total1=0,total2=0,cycle=0,outputs=0;
 wire a1,a2,b1,b2,aw,bw;wire [7:0] ad,bd;
 wire [13:0] used1=wp1-rp1,used2=wp2-rp2;wire af,bf;
 controller #(.IMG_WIDTH(8),.IMG_HEIGHT(8)) changed(.clk(clk),.rst_n(rst_n),.rdclk(),
 .rdreq1(a1),.rddata1(mem1[rp1]),.rdusedw1(used1),.rdreq2(a2),.rddata2(mem2[rp2]),.rdusedw2(used2),
 .wrclk(),.fifo_aclr(),.fifo_wrreq(aw),.fifo_wrdata(ad),.pair_fault(af));
 controller_reference #(.IMG_WIDTH(8),.IMG_HEIGHT(8)) original(.clk(clk),.rst_n(rst_n),.rdclk(),
 .rdreq1(b1),.rddata1(mem1[rp1]),.rdusedw1(used1),.rdreq2(b2),.rddata2(mem2[rp2]),.rdusedw2(used2),
 .wrclk(),.fifo_aclr(),.fifo_wrreq(bw),.fifo_wrdata(bd),.pair_fault(bf));
 always @(negedge clk)if(rst_n)begin
  if(a1!==b1 || a2!==b2 || aw!==bw || af!==bf || (aw && ad!==bd))$fatal(1,"Cycle-equivalence failure at %d",cycle);
  if(aw)outputs=outputs+1;
 end
 always @(posedge clk)if(rst_n)begin
  if(a1)begin if(rp1>=wp1)$fatal(1,"FIFO1 underflow");rp1<=rp1+1;end
  if(a2)begin if(rp2>=wp2)$fatal(1,"FIFO2 underflow");rp2<=rp2+1;end
 end
 integer frame,row,k;
 initial begin
  for(frame=0;frame<12;frame=frame+1)for(row=0;row<8;row=row+1)begin
   mem1[total1]=row;mem1[total1+1]=0;
   mem2[total2]=(frame==2 && row==3)?19:row;mem2[total2+1]=0;
   for(k=2;k<18;k=k+1)begin mem1[total1+k]=(frame+row+k)%251;mem2[total2+k]=(3*frame+row+k)%251;end
   total1=total1+18;total2=total2+18;
  end
  #20;rst_n=1;
  for(cycle=0;cycle<15000;cycle=cycle+1)begin
   @(negedge clk);
   if(wp1<total1 && (cycle%5)!=0)wp1=wp1+1;
   if(wp2<total2 && (cycle%7)!=0)wp2=wp2+1;
  end
  if(outputs<1500 || !af)$fatal(1,"Insufficient valid rows / fault coverage");
  $display("PASS registered FIFO enables exactly match original per cycle: asynchronous arrivals, full rows, row mismatch, resync, payloads and fault flag (%0d output bytes)",outputs);$finish;
 end
endmodule
