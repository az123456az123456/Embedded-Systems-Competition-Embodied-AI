`timescale 1ns/1ps
module camera_raw1080_pair_tb;
 reg clk=0,rst_n=0; always #4 clk=~clk;
 reg [13:0] n1=0,n2=0;
 integer i1=0,i2=0,out=0,y,k;
 reg [7:0] a[0:961],b[0:961];
 wire rd1,rd2,wr,fault; wire [7:0] data;
 // Exercise production defaults: 960 bytes per sensor and 1080 valid rows.
 controller dut(.clk(clk),.rst_n(rst_n),.rdclk(),
  .rdreq1(rd1),.rddata1(a[i1]),.rdusedw1(n1),
  .rdreq2(rd2),.rddata2(b[i2]),.rdusedw2(n2),
  .wrclk(),.fifo_aclr(),.fifo_wrreq(wr),.fifo_wrdata(data),.pair_fault(fault));
 always @(posedge clk) if(rst_n)begin
  if(rd1)begin i1<=i1+1;n1<=n1-1;end
  if(rd2)begin i2<=i2+1;n2<=n2-1;end
  if(wr)begin
   if(out>=1922)$fatal(1,"row too long");
   if(out<962 && data!==a[out])$fatal(1,"left/header row=%d byte=%d",y,out);
   if(out>=962 && data!==b[out-960])$fatal(1,"right row=%d byte=%d",y,out);
   out<=out+1;
  end
 end
 initial begin
  repeat(3)@(negedge clk);rst_n=1;
  for(y=0;y<1080;y=y+1)begin
   @(negedge clk);i1=0;i2=0;out=0;
   a[0]=y;a[1]=y>>8;b[0]=y;b[1]=y>>8;
   for(k=2;k<962;k=k+1)begin a[k]=(k+y)%251;b[k]=(k+y+79)%251;end
   n1=962;
   repeat(12)begin @(negedge clk);if(wr)$fatal(1,"incomplete pair emitted");end
   n2=962;
   repeat(1950)@(negedge clk);
   if(out!=1922 || i1!=962 || i2!=962 || fault)$fatal(1,"row %d incomplete out=%d",y,out);
  end
  $display("PASS dual 960+960 RAW bytes, all 1080 rows including rows above 1023");$finish;
 end
 initial begin #30000000;$fatal(1,"timeout");end
endmodule
