// Diagnostic telemetry only. Multibit counters are sampled asynchronously;
// use stable repeated line lengths and approximate PCLK rates, not for control.
module camera_diag #(parameter PERIOD=50000000, parameter BAUD_DIV=434)(
 input [15:0] af1,af2,
 input clk, reset_n, input [2:0] init_done,
 input pclk1,href1,vsync1,pclk2,href2,vsync2,
 output reg uart_tx=1'b1);
 reg [31:0] pc1=0,pc2=0;
 reg [15:0] len1=0,len2=0,last1=0,last2=0,frames1=0,frames2=0;
 reg h1=0,h2=0,v1=0,v2=0;
 always @(posedge pclk1) begin
  pc1<=pc1+1'b1;h1<=href1;v1<=vsync1;
  if(href1)len1<=len1+1'b1;else len1<=0;
  if(h1&&!href1)last1<=len1;
  if(vsync1&&!v1)frames1<=frames1+1'b1;
 end
 always @(posedge pclk2) begin
  pc2<=pc2+1'b1;h2<=href2;v2<=vsync2;
  if(href2)len2<=len2+1'b1;else len2<=0;
  if(h2&&!href2)last2<=len2;
  if(vsync2&&!v2)frames2<=frames2+1'b1;
 end
 reg [130:0] sample1=0,sample2=0;
 always @(posedge clk)begin
  sample1<={init_done,pc1,last1,frames1,pc2,last2,frames2};sample2<=sample1;
 end
 reg [31:0] timer=0;
 reg [8*24-1:0] packet=0;
 reg [15:0] baud=0;
 reg [4:0] index=0;
 reg [3:0] bitno=0;
 reg busy=0;
 wire [7:0] octet=packet[191:184];
 always @(posedge clk)begin
  if(!reset_n)begin timer<=0;busy<=0;uart_tx<=1;end
  else if(!busy)begin
   uart_tx<=1;
   if(timer==PERIOD-1)begin
    timer<=0;packet<={16'h4341,5'b0,sample2,af1,af2,8'h0a};
    busy<=1;baud<=0;bitno<=0;index<=0;
   end else timer<=timer+1'b1;
  end else if(baud==0)begin
   baud<=BAUD_DIV-1;
   if(bitno==0)uart_tx<=0;
   else if(bitno<9)uart_tx<=octet[bitno-1];
   else uart_tx<=1;
   if(bitno==9)begin
    bitno<=0;packet<=packet<<8;
    if(index==23)busy<=0;else index<=index+1'b1;
   end else bitno<=bitno+1'b1;
  end else baud<=baud-1'b1;
 end
endmodule
