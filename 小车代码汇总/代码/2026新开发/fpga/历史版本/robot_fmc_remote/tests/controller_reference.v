// Frozen pre-v4 row scheduler for cycle-equivalence regression.
// Pair only equal row indices. On mismatch discard to the next common row zero.
// FWFT source FIFOs retain complete payloads while their headers are compared.
module controller_reference #(parameter IMG_WIDTH=480, IMG_HEIGHT=1080)(
 input clk,rst_n, output rdclk,
 output reg rdreq1, input [7:0] rddata1,input [13:0] rdusedw1,
 output reg rdreq2, input [7:0] rddata2,input [13:0] rdusedw2,
 output wrclk,fifo_aclr,output reg fifo_wrreq,output reg [7:0] fifo_wrdata,
 output reg pair_fault
);
 localparam [8:0] WAIT=9'h001,H0=9'h002,H1=9'h004,CHECK=9'h008,
                  O0=9'h010,O1=9'h020,A=9'h040,B=9'h080,DROP=9'h100;
 (* syn_encoding="onehot" *) reg [8:0] state;
 reg [11:0] count;
 reg [15:0] row1,row2;
 reg have1,have2,take1,take2,drop1,drop2,resync;
 assign rdclk=clk;assign wrclk=clk;assign fifo_aclr=!rst_n;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=WAIT;count<=0;row1<=0;row2<=0;have1<=0;have2<=0;
   take1<=0;take2<=0;drop1<=0;drop2<=0;resync<=1;pair_fault<=0;
  end else case(state)
  WAIT:begin
   take1<=!have1 && rdusedw1>=IMG_WIDTH*2+2;
   take2<=!have2 && rdusedw2>=IMG_WIDTH*2+2;
   if((!have1 && rdusedw1>=IMG_WIDTH*2+2)||(!have2 && rdusedw2>=IMG_WIDTH*2+2))state<=H0;
   else if(have1 && have2)state<=CHECK;
  end
  H0:begin if(take1)row1[7:0]<=rddata1;if(take2)row2[7:0]<=rddata2;state<=H1;end
  H1:begin
   if(take1)begin row1[15:8]<=rddata1;have1<=1;end
   if(take2)begin row2[15:8]<=rddata2;have2<=1;end
   state<=CHECK;
  end
  CHECK:if(have1 && have2)begin
   count<=0;
   if(row1==row2 && row1<IMG_HEIGHT && (!resync || row1==0))begin resync<=0;state<=O0;end
   else begin
    if(row1!=row2 || row1>=IMG_HEIGHT || row2>=IMG_HEIGHT)pair_fault<=1;
    resync<=1;drop1<=row1!=0;drop2<=row2!=0;state<=DROP;
   end
  end else state<=WAIT;
  O0:state<=O1;
  O1:begin count<=0;state<=A;end
  A:if(count==IMG_WIDTH*2-1)begin count<=0;state<=B;end else count<=count+1'b1;
  B:if(count==IMG_WIDTH*2-1)begin have1<=0;have2<=0;state<=WAIT;end else count<=count+1'b1;
  DROP:if(count==IMG_WIDTH*2-1)begin
    if(drop1)have1<=0;if(drop2)have2<=0;state<=WAIT;
   end else count<=count+1'b1;
  default:state<=WAIT;
  endcase
 end
 always @* begin
  // Parallel decode avoids a priority mux on the FIFO read-enable paths.
  rdreq1=((state[1] || state[2]) && take1) || state[6] || (state[8] && drop1);
  rdreq2=((state[1] || state[2]) && take2) || state[7] || (state[8] && drop2);
  fifo_wrreq=0;fifo_wrdata=0;
  case(state)
  O0:begin fifo_wrreq=1;fifo_wrdata=row1[7:0];end
  O1:begin fifo_wrreq=1;fifo_wrdata=row1[15:8];end
  A:begin fifo_wrreq=1;fifo_wrdata=rddata1;end
  B:begin fifo_wrreq=1;fifo_wrdata=rddata2;end
  default:begin end
  endcase
 end
endmodule
