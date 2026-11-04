// OV5640 internal AF MCU firmware loader. Images remain enabled on AF failure.
// status = {stage[3:0], error[3:0], last_read[7:0]}.
// error: 1=NACK, 2=transaction timeout, 3=wrong ID, 4=poll timeout.
// Stage F/error0 means single AF completed and continuous AF was acknowledged.
module ov5640_af_loader #(
 parameter FW_FILE="camera/ov5640_af.hex",
 parameter FW_BYTES=4077,
 parameter WAIT_CYCLES=500000,
 parameter TIMEOUT_CYCLES=5000000,
 parameter MAX_POLLS=500
)(input clk,input rst_n,input start,
 output reg wr_req,output reg rd_req,output reg [15:0]addr,
 output reg [7:0]wrdata,input [7:0]rddata,input rw_done,input nack,
 output [15:0]status);
 reg [7:0]rom[0:4076];
 initial $readmemh(FW_FILE,rom);
 reg [11:0]index;
 reg [7:0]rom_q;
 always @(posedge clk)rom_q<=rom[index];
 reg [3:0]stage,error;
 reg [7:0]last_read;
 reg [1:0]phase;
 reg [31:0]timer;
 reg [9:0]polls;
 reg running;
 assign status={stage,error,last_read};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   stage<=0;error<=0;last_read<=0;phase<=0;timer<=0;
   index<=0;polls<=0;running<=0;wr_req<=0;rd_req<=0;addr<=0;wrdata<=0;
  end else begin
   wr_req<=0;rd_req<=0;
   if(start)running<=1;
   if(running && error==0 && stage!=15)begin
    case(phase)
     0:begin
      timer<=0;phase<=1;
      case(stage)
       0:begin rd_req<=1;addr<=16'h300a;end
       1:begin rd_req<=1;addr<=16'h300b;end
       2:begin wr_req<=1;addr<=16'h3000;wrdata<=8'h20;end
       3:begin wr_req<=1;addr<=16'h8000+index;wrdata<=rom_q;end
       4:begin wr_req<=1;addr<=16'h3022+index;wrdata<=index==7?8'h7f:0;end
       5:begin wr_req<=1;addr<=16'h3000;wrdata<=0;end
       6:begin rd_req<=1;addr<=16'h3029;end
       7:begin wr_req<=1;addr<=16'h3022;wrdata<=3;end
       8:begin rd_req<=1;addr<=16'h3029;end
       9,12:begin wr_req<=1;addr<=16'h3023;wrdata<=1;end
       10:begin wr_req<=1;addr<=16'h3022;wrdata<=8;end
       11,14:begin rd_req<=1;addr<=16'h3023;end
       13:begin wr_req<=1;addr<=16'h3022;wrdata<=4;end
      endcase
     end
     1:begin
      timer<=timer+1;
      if(timer==TIMEOUT_CYCLES-1)error<=2;
      else if(rw_done)begin
       timer<=0;phase<=2;
       if(nack)error<=1;
       else begin
        if(stage==0 || stage==1 || stage==6 || stage==8 || stage==11 || stage==14)
         last_read<=rddata;
        case(stage)
         0:if(rddata!=8'h56)error<=3;else stage<=1;
         1:if(rddata!=8'h40)error<=3;else stage<=2;
         2:begin index<=0;stage<=3;end
         3:if(index==FW_BYTES-1)begin index<=0;stage<=4;end else index<=index+1;
         4:if(index==7)begin index<=0;stage<=5;end else index<=index+1;
         6,8,11,14:begin
          if((stage==6 && rddata==8'h70) || (stage==8 && rddata==8'h10) ||
             ((stage==11 || stage==14) && rddata==0))begin stage<=stage+1;polls<=0;end
          else if(polls==MAX_POLLS-1)error<=4;
          else polls<=polls+1;
         end
         default:stage<=stage+1;
        endcase
       end
      end
     end
     2:begin
      // Firmware byte writes need only a short settling interval. Polls wait 10ms.
      timer<=timer+1;
      if(timer==((stage==6 || stage==8 || stage==11 || stage==14)?WAIT_CYCLES:3))begin
       timer<=0;phase<=0;
      end
     end
     default:phase<=0;
    endcase
   end
  end
 end
endmodule
