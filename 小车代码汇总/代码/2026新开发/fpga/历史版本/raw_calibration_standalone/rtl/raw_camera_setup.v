`timescale 1ns/1ps
// Calibration-only startup: unchanged v40 RAW8 sensor table, then freeze the
// current AEC/AGC/AWB state once. No asynchronous host register writes or AF.
module raw_camera_setup #(
 parameter BOOT_CYCLES=1050000, WARMUP_CYCLES=250000000,
 parameter SETTLE_CYCLES=25000000, TIMEOUT_CYCLES=1000000,
 parameter RESET_DELAY_CYCLES=250000
)(input clk,rst_n,input [7:0] rddata,input done,nack,
 output reg wr,rd,output reg [15:0] addr,output reg [7:0] data,
 output reg locked,output reg [7:0] error,output reg [7:0] stage,
 output reg [15:0] failed_addr,output reg [511:0] snapshot);
 reg [8:0] index;reg [31:0] timer;reg [1:0] phase;
 wire [23:0] lut;
 ov5640_init_table_rgb #(.IMAGE_WIDTH(960),.IMAGE_HEIGHT(1080),.IMAGE_FLIP_EN(1),.IMAGE_MIRROR_EN(0))
  table_raw(.clk(clk),.addr(index),.q(lut));
 reg [511:0] working;reg [7:0] aec_ctrl,awb_ctrl;
 function [15:0] reg_address;
  input [5:0] n;
  begin case(n)
   0:reg_address=16'h300a;1:reg_address=16'h300b;2:reg_address=16'h3503;
   3:reg_address=16'h3500;4:reg_address=16'h3501;5:reg_address=16'h3502;
   6:reg_address=16'h350a;7:reg_address=16'h350b;8:reg_address=16'h3406;
   9:reg_address=16'h3400;10:reg_address=16'h3401;11:reg_address=16'h3402;
   12:reg_address=16'h3403;13:reg_address=16'h3404;14:reg_address=16'h3405;
   15:reg_address=16'h5000;16:reg_address=16'h5001;17:reg_address=16'h4001;
   18:reg_address=16'h4005;19:reg_address=16'h5480;
   20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35:reg_address=16'h3800+n-20;
   36,37,38,39,40,41:reg_address=16'h3810+n-36;
   42:reg_address=16'h3820;43:reg_address=16'h3821;44:reg_address=16'h3034;
   45:reg_address=16'h3035;46:reg_address=16'h3036;47:reg_address=16'h3037;
   48:reg_address=16'h3108;49:reg_address=16'h460c;50:reg_address=16'h3824;
   51:reg_address=16'h4300;52:reg_address=16'h501f;53:reg_address=16'h4740;
   54:reg_address=16'h3a00;55:reg_address=16'h3008;56:reg_address=16'h3000;
   57:reg_address=16'h3600;58:reg_address=16'h3601;59:reg_address=16'h3022;
   60:reg_address=16'h3023;61:reg_address=16'h3029;62:reg_address=16'h56a1;
   default:reg_address=16'h4000;
  endcase end
 endfunction
 // stage: 0 boot,1 table,2 auto settle,3 readAEC,4 freezeAEC,
 // 5 readAWB,6 freezeAWB,7 settle,8 first readback,9 verification,10 locked.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   index<=0;timer<=0;phase<=0;wr<=0;rd<=0;addr<=0;data<=0;
   locked<=0;error<=0;stage<=0;failed_addr<=0;snapshot<=0;working<=0;aec_ctrl<=0;awb_ctrl<=0;
  end else begin
   wr<=0;rd<=0;
   if(error==0 && !locked)begin
    if(stage==0 || stage==2 || stage==7)begin
     timer<=timer+1'b1;
     if(timer==((stage==0)?BOOT_CYCLES-1:(stage==2)?WARMUP_CYCLES-1:SETTLE_CYCLES-1))begin
      timer<=0;phase<=0;stage<=stage==0?1:stage==2?3:8;index<=0;
     end
    end else if(stage==1 && phase==3)begin
     timer<=timer+1'b1;
     if(timer==RESET_DELAY_CYCLES-1)begin timer<=0;phase<=0;end
    end else case(phase)
     0:begin phase<=1;timer<=0;end // registered ROM/address setup
     1:begin
      phase<=2;timer<=0;
      if(stage==1)begin wr<=1;addr<=lut[23:8];data<=lut[7:0];end
      else if(stage==3 || stage==5)begin rd<=1;addr<=stage==3?16'h3503:16'h3406;end
      else if(stage==4 || stage==6)begin wr<=1;addr<=stage==4?16'h3503:16'h3406;data<=stage==4?(aec_ctrl|8'h03):(awb_ctrl|8'h01);end
      else begin rd<=1;addr<=reg_address(index[5:0]);end
     end
     2:begin
      timer<=timer+1'b1;
      if(done)begin
       timer<=0;phase<=0;
       if(nack)begin error<=1;failed_addr<=addr;end
       else case(stage)
        1:begin
         if(index==258)begin stage<=2;index<=0;end
         else begin index<=index+1'b1;if(index==1)phase<=3;end
        end
        3:begin aec_ctrl<=rddata;stage<=4;end
        4:stage<=5;
        5:begin awb_ctrl<=rddata;stage<=6;end
        6:stage<=7;
        8,9:begin
         if(stage==8)working[511-index[5:0]*8 -: 8]<=rddata;
         // Compare only locked control values; average light meter and
         // unrelated AF status are diagnostic, not stable configuration.
         if(stage==9 && index>=2 && index<=14 && rddata!=working[511-index[5:0]*8 -: 8])begin error<=4;failed_addr<=addr;end
         if(index==0 && rddata!=8'h56 || index==1 && rddata!=8'h40)begin error<=2;failed_addr<=addr;end
         if(index==2 && (rddata&8'h03)!=3 || index==8 && !rddata[0])begin error<=3;failed_addr<=addr;end
         if(index==63)begin
          index<=0;
          if(stage==8)stage<=9;
          else begin snapshot<=working;locked<=1;stage<=10;end
         end else index<=index+1'b1;
        end
        default:begin error<=5;failed_addr<=addr;end
       endcase
      end else if(timer==TIMEOUT_CYCLES-1)begin error<=6;failed_addr<=addr;end
     end
     default:phase<=0;
    endcase
   end
  end
 end
endmodule
