`timescale 1ns/1ps
// RGC1 is a camera session only. It cannot arm or command any motor.
// GMII input includes preamble/SFD, zero padding and Ethernet FCS.
module rgb_session_rx(
 input wire clk,rst_n,input wire [7:0] data,input wire valid,error,
 output reg session_valid,output reg [15:0] session_epoch,
 output reg [31:0] accepted,rejected
);
 reg active,sfd,bad;
 reg [3:0] preamble;
 reg [11:0] count;
 reg [47:0] destination;
 reg [15:0] ethertype;
 reg [127:0] body;
 reg [31:0] fcs,body_crc;
 // End-of-frame validation is registered before it drives the session CE.
 // Never put the full CRC/header/address comparison behind GMII valid's
 // falling edge in one 125 MHz path.
 reg check_pending,check_framing,check_fcs,check_destination,check_header,check_crc;
 reg [15:0] checked_epoch;
 function [31:0] crc_byte;
  input [31:0] previous;input [7:0] value;reg [31:0] c;integer k;
  begin c=previous^{24'd0,value};for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_byte=c;end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;sfd<=0;bad<=0;preamble<=0;count<=0;
   destination<=0;ethertype<=0;body<=0;fcs<=32'hffffffff;body_crc<=32'hffffffff;
   session_valid<=0;session_epoch<=0;accepted<=0;rejected<=0;
   check_pending<=0;check_framing<=0;check_fcs<=0;check_destination<=0;check_header<=0;check_crc<=0;checked_epoch<=0;
  end else begin
   session_valid<=0;
   check_pending<=active && !valid;
   check_framing<=sfd && !bad && count==64;
   check_fcs<=fcs==32'hdebb20e3;
   check_destination<=destination==48'h02ac13800001 || destination==48'hffffffffffff;
   check_header<=ethertype==16'h88ba && body[127:96]==32'h52474331 &&
       body[95:88]==1 && body[87:80]==1 && body[79:64]!=0 && body[63:32]==0;
   check_crc<=body[31:0]==~body_crc;
   checked_epoch<=body[79:64];
   if(check_pending)begin
    if(check_framing && check_fcs && check_destination && check_header && check_crc)begin
     session_valid<=1;session_epoch<=checked_epoch;accepted<=accepted+1'b1;
    end else rejected<=rejected+1'b1;
   end
   if(valid)begin
    active<=1;if(error)bad<=1;
    if(!sfd)begin
     if(data==8'h55 && preamble<8)preamble<=preamble+1'b1;
     else if(data==8'hd5 && preamble>=7)begin sfd<=1;count<=0;fcs<=32'hffffffff;body_crc<=32'hffffffff;body<=0;destination<=0;ethertype<=0;end
     else bad<=1;
    end else begin
     if(count<2047)count<=count+1'b1;else bad<=1;
     fcs<=crc_byte(fcs,data);
     if(count<6)destination<={destination[39:0],data};
     if(count==12 || count==13)ethertype<={ethertype[7:0],data};
     if(count>=14 && count<30)body<={body[119:0],data};
     if(count>=14 && count<26)body_crc<=crc_byte(body_crc,data);
     if(count>=30 && count<60 && data!=0)bad<=1;
    end
   end else if(active)begin
    active<=0;sfd<=0;bad<=0;preamble<=0;count<=0;
   end
  end
 end
endmodule
