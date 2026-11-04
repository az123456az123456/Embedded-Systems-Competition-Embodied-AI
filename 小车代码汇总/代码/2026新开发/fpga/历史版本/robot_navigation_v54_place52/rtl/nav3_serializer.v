`timescale 1ns/1ps
// One256B BODY, synchronous32bit RAM prefetch, CRC16/FCS and12-byte IFG.
module nav3_serializer(
 input wire clk,reset_n,start,input wire[2:0]slot,input wire nva,
 output wire ram_req,output wire[8:0]ram_addr,input wire[31:0]ram_data,
 output reg active,done,output reg gmii_valid,output reg[7:0]gmii_data
);
 reg[8:0]position;reg[2:0]saved_slot;reg saved_nva;reg[31:0]word;
 reg[31:0]crc32;reg[15:0]crc16;reg[7:0]byte_value;
 wire[8:0]relative=position-9'd20;
 assign ram_req=active&&position>=20&&position<=272&&relative[1:0]==0;
 assign ram_addr={saved_slot,6'd0}+{3'd0,relative[7:2]};
 function[31:0]c32;input[31:0]previous;input[7:0]v;integer k;reg[31:0]c;
 begin c=previous^{24'd0,v};for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;c32=c;end endfunction
 function[15:0]c16;input[15:0]previous;input[7:0]v;integer k;reg[15:0]c;
 begin c=previous^{v,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;c16=c;end endfunction
 // Fold the byte already registered on GMII, not the current position/mux.
 // This retimes only the private CRC state by one cycle: the public bytes,
 // RAM prefetch, active/done and IFG timing remain unchanged.  At the first
 // CRC16/FCS byte use the next state so the preceding final byte is included.
 wire[31:0]crc32_next=c32(crc32,gmii_data);
 wire[15:0]crc16_next=c16(crc16,gmii_data);
 wire[31:0]fcs=~crc32,fcs_next=~crc32_next;
 wire[8:0]body_byte=position-9'd22;
 always @*begin
  byte_value=0;
  if(position<7)byte_value=8'h55;
  else if(position==7)byte_value=8'hd5;
  else if(position<14)byte_value=8'hff;
  else if(position<22)case(position)
   14:byte_value=8'h02;15:byte_value=8'hac;16:byte_value=8'h13;17:byte_value=8'h80;
   18:byte_value=0;19:byte_value=1;20:byte_value=8'h88;default:byte_value=saved_nva?8'hb6:8'hb9;
  endcase
  else if(position<278)case(body_byte[1:0])
   0:byte_value=word[31:24];1:byte_value=word[23:16];2:byte_value=word[15:8];default:byte_value=word[7:0];
  endcase
  else case(position)
   278:byte_value=crc16_next[15:8];279:byte_value=crc16[7:0];
   280:byte_value=fcs_next[7:0];281:byte_value=fcs[15:8];282:byte_value=fcs[23:16];283:byte_value=fcs[31:24];
   default:byte_value=0;
  endcase
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin active<=0;done<=0;gmii_valid<=0;gmii_data<=0;position<=0;saved_slot<=0;saved_nva<=0;word<=0;crc32<=32'hffffffff;crc16<=16'hffff;end
  else begin
   done<=0;gmii_valid<=0;
   if(!active)begin
    if(start)begin active<=1;position<=0;saved_slot<=slot;saved_nva<=nva;crc32<=32'hffffffff;crc16<=16'hffff;end
   end else begin
    gmii_valid<=position<284;gmii_data<=byte_value;
    if(position>=21&&position<=273&&(position-21)%4==0)word<=ram_data;
    if(position>=9&&position<=280)crc32<=crc32_next;
    if(position>=23&&position<=278)crc16<=crc16_next;
    if(position==295)begin active<=0;done<=1;position<=0;end
    else position<=position+1'b1;
   end
  end
 end
endmodule
