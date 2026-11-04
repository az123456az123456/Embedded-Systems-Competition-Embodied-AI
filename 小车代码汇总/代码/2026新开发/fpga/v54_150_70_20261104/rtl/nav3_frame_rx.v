`timescale 1ns/1ps
// Fixed 256B BODY parser. Original bytes go directly to an allocated RAM bank;
// Identity remains registered; shape-only fields are checked as bytes arrive.
// Two RX pipeline stages added; wire format and CDC depth are unchanged.
module nav3_frame_rx(
 input wire clk,reset_n,input wire[7:0]data,input wire valid,error,
 input wire[3:0]lane_available,input wire[35:0]lane_base,input wire[7:0]lane_bank,
 output reg wr_en,output reg[8:0]wr_addr,output reg[31:0]wr_data,
 output reg publish,output reg[1:0]publish_lane,publish_bank,output reg[8:0]publish_bytes,
 output reg publish_family,output reg[7:0]publish_opcode,
 output reg cancel_pulse,output wire[417:0]cancel_desc,
 output reg[31:0]bad_frames,busy_frames
);
 reg active,sfd,bad,drop;reg[3:0]preamble;
 // The byte counter is used by the shape checker and the RAM writer.  Ask
 // synthesis to replicate its read cone instead of routing one counter bit
 // across every validation/write consumer.  This is a physical fanout hint;
 // it does not alter the counter or packet timing.
 (* syn_maxfan=8 *) reg[10:0]count;
 // U0: position, data, qualifiers and boundary belong to one byte transaction.
 // rx_count only creates positions; count/body_index are its registered bundle.
 reg[7:0]byte_data;reg byte_valid,byte_error,byte_sof,byte_eof;
 reg input_valid_q,input_sfd;reg[3:0]input_preamble;reg[10:0]rx_count;
 (* syn_maxfan=8 *) reg[10:0]body_index;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   byte_data<=0;byte_valid<=0;byte_error<=0;byte_sof<=0;byte_eof<=0;
   input_valid_q<=0;input_sfd<=0;input_preamble<=0;rx_count<=0;
   count<=0;body_index<=0;
  end else begin
   byte_data<=data;byte_valid<=valid;byte_error<=error;
   byte_sof<=valid&&!input_valid_q;byte_eof<=!valid&&input_valid_q;
   count<=rx_count;body_index<=rx_count-11'd14;input_valid_q<=valid;
   if(!valid)begin input_sfd<=0;input_preamble<=0;rx_count<=0;end
   else if(!input_sfd)begin
    if(data==8'h55&&input_preamble<8)input_preamble<=input_preamble+1'b1;
    else if(data==8'hd5&&input_preamble>=7)begin input_sfd<=1;rx_count<=0;end
   end else if(rx_count!=2047)rx_count<=rx_count+1'b1;
  end
 end
 // U2 result is valid only for this frame; U3 EOF has a separate valid bit.
 reg crc16_equal_q,crc16_ready;
 reg eof_qualification_valid,eof_wire_ok;
 reg[47:0]destination;reg[15:0]ethertype,crc16,got_crc16;reg[31:0]crc32;
 reg[767:0]header;
 reg[95:0]cancel_payload;
 reg[31:0]assembly,first_word,second_word;
 reg nav_family,nva_family,header_version_bad,header_reserved_bad;
 reg[16:0]nz_word;
 reg[7:0]name_length;reg name_bad;
 reg[15:0]page_offset,page_count;
 // Header length is complete at BODY89, well before payload validation.
 // Use a bounded precomputed end instead of a 32-bit add before each compare.
 reg[16:0]payload_end;
 reg page_shape_bad;
 reg ph0_gt16,ph0_ge4096,ph0_nz,ph2_zero,ph2_gt64,ph8_ge512,ph8_unaligned,
     ph10_ne32,ph12_ne100,pb14_ne64,pb15_ne64,pw0_ne1,pw12_gt100,
     pose_yaw_bad,age_low_gt250m,pose_flags_bad,pb0_gt5,pb1_gt3,
     state_flags_bad,ph40_gt4096,ph42_ge4096,ph44_ge4096,ph46_bad;
 wire[31:0]stream_word={assembly[23:0],byte_data};
 wire[15:0]stream_half={assembly[7:0],byte_data};
 reg nz_map,nz_revision,nz_localization,nz_plan,nz_request;
 // Shape needs only zero/nonzero; retain the original identity registers
 // for the coherent CANCEL descriptor, but remove their wide reduction cone.
 reg nz_transport,nz_session,nz_run,nz_goal;
 reg shape_q,shape_ready;
 reg state_owner_bad;
 reg[8:0]base;reg[1:0]lane,bank;
 reg tail_bad,reserved_bad,page_bad;reg[7:0]page_hi;
 // The RAM write and publish interfaces are deliberately two-stage.  The
 // parser still consumes one GMII byte per rx_clk, but the wide shape/count
 // cone terminates at a single local pending register instead of driving the
 // packet RAM and every publish/diagnostic consumer in the same edge.
 reg write_pending;reg[8:0]write_pending_addr;reg[31:0]write_pending_data;
 reg accept_pending,accept_drop_q,accept_cancel_q;reg[1:0]accept_lane_q,accept_bank_q;
 reg[8:0]accept_bytes_q;reg accept_family_q;reg[7:0]accept_opcode_q;
 wire[15:0]payload_bytes=header[63:48];
 wire[7:0]opcode=header[727:720];
 wire nva=nva_family;
 
 wire[63:0]transport=header[703:640],session=header[639:576],run=header[575:512];
 wire[127:0]goal=header[511:384];
 // Shape only needs zero/nonzero for these fields; original64bit values remain
 // in packet RAM for the CPU and owner checker, never reconstructed here.
 wire map_epoch=nz_map,map_revision=nz_revision,localization=nz_localization,plan=nz_plan,request_id=nz_request;
 // Header/payload remain unchanged through the following source-mailbox capture
 // edge. Only cancel_pulse qualifies this bus; avoid a redundant418-bit FF copy.
 assign cancel_desc={nva?cancel_payload[31:0]:cancel_payload[95:64],nva?cancel_payload[95:32]:64'd0,transport,goal,run,session,nva?2'd2:2'd1};
 function[31:0]c32;input[31:0]previous;input[7:0]v;integer k;reg[31:0]c;
 begin c=previous^{24'd0,v};for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;c32=c;end endfunction
 function[15:0]c16;input[15:0]previous;input[7:0]v;integer k;reg[15:0]c;
 begin c=previous^{v,8'd0};for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;c16=c;end endfunction
 // Decode the opcode as its byte arrives. Parallel format rules are captured
 // after the last BODY byte; the next cycle only selects registered results.
 // CRC/FCS reception leaves enough cycles for both stages before EOF.
 reg[15:0]op_onehot;
 wire task_id=nz_run&&nz_goal&&map_epoch&&map_revision&&localization;
 wire regular_ids=nz_transport&&nz_session;
 wire request_ids=regular_ids&&request_id;
 wire[15:0]nav_request_rule;
 // Bit order: 01 02 03 04 10 11 12 20 21 22 30 31 40 41 42 43.
 assign nav_request_rule[0]=payload_bytes==0;
 assign nav_request_rule[1]=request_ids&&payload_bytes==68&&nz_word[0]&&
       name_length>=1&&name_length<=63&&!name_bad;
 assign nav_request_rule[2]=nz_transport&&request_id&&!nz_session&&
       payload_bytes==16&&(|nz_word[1:0])&&(|nz_word[3:2]);
 assign nav_request_rule[3]=request_ids&&payload_bytes==24&&(|nz_word[1:0])&&
       !pw12_gt100&&!(|nz_word[5:4]);
 assign nav_request_rule[4]=request_ids&&payload_bytes==32&&map_epoch&&map_revision&&
       nz_word[0]&&!ph12_ne100&&!pb14_ne64&&!pb15_ne64&&(|nz_word[6:5])&&!nz_word[7];
 assign nav_request_rule[5]=request_ids&&payload_bytes==48&&map_epoch&&map_revision&&
       (|nz_word[1:0])&&!ph8_ge512&&!ph8_unaligned&&!ph10_ne32&&!nz_word[3];
 assign nav_request_rule[6]=request_ids&&payload_bytes==16&&map_epoch&&map_revision&&
       (|nz_word[1:0])&&!nz_word[3];
 assign nav_request_rule[7]=request_ids&&payload_bytes==24&&task_id&&nz_word[2]&&
       (|nz_word[4:3])&&!nz_word[5];
 assign nav_request_rule[8]=request_ids&&payload_bytes==24&&task_id&&plan&&
       (|nz_word[1:0])&&(|nz_word[3:2])&&(|nz_word[5:4]);
 assign nav_request_rule[9]=request_ids&&payload_bytes==8&&nz_run&&nz_goal&&!nz_word[1];
 assign nav_request_rule[10]=regular_ids&&payload_bytes==60&&map_epoch&&map_revision&&
       localization&&nz_word[3]&&(|nz_word[5:4])&&(|nz_word[13:12])&&
       !nz_word[9]&&!age_low_gt250m&&!pose_yaw_bad&&!pose_flags_bad;
 assign nav_request_rule[11]=regular_ids&&payload_bytes==60&&task_id&&plan&&
       nz_word[3]&&(|nz_word[5:4])&&(|nz_word[13:12])&&!nz_word[9]&&
       !age_low_gt250m&&!nz_word[2]&&!pose_flags_bad;
 assign nav_request_rule[12]=regular_ids&&payload_bytes==160&&!pb0_gt5&&!pb1_gt3&&
       !state_flags_bad&&!state_owner_bad;
 assign nav_request_rule[13]=request_ids&&payload_bytes==8&&task_id&&plan&&
       !ph0_ge4096&&!ph2_zero&&!ph2_gt64;
 assign nav_request_rule[14]=regular_ids&&payload_bytes==64&&!nz_word[7]&&!nz_word[15];
 assign nav_request_rule[15]=regular_ids&&payload_bytes==64&&task_id&&!ph0_gt16&&
       !ph40_gt4096&&!ph42_ge4096&&!ph44_ge4096&&!ph46_bad&&!nz_word[15];
 wire reply_fields_ok=!ph0_gt16&&!nz_word[7];
 wire[15:0]nav_reply_rule;
 assign nav_reply_rule[0]=reply_fields_ok&&payload_bytes==64;
 assign nav_reply_rule[9:1]={9{reply_fields_ok&&payload_bytes==32}};
 assign nav_reply_rule[12:10]=0;
 assign nav_reply_rule[13]=reply_fields_ok&&payload_bytes>=32&&!page_shape_bad&&
       !page_bad&&!(ph0_nz&&page_count!=0);
 assign nav_reply_rule[15:14]=0;
 wire[2:0]nva_request_rule;
 wire nva_ids=request_ids&&nz_run;
 assign nva_request_rule[0]=payload_bytes==0;
 assign nva_request_rule[1]=nva_ids&&payload_bytes==64&&!pw0_ne1&&
       (|nz_word[2:1])&&(|nz_word[4:3])&&(|nz_word[6:5])&&
       (|nz_word[8:7])&&(|nz_word[12:11])&&!(|nz_word[15:13]);
 assign nva_request_rule[2]=nva_ids&&payload_bytes==16&&!nz_word[3];
 reg shape_common_q,nva_reply_q;
 reg[15:0]nav_request_q,nav_reply_q;
 reg[2:0]nva_request_q;
 wire shape_selected=nva ?
       ((|op_onehot[2:0])&&(opcode[7]?nva_reply_q:|(op_onehot[2:0]&nva_request_q))) :
       |(op_onehot&(opcode[7]?nav_reply_q:nav_request_q));
 wire shape=shape_common_q&&shape_selected;
 wire crc16_equal;
 nav_eq_lut #(.WIDTH(16)) crc16_compare(.a(crc16),.b(got_crc16),.eq(crc16_equal),.ne());
 integer i;
 reg[1:0]selected_lane;reg[8:0]selected_base;
 always @*begin
  selected_lane=0;
  if(nav_family&&(byte_data==8'h30||byte_data==8'h31))selected_lane=1;
  if((nav_family&&byte_data==8'h22)||(nva_family&&byte_data==3))selected_lane=3;
  if(nva_family&&byte_data==2)selected_lane=2;
  selected_base=lane_base[selected_lane*9 +:9];
 end

 wire[8:0]view_limit=lane==0?9'd256:lane==3?9'd128:9'd160;
 // Keep the acceptance predicate as one registered-clock-domain decision.
 // Older netlists let the wide payload/shape cone fan directly into
 // publish_* and the diagnostic counters.  The current parser has no full
 // payload register; this scalar predicate keeps those consumers local.
 // U3 registers the complete final-FCS/EOF wire qualification; U4 only
 // merges already registered results. Invalid is distinct from not-yet-ready.
 (* syn_keep=1 *) wire eof_raw_qualification=sfd&&!bad&&count==11'd276&&
       destination==48'h02ac13800001&&
       ((nav_family&&ethertype==16'h88b9)||(nva_family&&ethertype==16'h88b6))&&
       crc32==32'hdebb20e3;
 (* syn_keep=1 *) wire frame_accept=eof_qualification_valid&&eof_wire_ok&&
       crc16_ready&&crc16_equal_q&&shape_ready&&shape_q;
 (* syn_keep=1 *) wire write_word_valid=
       (body_index==11'd7&&!drop)||
       (body_index==11'd8&&!drop)||
       (body_index>=11'd11&&body_index<view_limit&&body_index[1:0]==2'd3&&!drop);
 reg[8:0]write_word_addr;reg[31:0]write_word_data;
 always @* begin
  write_word_addr=base;write_word_data=first_word;
  if(body_index==11'd8)begin write_word_addr=base+1'b1;write_word_data=second_word;end
  else if(body_index>=11'd11)begin
   write_word_addr=base+{3'd0,body_index[7:2]};
   write_word_data={assembly[23:0],byte_data};
  end
 end
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   active<=0;sfd<=0;bad<=0;drop<=0;preamble<=0;destination<=0;ethertype<=0;
   crc32<=32'hffffffff;crc16<=16'hffff;got_crc16<=0;header<=0;cancel_payload<=0;nav_family<=0;nva_family<=0;header_version_bad<=0;header_reserved_bad<=0;
      nz_word<=0;name_length<=0;name_bad<=0;page_offset<=0;page_count<=0;payload_end<=0;page_shape_bad<=0;
      ph0_gt16<=0;ph0_ge4096<=0;ph0_nz<=0;ph2_zero<=0;ph2_gt64<=0;ph8_ge512<=0;ph8_unaligned<=0;ph10_ne32<=0;ph12_ne100<=0;pb14_ne64<=0;pb15_ne64<=0;pw0_ne1<=0;pw12_gt100<=0;pose_yaw_bad<=0;age_low_gt250m<=0;pose_flags_bad<=0;pb0_gt5<=0;pb1_gt3<=0;state_flags_bad<=0;ph40_gt4096<=0;ph42_ge4096<=0;ph44_ge4096<=0;ph46_bad<=0;
   nz_map<=0;nz_revision<=0;nz_localization<=0;nz_plan<=0;nz_request<=0;
   nz_transport<=0;nz_session<=0;nz_run<=0;nz_goal<=0;shape_q<=0;shape_ready<=0;
   op_onehot<=0;shape_common_q<=0;nav_request_q<=0;nav_reply_q<=0;nva_request_q<=0;nva_reply_q<=0;
   crc16_equal_q<=0;crc16_ready<=0;eof_qualification_valid<=0;eof_wire_ok<=0;
   assembly<=0;first_word<=0;second_word<=0;state_owner_bad<=0;base<=0;lane<=0;bank<=0;
   tail_bad<=0;reserved_bad<=0;page_bad<=0;page_hi<=0;
   wr_en<=0;wr_addr<=0;wr_data<=0;write_pending<=0;write_pending_addr<=0;write_pending_data<=0;
   accept_pending<=0;accept_drop_q<=0;accept_cancel_q<=0;accept_lane_q<=0;accept_bank_q<=0;accept_bytes_q<=0;accept_family_q<=0;accept_opcode_q<=0;
   publish<=0;publish_lane<=0;publish_bank<=0;publish_bytes<=0;publish_family<=0;publish_opcode<=0;
   cancel_pulse<=0;bad_frames<=0;busy_frames<=0;
  end else begin
   // The previous pending word is presented to the RAM wrapper while the
   // current byte computes the next pending word.  This keeps the RAM write
   // path shallow without changing byte order or packet contents.
   wr_en<=write_pending;wr_addr<=write_pending_addr;wr_data<=write_pending_data;
   write_pending<=0;publish<=0;cancel_pulse<=0;
   if(byte_valid)begin
    // A byte_valid byte cannot follow the end-of-frame idle cycle.  Clearing a
    // stale pending decision here makes the boundary explicit and avoids
    // replaying a publish if a malformed source violates that assumption.
    if(accept_pending)accept_pending<=0;
    if(byte_sof)begin eof_qualification_valid<=0;crc16_ready<=0;end
    active<=1;if(byte_error)bad<=1;
    if(!sfd)begin
     if(byte_data==8'h55&&preamble<8)preamble<=preamble+1'b1;
     else if(byte_data==8'hd5&&preamble>=7)begin
      sfd<=1;crc32<=32'hffffffff;crc16<=16'hffff;got_crc16<=0;
      destination<=0;ethertype<=0;header<=0;cancel_payload<=0;nav_family<=0;nva_family<=0;header_version_bad<=0;header_reserved_bad<=0;
      nz_word<=0;name_length<=0;name_bad<=0;page_offset<=0;page_count<=0;payload_end<=0;page_shape_bad<=0;
      ph0_gt16<=0;ph0_ge4096<=0;ph0_nz<=0;ph2_zero<=0;ph2_gt64<=0;ph8_ge512<=0;ph8_unaligned<=0;ph10_ne32<=0;ph12_ne100<=0;pb14_ne64<=0;pb15_ne64<=0;pw0_ne1<=0;pw12_gt100<=0;pose_yaw_bad<=0;age_low_gt250m<=0;pose_flags_bad<=0;pb0_gt5<=0;pb1_gt3<=0;state_flags_bad<=0;ph40_gt4096<=0;ph42_ge4096<=0;ph44_ge4096<=0;ph46_bad<=0;tail_bad<=0;reserved_bad<=0;page_bad<=0;state_owner_bad<=0;drop<=0;
      nz_map<=0;nz_revision<=0;nz_localization<=0;nz_plan<=0;nz_request<=0;
      nz_transport<=0;nz_session<=0;nz_run<=0;nz_goal<=0;shape_q<=0;shape_ready<=0;
   op_onehot<=0;shape_common_q<=0;nav_request_q<=0;nav_reply_q<=0;nva_request_q<=0;nva_reply_q<=0;
   crc16_equal_q<=0;crc16_ready<=0;eof_qualification_valid<=0;eof_wire_ok<=0;
     end else bad<=1;
    end else begin
     if(count==2047)bad<=1;
     crc32<=c32(crc32,byte_data);
     if(count<6)destination<={destination[39:0],byte_data};
     if(count==12||count==13)ethertype<={ethertype[7:0],byte_data};
     if(count>=14&&count<270)begin
      crc16<=c16(crc16,byte_data);assembly<={assembly[23:0],byte_data};
      // Static byte enables remove wide subtract/shift decoders on the
      // header CE path. Each selected slice still receives the same byte.
      for(i=0;i<96;i=i+1)
       if(i==5||(i>=8&&i<48)||(i>=88&&i<90))
        if(body_index==i)header[767-i*8 -:8]<=byte_data;
      if(body_index==5)op_onehot<={
       byte_data[6:0]==7'h43,byte_data[6:0]==7'h42,byte_data[6:0]==7'h41,byte_data[6:0]==7'h40,
       byte_data[6:0]==7'h31,byte_data[6:0]==7'h30,byte_data[6:0]==7'h22,byte_data[6:0]==7'h21,
       byte_data[6:0]==7'h20,byte_data[6:0]==7'h12,byte_data[6:0]==7'h11,byte_data[6:0]==7'h10,
       byte_data[6:0]==7'h04,byte_data[6:0]==7'h03,byte_data[6:0]==7'h02,byte_data[6:0]==7'h01};
      if(body_index==89)payload_end<={1'b0,stream_half}+17'd96;
      if(body_index==3)begin nav_family<=stream_word==32'h4e415633;nva_family<=stream_word==32'h4e564131;end
      if(body_index==4)header_version_bad<=byte_data!=1;
      if((body_index>=6&&body_index<8)||(body_index>=90&&body_index<96))
       if(byte_data!=0)header_reserved_bad<=1;
      if(body_index>=8&&body_index<16)nz_transport<=nz_transport||byte_data!=0;
      if(body_index>=16&&body_index<24)nz_session<=nz_session||byte_data!=0;
      if(body_index>=24&&body_index<32)nz_run<=nz_run||byte_data!=0;
      if(body_index>=32&&body_index<48)nz_goal<=nz_goal||byte_data!=0;
      if(body_index>=48&&body_index<56)nz_map<=nz_map||byte_data!=0;
      if(body_index>=56&&body_index<64)nz_revision<=nz_revision||byte_data!=0;
      if(body_index>=64&&body_index<72)nz_localization<=nz_localization||byte_data!=0;
      if(body_index>=72&&body_index<80)nz_plan<=nz_plan||byte_data!=0;
      if(body_index>=80&&body_index<88)nz_request<=nz_request||byte_data!=0;
      for(i=0;i<12;i=i+1)
       if(body_index==96+i)cancel_payload[95-i*8 -:8]<=byte_data;
      for(i=0;i<17;i=i+1)
       if(body_index==99+i*4)nz_word[i]<=stream_word!=0;
      if(body_index==100)name_length<=byte_data;
      if(body_index>=101&&body_index<164)
       if((body_index-101<name_length)?byte_data==0:byte_data!=0)name_bad<=1;
      case(body_index)
       96:pb0_gt5<=byte_data>5;
       97:begin ph0_gt16<=stream_half>16;ph0_ge4096<=stream_half>=4096;ph0_nz<=stream_half!=0;pb1_gt3<=byte_data>3;end
       99:begin ph2_zero<=stream_half==0;ph2_gt64<=stream_half>64;pw0_ne1<=stream_word!=1;end
       103:state_flags_bad<=(stream_word&32'hffffff00)!=0;
       105:begin ph8_ge512<=stream_half>=512;ph8_unaligned<=stream_half[4:0]!=0;end
       107:begin ph10_ne32<=stream_half!=32;pose_yaw_bad<=$signed(stream_word)< -32'sd3141593||$signed(stream_word)>=32'sd3141593;end
       109:ph12_ne100<=stream_half!=100;
       110:pb14_ne64<=byte_data!=64;
       111:begin pb15_ne64<=byte_data!=64;pw12_gt100<=stream_word>100;end
       121:page_offset<=stream_half;
       123:begin
        page_count<=stream_half;
        page_shape_bad<=stream_half>16'd64||
         {2'b0,payload_bytes}!=(18'd32+{1'b0,stream_half,1'b0})||
         page_offset>16'd4096||({1'b0,page_offset}+{1'b0,stream_half})>17'd4096;
       end
       137:ph40_gt4096<=stream_half>4096;
       139:begin ph42_ge4096<=stream_half>=4096;age_low_gt250m<=stream_word>250000000;end
       141:ph44_ge4096<=stream_half>=4096;
       143:ph46_bad<=(stream_half&16'hfffe)!=0;
       155:pose_flags_bad<=(stream_word&32'hfffffffc)!=0;
      endcase
      if(body_index>=96&&{6'd0,body_index}>=payload_end&&byte_data!=0)tail_bad<=1;
      if((body_index>=220&&body_index<223&&byte_data!=0)||(body_index==223&&byte_data>1))state_owner_bad<=1;
      if(nav_family&&opcode==8'h40&&((body_index>=226&&body_index<=227)||body_index>=248)&&byte_data!=0)reserved_bad<=1;
      if(opcode==8'hc1&&body_index>=128&&{6'd0,body_index}<payload_end)begin
       if(!body_index[0])page_hi<=byte_data;
       else if({page_hi,byte_data}>=4096)page_bad<=1;
      end
      if(body_index==3)first_word<={assembly[23:0],byte_data};
      if(body_index==5)begin lane<=selected_lane;base<=selected_base;bank<=lane_bank[selected_lane*2 +:2];drop<=!lane_available[selected_lane];end
      if(body_index==7)second_word<={assembly[23:0],byte_data};
      if(write_word_valid)begin
       write_pending<=1;write_pending_addr<=write_word_addr;write_pending_data<=write_word_data;
      end
     end
     // count269 consumed the last BODY byte, including the final reserved,
     // tail and page checks. At count270 capture independent rule results;
     // count271 selects them without a length/ID/opcode priority mux chain.
     if(count==270)begin
      shape_common_q<=(nav_family||nva_family)&&!header_version_bad&&
          !header_reserved_bad&&payload_bytes<=160&&!tail_bad&&!reserved_bad;
      nav_request_q<=nav_request_rule;nav_reply_q<=nav_reply_rule;
      nva_request_q<=nva_request_rule;
      nva_reply_q<=reply_fields_ok&&payload_bytes==32;
     end
     if(count==271)begin shape_q<=shape;shape_ready<=1;end
     if(count==270||count==271)got_crc16<={got_crc16[7:0],byte_data};
     // Count271 completed got_crc16. Count272 is the first FCS byte.
     if(count==272)begin crc16_equal_q<=crc16_equal;crc16_ready<=1;end
    end
   end else if(active)begin
    if(accept_pending)begin
     // U5. Keep identity/header/payload stable through the U6 source capture.
     if(accept_drop_q)busy_frames<=busy_frames+1'b1;
     else begin
      publish<=1;publish_lane<=accept_lane_q;publish_bank<=accept_bank_q;publish_bytes<=accept_bytes_q;
      publish_family<=accept_family_q;publish_opcode<=accept_opcode_q;
     end
     if(accept_cancel_q)cancel_pulse<=1;
     accept_pending<=0;active<=0;sfd<=0;bad<=0;drop<=0;preamble<=0;
     shape_ready<=0;crc16_ready<=0;eof_qualification_valid<=0;
    end else if(eof_qualification_valid)begin
     // U4: only a completed qualification can accept or reject the frame.
     eof_qualification_valid<=0;
     if(frame_accept)accept_pending<=1;
     else begin
      bad_frames<=bad_frames+1'b1;active<=0;sfd<=0;bad<=0;drop<=0;preamble<=0;
      shape_ready<=0;crc16_ready<=0;
     end
    end else if(byte_eof)begin
     // U3: first aligned idle records final CRC32/address/length qualification
     // and the entire publish transaction. Do not clear a frame still pending.
     eof_wire_ok<=eof_raw_qualification;eof_qualification_valid<=1;
     accept_drop_q<=drop;accept_lane_q<=lane;accept_bank_q<=bank;
     accept_bytes_q<=lane==0?9'd256:lane==3?(nva?9'd112:9'd104):9'd160;
     accept_family_q<=nva;accept_opcode_q<=opcode;
     accept_cancel_q<=(!nva&&opcode==8'h22)||(nva&&opcode==3);
    end
   end
  end
 end
endmodule
