`timescale 1ns/1ps
// NAV3 immutable bank ingress. RX owns every bank role; CPU pins via handshake.
module nav3_ingress (
 input wire rx_clk,rx_reset_n,cpu_clk,cpu_reset_n,
 input wire [31:0] device_ms,
 input wire [7:0] gmii_data,input wire gmii_valid,gmii_error,
 input wire bus_req,input wire [11:0] bus_offset,
 input wire [31:0] bus_wdata,input wire [3:0] bus_wstrb,
 output reg bus_rsp_valid,output reg [31:0] bus_rsp_data,
 output wire [127:0] pin_tokens,pin_rx_ms,
 output wire [15:0] pin_flags,output wire [35:0] pin_bytes,
 input wire source_req,input wire [1:0] source_lane,input wire [5:0] source_word,
 output wire source_grant,output reg source_valid,output reg [31:0] source_data,
 output wire cancel_valid,output wire [417:0] cancel_desc,input wire cancel_ready
);
 function[8:0]physical;input[1:0]l,b;begin
  case(l)
   0:physical=0;
   1:case(b)0:physical=64;1:physical=104;default:physical=144;endcase
   2:case(b)0:physical=184;1:physical=224;default:physical=264;endcase
   3:case(b)0:physical=304;1:physical=336;default:physical=368;endcase
  endcase
 end endfunction
 function[3:0]slot;input[1:0]l,b;begin
  case(l)0:slot={2'd0,b};1:slot=4'd3+{2'd0,b};2:slot=4'd6+{2'd0,b};default:slot=4'd9+{2'd0,b};endcase
 end endfunction
 function[8:0]response_bytes;input[1:0]l;input[1:0]meta;begin
  if(!meta[1])response_bytes=0;
  else case(l)0:response_bytes=9'd256;1,2:response_bytes=9'd160;default:response_bytes=meta[0]?9'd112:9'd104;endcase
 end endfunction
 reg[3:0]rx_pinned,rx_latest;
 reg[1:0]rx_pin_bank[0:3],rx_latest_bank[0:3];
 reg[31:0]rx_pin_token[0:3],rx_serial[0:3],rx_latest_token[0:3];
 reg[1:0]bank_meta[0:11];
 reg[31:0]superseded[0:3];
 wire[3:0]cmd_ready,cmd_valid,resp_ready,resp_valid,pub_ready,pub_valid;
 reg[3:0]available;reg[35:0]bases;reg[7:0]banks;
 wire [3:0] rx_serial_full;
 wire [31:0] rx_serial_next[0:3];
 wire wr_en,publish,cancel_pulse;wire[8:0]wr_addr,publish_bytes;
 wire[31:0]wr_data,bad_frames,busy_frames;wire[1:0]publish_lane,publish_bank;wire publish_family;wire[7:0]publish_opcode;
 reg[51:0]pub_data[0:3];
 genvar sn,sb;
 generate for(sn=0;sn<4;sn=sn+1)begin: serial_bound
  nav_eq_reduce_lut4 #(.N(32)) full(.v(rx_serial[sn]),.q(rx_serial_full[sn]));
  // Independent bit carries avoid an eight-bit adder after the 24-bit
  // all-ones reduction on the critical most-significant token bits.
  assign rx_serial_next[sn][0]=~rx_serial[sn][0];
  for(sb=1;sb<32;sb=sb+1)begin: bit_step
   wire carry;
   nav_eq_reduce_lut4 #(.N(sb)) low_full(.v(rx_serial[sn][sb-1:0]),.q(carry));
   assign rx_serial_next[sn][sb]=rx_serial[sn][sb]^carry;
  end
  // Each lane writes its own counter and publication payload. There is no
  // dynamic read/modify/write mux in the serial-register feedback path.
  always @(posedge rx_clk or negedge rx_reset_n)begin
   if(!rx_reset_n)begin rx_serial[sn]<=0;rx_latest_token[sn]<=0;pub_data[sn]<=0;end
   else if(publish&&publish_lane==sn)begin
    rx_serial[sn]<=rx_serial_next[sn];
    rx_latest_token[sn]<=rx_serial_next[sn];
    pub_data[sn]<={publish_family,publish_opcode,rx_serial_next[sn],publish_bank,publish_bytes};
   end
  end
 end endgenerate
 integer l,b;
 always @*begin
  available=0;bases=0;banks=0;
  for(l=0;l<4;l=l+1)begin
   if(l==0)begin available[l]=!rx_pinned[l];bases[l*9 +:9]=0;end
   else for(b=2;b>=0;b=b-1)begin
    if((!rx_pinned[l]||rx_pin_bank[l]!=b)&&(!rx_latest[l]||rx_latest_bank[l]!=b))begin
     available[l]=1;banks[l*2 +:2]=b[1:0];bases[l*9 +:9]=physical(l[1:0],b[1:0]);
    end
   end
   if(rx_serial_full[l]||!pub_ready[l])available[l]=0;
  end
 end
 wire[417:0]cancel_wire;
 // Per-lane diagnostic increments have no dynamic bank read/write mux.  Each
 // byte uses the old lower-byte all-ones prefix, preserving 32-bit wraparound.
 // These counters still observe old rx_latest on a simultaneous acquire.
 wire [31:0] superseded_next[1:3];
 genvar dc,db;
 generate for(dc=1;dc<4;dc=dc+1)begin: diagnostic_increment
  assign superseded_next[dc][7:0]=superseded[dc][7:0]+8'd1;
  for(db=1;db<4;db=db+1)begin: byte_step
   wire carry;
   nav_eq_reduce_lut4 #(.N(db*8)) low_full(.v(superseded[dc][db*8-1:0]),.q(carry));
   assign superseded_next[dc][db*8+:8]=superseded[dc][db*8+:8]+carry;
  end
  always @(posedge rx_clk or negedge rx_reset_n)begin
   if(!rx_reset_n)superseded[dc]<=0;
   else if(publish&&publish_lane==dc&&rx_latest[dc])
    superseded[dc]<=superseded_next[dc];
  end
 end endgenerate
 nav3_frame_rx parser(.clk(rx_clk),.reset_n(rx_reset_n),.data(gmii_data),.valid(gmii_valid),.error(gmii_error),
  .lane_available(available),.lane_base(bases),.lane_bank(banks),.wr_en(wr_en),.wr_addr(wr_addr),.wr_data(wr_data),
  .publish(publish),.publish_lane(publish_lane),.publish_bank(publish_bank),.publish_bytes(publish_bytes),
  .publish_family(publish_family),.publish_opcode(publish_opcode),
  .cancel_pulse(cancel_pulse),.cancel_desc(cancel_wire),.bad_frames(bad_frames),.busy_frames(busy_frames));
 wire cancel_source_ready;
 nav3_async_mailbox #(.WIDTH(418),.CAPTURE_DST(1)) cancel_cross(.src_clk(rx_clk),.src_reset_n(rx_reset_n),
  .src_valid(cancel_pulse),.src_data(cancel_wire),.src_ready(cancel_source_ready),
  .dst_clk(cpu_clk),.dst_reset_n(cpu_reset_n),.dst_ready(cancel_ready),.dst_valid(cancel_valid),.dst_data(cancel_desc));

 reg[3:0]cmd_send,resp_send,pub_send;
 reg[32:0]cmd_data[0:3];reg[43:0]resp_data[0:3];
 wire[32:0]rx_command[0:3];wire[43:0]cpu_response[0:3];wire[51:0]cpu_publish[0:3];
 genvar g;
 generate for(g=0;g<4;g=g+1)begin:lane_cross
  nav3_async_mailbox #(.WIDTH(33)) command(.src_clk(cpu_clk),.src_reset_n(cpu_reset_n),
   .src_valid(cmd_send[g]),.src_data(cmd_data[g]),.src_ready(cmd_ready[g]),
   .dst_clk(rx_clk),.dst_reset_n(rx_reset_n),.dst_ready(resp_ready[g]),.dst_valid(cmd_valid[g]),.dst_data(rx_command[g]));
  nav3_async_mailbox #(.WIDTH(44)) response(.src_clk(rx_clk),.src_reset_n(rx_reset_n),
   .src_valid(resp_send[g]),.src_data(resp_data[g]),.src_ready(resp_ready[g]),
   .dst_clk(cpu_clk),.dst_reset_n(cpu_reset_n),.dst_ready(1'b1),.dst_valid(resp_valid[g]),.dst_data(cpu_response[g]));
  nav3_async_mailbox #(.WIDTH(52)) published(.src_clk(rx_clk),.src_reset_n(rx_reset_n),
   .src_valid(pub_send[g]),.src_data(pub_data[g]),.src_ready(pub_ready[g]),
   .dst_clk(cpu_clk),.dst_reset_n(cpu_reset_n),.dst_ready(1'b1),.dst_valid(pub_valid[g]),.dst_data(cpu_publish[g]));
 end endgenerate
 integer ri;
 always @(posedge rx_clk or negedge rx_reset_n)begin
  if(!rx_reset_n)begin
   rx_pinned<=0;rx_latest<=0;resp_send<=0;pub_send<=0;
   for(ri=0;ri<4;ri=ri+1)begin
    rx_pin_bank[ri]<=0;rx_latest_bank[ri]<=0;rx_pin_token[ri]<=0;
    resp_data[ri]<=0;
   end
   superseded[0]<=0; // Ordinary lane is pinned, never superseded.
   for(ri=0;ri<12;ri=ri+1)bank_meta[ri]<=0;
  end else begin
   resp_send<=0;pub_send<=0;
   if(publish)begin
    bank_meta[slot(publish_lane,publish_bank)]<={1'b1,publish_family};
    pub_send[publish_lane]<=1;
    if(publish_lane==0)begin rx_pinned[0]<=1;rx_pin_bank[0]<=0;rx_pin_token[0]<=rx_serial_next[0];end
    else begin
     rx_latest[publish_lane]<=1;rx_latest_bank[publish_lane]<=publish_bank;
    end
   end
   for(ri=0;ri<4;ri=ri+1)if(cmd_valid[ri]&&resp_ready[ri])begin
    resp_send[ri]<=1;
    if(rx_command[ri][32])begin
     if(rx_pinned[ri]&&rx_pin_token[ri]==rx_command[ri][31:0])begin
      rx_pinned[ri]<=0;resp_data[ri]<=0;
     end else resp_data[ri]<={rx_pinned[ri],rx_pin_bank[ri],response_bytes(ri[1:0],bank_meta[slot(ri[1:0],rx_pin_bank[ri])]),rx_pin_token[ri]};
    end else if(rx_pinned[ri])begin
     resp_data[ri]<={1'b1,rx_pin_bank[ri],response_bytes(ri[1:0],bank_meta[slot(ri[1:0],rx_pin_bank[ri])]),rx_pin_token[ri]};
    end else if(publish&&publish_lane==ri)begin
     rx_pinned[ri]<=1;rx_pin_bank[ri]<=publish_bank;rx_pin_token[ri]<=rx_serial_next[ri];rx_latest[ri]<=0;
     resp_data[ri]<={1'b1,publish_bank,publish_bytes,rx_serial_next[ri]};
    end else if(rx_latest[ri])begin
     rx_pinned[ri]<=1;rx_pin_bank[ri]<=rx_latest_bank[ri];rx_pin_token[ri]<=rx_latest_token[ri];rx_latest[ri]<=0;
     resp_data[ri]<={1'b1,rx_latest_bank[ri],response_bytes(ri[1:0],bank_meta[slot(ri[1:0],rx_latest_bank[ri])]),rx_latest_token[ri]};
    end else resp_data[ri]<=0;
   end
  end
 end
 // Binary diagnostic counters cross as registered Gray buses, never authority.
 reg[31:0]bad_gray,busy_gray,pose_gray,permit_gray;
 (* syn_async_reg=1 *) reg[31:0]bad_s0,bad_s1,busy_s0,busy_s1,pose_s0,pose_s1,permit_s0,permit_s1;
 (* syn_async_reg=1 *) reg[3:0]latest_s0,latest_s1;
 always @(posedge rx_clk)begin bad_gray<=bad_frames^(bad_frames>>1);busy_gray<=busy_frames^(busy_frames>>1);
  pose_gray<=superseded[1]^(superseded[1]>>1);permit_gray<=superseded[2]^(superseded[2]>>1);end
 function[31:0]ungray;input[31:0]v;integer k;begin ungray[31]=v[31];for(k=30;k>=0;k=k-1)ungray[k]=ungray[k+1]^v[k];end endfunction
 reg[3:0]cpu_pinned,acquire_busy,replied_empty;
 reg[1:0]cpu_bank[0:3];reg[31:0]cpu_token[0:3],cpu_stamp[0:11],cpu_pub_token[0:11],accepted_count[0:3];
 reg[8:0]cpu_bytes[0:3];reg[8:0]ordinary_summary;
 generate for(g=0;g<4;g=g+1)begin:pin_projection
  assign pin_tokens[g*32 +:32]=cpu_pinned[g]?cpu_token[g]:32'd0;
  assign pin_rx_ms[g*32 +:32]=cpu_pinned[g]?cpu_stamp[g*3+cpu_bank[g]]:32'd0;
  assign pin_flags[g*4 +:4]={replied_empty[g],acquire_busy[g],latest_s1[g],cpu_pinned[g]};
  assign pin_bytes[g*9 +:9]=cpu_pinned[g]?cpu_bytes[g]:9'd0;
 end endgenerate
 reg read_busy,read_source,body_pending;reg[1:0]body_lane;reg[5:0]body_word;
 wire read_source_now=source_req&&!read_busy;
 wire read_body_now=body_pending&&!read_busy&&!source_req;
 wire read_request=read_source_now||read_body_now;
 wire[1:0]read_lane=read_source_now?source_lane:body_lane;
 wire[5:0]read_word=read_source_now?source_word:body_word;
 wire read_legal=cpu_pinned[read_lane]&&({1'b0,read_word,2'b00}<cpu_bytes[read_lane]);
 wire[8:0]read_address=physical(read_lane,cpu_bank[read_lane])+{3'd0,read_word};
 wire[31:0]ram_data;
 reg read_was_legal;
 assign source_grant=read_source_now;
 nav3_packet_ram #(.WORDS(400)) packet_memory(.wr_clk(rx_clk),.wr_en(wr_en),.wr_addr(wr_addr),.wr_data(wr_data),
  .rd_clk(cpu_clk),.rd_en(read_request&&read_legal),.rd_addr(read_address),.rd_data(ram_data));
 integer ci;reg[1:0]bus_lane;reg[11:0]body_offset;reg body_address;
 always @*begin
  bus_lane=0;body_offset=0;body_address=0;
  if(bus_offset>=12'h500&&bus_offset<12'h600)begin body_address=1;bus_lane=0;body_offset=bus_offset-12'h500;end
  else if(bus_offset>=12'h700&&bus_offset<12'h7a0)begin body_address=1;bus_lane=1;body_offset=bus_offset-12'h700;end
  else if(bus_offset>=12'h7a0&&bus_offset<12'h840)begin body_address=1;bus_lane=2;body_offset=bus_offset-12'h7a0;end
  else if(bus_offset>=12'h840&&bus_offset<12'h8c0)begin body_address=1;bus_lane=3;body_offset=bus_offset-12'h840;end
 end
 task command_lane;input integer n;input release_pin;begin
  if(!acquire_busy[n]&&cmd_ready[n]&&(release_pin?(cpu_pinned[n]&&bus_wdata==cpu_token[n]):(!cpu_pinned[n]&&bus_wdata==1)))begin
   cmd_data[n]<={release_pin,bus_wdata};cmd_send[n]<=1;acquire_busy[n]<=1;replied_empty[n]<=0;
   if(release_pin)begin cpu_pinned[n]<=0;cpu_token[n]<=0;end
  end
 end endtask
 always @(posedge cpu_clk or negedge cpu_reset_n)begin
  if(!cpu_reset_n)begin
   cmd_send<=0;cpu_pinned<=0;acquire_busy<=0;replied_empty<=0;ordinary_summary<=0;
   bus_rsp_valid<=0;bus_rsp_data<=0;source_valid<=0;source_data<=0;read_busy<=0;read_source<=0;read_was_legal<=0;body_pending<=0;body_lane<=0;body_word<=0;
   bad_s0<=0;bad_s1<=0;busy_s0<=0;busy_s1<=0;pose_s0<=0;pose_s1<=0;permit_s0<=0;permit_s1<=0;latest_s0<=0;latest_s1<=0;
   for(ci=0;ci<4;ci=ci+1)begin cpu_bank[ci]<=0;cpu_token[ci]<=0;cpu_bytes[ci]<=0;cmd_data[ci]<=0;accepted_count[ci]<=0;end
   for(ci=0;ci<12;ci=ci+1)begin cpu_stamp[ci]<=0;cpu_pub_token[ci]<=0;end
  end else begin
   bad_s0<=bad_gray;bad_s1<=bad_s0;busy_s0<=busy_gray;busy_s1<=busy_s0;pose_s0<=pose_gray;pose_s1<=pose_s0;permit_s0<=permit_gray;permit_s1<=permit_s0;
   latest_s0<=rx_latest;latest_s1<=latest_s0;
   cmd_send<=0;bus_rsp_valid<=0;source_valid<=0;
   for(ci=0;ci<4;ci=ci+1)begin
    if(pub_valid[ci])begin
     accepted_count[ci]<=accepted_count[ci]+1'b1;
     cpu_stamp[slot(ci[1:0],cpu_publish[ci][10:9])]<=device_ms;
     cpu_pub_token[slot(ci[1:0],cpu_publish[ci][10:9])]<=cpu_publish[ci][42:11];
     if(ci==0)begin cpu_pinned[0]<=1;cpu_bank[0]<=0;cpu_token[0]<=cpu_publish[0][42:11];cpu_bytes[0]<=cpu_publish[0][8:0];ordinary_summary<=cpu_publish[0][51:43];end
    end
    if(resp_valid[ci])begin
     acquire_busy[ci]<=0;cpu_pinned[ci]<=cpu_response[ci][43];replied_empty[ci]<=!cpu_response[ci][43];
     cpu_bank[ci]<=cpu_response[ci][42:41];cpu_bytes[ci]<=cpu_response[ci][40:32];cpu_token[ci]<=cpu_response[ci][31:0];
    end
   end
   if(read_busy)begin
    read_busy<=0;
    if(read_source)begin source_data<=read_was_legal?ram_data:32'd0;source_valid<=1;end
    else begin bus_rsp_data<=read_was_legal?ram_data:32'd0;bus_rsp_valid<=1;end
   end
   if(read_request)begin
    read_busy<=1;read_source<=read_source_now;read_was_legal<=read_legal;
    if(read_body_now)body_pending<=0;
   end
   if(bus_req)begin
    if(body_address&&bus_wstrb==0)begin body_pending<=1;body_lane<=bus_lane;body_word<=body_offset[7:2];end
    else begin
     bus_rsp_valid<=1;bus_rsp_data<=0;
     if(bus_wstrb==0)case(bus_offset)
      12'h424:bus_rsp_data<=cpu_pinned[0]?{15'd0,ordinary_summary,7'd0,1'b1}:32'd0;
      12'h428:bus_rsp_data<=pin_tokens[31:0];12'h42c:bus_rsp_data<=pin_rx_ms[31:0];
      12'h444:bus_rsp_data<=pin_tokens[63:32];12'h448:bus_rsp_data<=pin_rx_ms[63:32];
      12'h44c:bus_rsp_data<={28'd0,pin_flags[7:4]};12'h454:bus_rsp_data<=accepted_count[1];
      12'h45c:bus_rsp_data<=pin_tokens[95:64];12'h460:bus_rsp_data<=pin_rx_ms[95:64];
      12'h464:bus_rsp_data<={28'd0,pin_flags[11:8]};12'h46c:bus_rsp_data<=accepted_count[2];
      12'h470:bus_rsp_data<={31'd0,(cpu_pinned[3]||latest_s1[3]||acquire_busy[3])};
      12'h474:bus_rsp_data<=pin_tokens[127:96];12'h478:bus_rsp_data<=pin_rx_ms[127:96];
      12'h47c:bus_rsp_data<=cpu_pinned[3]?(cpu_bytes[3]==112?32'd2:32'd1):32'd0;
      12'h488:bus_rsp_data<={23'd0,cpu_bytes[3]};12'h498:bus_rsp_data<=ungray(busy_s1);
      12'h49c:bus_rsp_data<=ungray(pose_s1);12'h4a0:bus_rsp_data<=ungray(permit_s1);12'h4a8:bus_rsp_data<=ungray(bad_s1);
      default:bus_rsp_data<=0;
     endcase
     else if(bus_wstrb==15)case(bus_offset)
      12'h430:command_lane(0,1);12'h440:command_lane(1,0);12'h450:command_lane(1,1);
      12'h458:command_lane(2,0);12'h468:command_lane(2,1);12'h480:command_lane(3,0);12'h484:command_lane(3,1);
      default:begin end
     endcase
    end
   end
  end
 end
endmodule
