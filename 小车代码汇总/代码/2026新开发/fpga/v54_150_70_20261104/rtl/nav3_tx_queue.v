`timescale 1ns/1ps
// Five class-separated immutable slots, oneCPU write/oneTX read BSRAM.
module nav3_tx_queue #(
 parameter integer TX_HZ=125000000
)(
 input wire cpu_clk,cpu_reset_n,tx_clk,tx_reset_n,
 input wire bus_req,input wire [11:0] bus_offset,
 input wire [31:0] bus_wdata,input wire [3:0] bus_wstrb,
 output reg bus_rsp_valid,output reg [31:0] bus_rsp_data,
 output reg [31:0] last_tx_ticket,
 input wire tx_allow,output wire tx_pending,tx_busy,
 output wire gmii_valid,output wire [7:0] gmii_data
);
 localparam integer RATE_CYCLES=TX_HZ/1000;
 reg[4:0]slot_busy,slot_seen_busy,enqueue;
 reg[32:0]slot_meta[0:4];wire[32:0]network_meta[0:4];wire[4:0]enqueue_ready,network_valid,network_done;
 reg reserved,ticket_exhausted;reg[1:0]reserved_class;reg[2:0]reserved_slot;
 reg[31:0]next_ticket,reserved_ticket;reg[63:0]written;reg reserved_nva,magic_ok;
 reg[2:0]allocation;reg allocation_ok;
 integer i;
 always @*begin
  allocation=0;allocation_ok=1;
  case(bus_wdata)
   0:begin if(!slot_busy[0])allocation=0;else if(!slot_busy[1])allocation=1;else allocation_ok=0;end
   1:begin allocation=2;allocation_ok=!slot_busy[2];end
   2:begin allocation=3;allocation_ok=!slot_busy[3];end
   3:begin allocation=4;allocation_ok=!slot_busy[4];end
   default:allocation_ok=0;
  endcase
 end
 genvar g;
 generate for(g=0;g<5;g=g+1)begin:slot_cross
  nav3_async_mailbox #(.WIDTH(33))queue(.src_clk(cpu_clk),.src_reset_n(cpu_reset_n),
   .src_valid(enqueue[g]),.src_data(slot_meta[g]),.src_ready(enqueue_ready[g]),
   .dst_clk(tx_clk),.dst_reset_n(tx_reset_n),.dst_ready(network_done[g]),
   .dst_valid(network_valid[g]),.dst_data(network_meta[g]));
 end endgenerate
 wire body_address=bus_offset>=12'h600&&bus_offset<12'h700;
 wire body_write=bus_req&&body_address&&bus_wstrb==15&&reserved;
 wire[8:0]write_address={reserved_slot,6'd0}+{3'd0,bus_offset[7:2]};
 wire serializer_active,serializer_done,serializer_req;wire[8:0]serializer_address;
 // TX pacing only needs to count down from RATE_CYCLES-1.  Keep the
 // counter parameter-sized so a 1 kHz (125000-cycle) rate uses 17 bits
 // instead of a full 32-bit register; this path is outside safety/control.
 localparam integer RATE_COUNT_BITS=(RATE_CYCLES>0)?$clog2(RATE_CYCLES+1):1;
 reg[2:0]active_slot;reg[RATE_COUNT_BITS-1:0]rate_count;
 reg[2:0]selected;reg any_pending;
 // Tickets never wrap (allocation stops at ffffffff). Compare bytes in
 // parallel instead of a 32-bit carry chain in the 125MHz arbitration cone.
 wire[31:0]ticket0=network_meta[0][31:0],ticket1=network_meta[1][31:0];
 wire[3:0]ticket_lt,ticket_eq;
 generate for(g=0;g<4;g=g+1)begin:ticket_compare
  assign ticket_lt[g]=ticket0[g*8+:8]<ticket1[g*8+:8];
  assign ticket_eq[g]=ticket0[g*8+:8]==ticket1[g*8+:8];
 end endgenerate
 wire ticket0_first=ticket_lt[3]||(ticket_eq[3]&&ticket_lt[2])||
  ((&ticket_eq[3:2])&&ticket_lt[1])||((&ticket_eq[3:1])&&ticket_lt[0]);
 always @*begin
  selected=0;any_pending=1;
  if(network_valid[0]&&network_valid[1])selected=ticket0_first?0:1;
  else if(network_valid[0])selected=0;
  else if(network_valid[1])selected=1;
  else if(network_valid[2])selected=2;
  else if(network_valid[3])selected=3;
  else if(network_valid[4])selected=4;
  else any_pending=0;
 end
 assign tx_pending=any_pending&&!serializer_active&&!serializer_done&&rate_count==0;
 wire launch=tx_pending&&tx_allow;
 assign tx_busy=serializer_active;
 generate for(g=0;g<5;g=g+1)begin:completion
  assign network_done[g]=serializer_done&&active_slot==g;
 end endgenerate
 always @(posedge tx_clk or negedge tx_reset_n)begin
  if(!tx_reset_n)begin active_slot<=0;rate_count<=0;end
  else begin
   if(rate_count!=0)rate_count<=rate_count-1'b1;
   if(launch)begin active_slot<=selected;rate_count<=RATE_CYCLES>0?RATE_CYCLES-1:0;end
  end
 end
 wire[31:0]ram_data;
 nav3_serializer serializer(.clk(tx_clk),.reset_n(tx_reset_n),.start(launch),.slot(selected),
  .nva(network_meta[selected][32]),.ram_req(serializer_req),.ram_addr(serializer_address),.ram_data(ram_data),
  .active(serializer_active),.done(serializer_done),.gmii_valid(gmii_valid),.gmii_data(gmii_data));
 // Readback uses otherwise-free TX read cycles, not a third memory port.
 reg read_send,response_send;reg[8:0]read_address;reg[31:0]response_word;
 wire read_ready,read_valid,response_ready,response_valid;wire[8:0]network_read_address;wire[31:0]cpu_read_data;
 reg read_pipe;
 wire read_grant=read_valid&&!serializer_req&&!read_pipe&&response_ready;
 nav3_async_mailbox #(.WIDTH(9))read_cross(.src_clk(cpu_clk),.src_reset_n(cpu_reset_n),
  .src_valid(read_send),.src_data(read_address),.src_ready(read_ready),
  .dst_clk(tx_clk),.dst_reset_n(tx_reset_n),.dst_ready(read_grant),.dst_valid(read_valid),.dst_data(network_read_address));
 nav3_async_mailbox #(.WIDTH(32))read_response(.src_clk(tx_clk),.src_reset_n(tx_reset_n),
  .src_valid(response_send),.src_data(response_word),.src_ready(response_ready),
  .dst_clk(cpu_clk),.dst_reset_n(cpu_reset_n),.dst_ready(1'b1),.dst_valid(response_valid),.dst_data(cpu_read_data));
 nav3_packet_ram #(.WORDS(320))packet_memory(.wr_clk(cpu_clk),.wr_en(body_write),.wr_addr(write_address),.wr_data(bus_wdata),
  .rd_clk(tx_clk),.rd_en(serializer_req||read_grant),.rd_addr(serializer_req?serializer_address:network_read_address),.rd_data(ram_data));
 always @(posedge tx_clk or negedge tx_reset_n)begin
  if(!tx_reset_n)begin read_pipe<=0;response_send<=0;response_word<=0;end
  else begin
   response_send<=0;read_pipe<=read_grant;
   if(read_pipe)begin response_word<=ram_data;response_send<=1;end
  end
 end
 always @(posedge cpu_clk or negedge cpu_reset_n)begin
  if(!cpu_reset_n)begin
   slot_busy<=0;slot_seen_busy<=0;enqueue<=0;reserved<=0;ticket_exhausted<=0;reserved_class<=0;reserved_slot<=0;
   next_ticket<=1;reserved_ticket<=0;written<=0;reserved_nva<=0;magic_ok<=0;last_tx_ticket<=0;
   bus_rsp_valid<=0;bus_rsp_data<=0;read_send<=0;read_address<=0;
   for(i=0;i<5;i=i+1)slot_meta[i]<=0;
  end else begin
   enqueue<=0;read_send<=0;bus_rsp_valid<=0;bus_rsp_data<=0;
   for(i=0;i<5;i=i+1)if(slot_busy[i])begin
    if(!enqueue_ready[i])slot_seen_busy[i]<=1;
    if(slot_seen_busy[i]&&enqueue_ready[i])begin slot_busy[i]<=0;slot_seen_busy[i]<=0;last_tx_ticket<=slot_meta[i][31:0];end
   end
   if(response_valid)begin bus_rsp_valid<=1;bus_rsp_data<=cpu_read_data;end
   if(bus_req)begin
    bus_rsp_valid<=1;
    if(bus_wstrb==0)begin
     case(bus_offset)
      12'h434:bus_rsp_data<={31'd0,reserved};
      12'h438:bus_rsp_data<=reserved?reserved_ticket:next_ticket;
      12'h490:bus_rsp_data<={30'd0,reserved_class};
      12'h494:bus_rsp_data<=256;
      default:if(body_address&&reserved&&written[bus_offset[7:2]]&&read_ready)begin
       read_address<=write_address;read_send<=1;bus_rsp_valid<=0;
      end
     endcase
    end else if(bus_wstrb!=15)bus_rsp_data<=3;
    else case(bus_offset)
     12'h490:begin
      if(reserved)begin if(bus_wdata!={30'd0,reserved_class})bus_rsp_data<=2;end
      else if(ticket_exhausted)bus_rsp_data<=5;
      else if(allocation_ok)begin reserved<=1;reserved_class<=bus_wdata[1:0];reserved_slot<=allocation;reserved_ticket<=next_ticket;written<=0;magic_ok<=0;end
      else bus_rsp_data<=bus_wdata>3?12:2;
     end
     12'h494:if(bus_wdata!=256)bus_rsp_data<=12;
     12'h43c:begin
      if(!reserved||bus_wdata!=reserved_ticket)bus_rsp_data<=5;
      else if(written!=64'hffffffffffffffff)bus_rsp_data<=2;
      else if(!magic_ok)bus_rsp_data<=6;
      else begin
       enqueue[reserved_slot]<=1;slot_meta[reserved_slot]<={reserved_nva,reserved_ticket};slot_busy[reserved_slot]<=1;slot_seen_busy[reserved_slot]<=0;
       reserved<=0;written<=0;
       if(next_ticket==32'hffffffff)ticket_exhausted<=1;else next_ticket<=next_ticket+1'b1;
      end
     end
     default:if(body_address)begin
      if(!reserved)bus_rsp_data<=2;
      else begin
       written[bus_offset[7:2]]<=1;
       if(bus_offset[7:2]==0)begin magic_ok<=bus_wdata==32'h4e415633||bus_wdata==32'h4e564131;reserved_nva<=bus_wdata==32'h4e564131;end
      end
     end
    endcase
   end
  end
 end
endmodule
