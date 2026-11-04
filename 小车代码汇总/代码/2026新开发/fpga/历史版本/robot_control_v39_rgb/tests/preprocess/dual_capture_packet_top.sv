// Integration harness: real image/CDC/packet RTL, only physical sensors and
// external control-traffic ownership are driven by the C++ testbench.
module dual_capture_packet_top(
 input tx_clk,cam0_clk,cam1_clk,rst_n,
 input href0,vsync0,input [7:0] raw0,
 input href1,vsync1,input [7:0] raw1,
 input session_commit,input [15:0] requested_epoch,input tx_allow,
 output tx_busy,tx_pending,gmii_valid,output [7:0] gmii_data,
 output [10:0] used0,used1,output full0,full1,
 output [31:0] dropped0,dropped1,completed0,completed1,
 output [31:0] packets,pairs_started,pair_fault_count,
 output [15:0] epoch,bridge_epoch0,bridge_epoch1,
 output [31:0] source_seq0,source_seq1,
 output source_sof0,source_sof1
);
 reg [15:0] live_epoch;reg commit;reg [2:0] reset_count;
 wire ready0,ready1,valid0,valid1;wire [511:0] data0,data1;
 assign epoch=live_epoch;
 wire fifo_reset=!rst_n||reset_count!=0;
 always @(posedge tx_clk)begin
  if(!rst_n)begin live_epoch<=0;commit<=0;reset_count<=4;end
  else begin
   commit<=0;if(reset_count!=0)reset_count<=reset_count-1'b1;
   if(session_commit&&requested_epoch!=0&&requested_epoch!=live_epoch&&
       !tx_busy&&ready0&&ready1&&reset_count==0)begin
    live_epoch<=requested_epoch;commit<=1;reset_count<=4;
   end
  end
 end
 imu_snapshot_cdc #(.CAPTURE_WHEN_IDLE(1)) mailbox0(
  .src_clk(tx_clk),.src_rst_n(rst_n),.src_valid(commit),.src_data({496'd0,live_epoch}),.src_ready(ready0),
  .dst_clk(cam0_clk),.dst_rst_n(rst_n),.dst_ready(1'b1),.dst_valid(valid0),.dst_data(data0));
 imu_snapshot_cdc #(.CAPTURE_WHEN_IDLE(1)) mailbox1(
  .src_clk(tx_clk),.src_rst_n(rst_n),.src_valid(commit),.src_data({496'd0,live_epoch}),.src_ready(ready1),
  .dst_clk(cam1_clk),.dst_rst_n(rst_n),.dst_ready(1'b1),.dst_valid(valid1),.dst_data(data1));
 reg [16:0] div_ms;reg [31:0] uptime,uptime_gray;
 always @(posedge tx_clk)begin
  if(!rst_n)begin div_ms<=0;uptime<=1000;uptime_gray<=1000^(1000>>1);end
  else if(div_ms==124999)begin
   div_ms<=0;uptime<=uptime+1'b1;uptime_gray<=(uptime+1'b1)^((uptime+1'b1)>>1);
  end else div_ms<=div_ms+1'b1;
 end
 wire w0,w1,pop0,pop1,empty0,empty1,fault0,fault1,pair_fault;
 wire [35:0] wd0,wd1,rd0,rd1;
 rgb_capture_bridge bridge0(.clk(cam0_clk),.rst_n(rst_n),.init_done(1'b1),
  .href(href0),.vsync(vsync0),.raw(raw0),.session_valid(valid0),.session_epoch(data0[15:0]),
  .uptime_gray(uptime_gray),.fifo_full(full0||used0>=2044),.fifo_write(w0),.fifo_word(wd0),
  .fault(fault0),.completed_frames(completed0),.dropped_frames(dropped0));
 rgb_capture_bridge bridge1(.clk(cam1_clk),.rst_n(rst_n),.init_done(1'b1),
  .href(href1),.vsync(vsync1),.raw(raw1),.session_valid(valid1),.session_epoch(data1[15:0]),
  .uptime_gray(uptime_gray),.fifo_full(full1||used1>=2044),.fifo_write(w1),.fifo_word(wd1),
  .fault(fault1),.completed_frames(completed1),.dropped_frames(dropped1));
 rgb_stream_fifo fifo0(.wrclk(cam0_clk),.rdclk(tx_clk),.aclr(fifo_reset),.wrreq(w0),.rdreq(pop0),
  .wrdata(wd0),.rddata(rd0),.rdusedw(),.wrusedw(used0),.empty(empty0),.full(full0));
 rgb_stream_fifo fifo1(.wrclk(cam1_clk),.rdclk(tx_clk),.aclr(fifo_reset),.wrreq(w1),.rdreq(pop1),
  .wrdata(wd1),.rddata(rd1),.rdusedw(),.wrusedw(used1),.empty(empty1),.full(full1));
 rgb_packet_tx sender(.clk(tx_clk),.rst_n(rst_n),.epoch(live_epoch),.tx_allow(tx_allow),
  .tx_busy(tx_busy),.tx_pending(tx_pending),.gmii_valid(gmii_valid),.gmii_data(gmii_data),
  .fifo0_data(rd0),.fifo0_empty(empty0),.fifo0_pop(pop0),.fifo1_data(rd1),.fifo1_empty(empty1),.fifo1_pop(pop1),
  .pair_fault(pair_fault),.pair_fault_count(pair_fault_count),.packets_sent(packets),.pairs_started(pairs_started));
 assign source_seq0=bridge0.frame_sequence;assign source_seq1=bridge1.frame_sequence;
 assign bridge_epoch0=bridge0.epoch;assign bridge_epoch1=bridge1.epoch;
 assign source_sof0=bridge0.raw_sof&&!bridge0.new_session;
 assign source_sof1=bridge1.raw_sof&&!bridge1.new_session;
endmodule
