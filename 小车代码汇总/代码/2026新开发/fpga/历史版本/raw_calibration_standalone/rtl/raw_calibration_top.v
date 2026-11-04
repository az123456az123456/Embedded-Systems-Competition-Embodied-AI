`timescale 1ns/1ps
module raw_calibration_top(
 input clk50m,
 output [3:0] led,motor_pwm,motor_in1,motor_in2,
 output uart_tx,
 output rgmii_tx_clk,rgmii_tx_ctl,output [3:0] rgmii_txd,
 output cam1_pwdn,cam1_rst_n,cam1_xclk,cam1_scl,inout cam1_sda,
 input cam1_pclk,cam1_href,cam1_vsync,input [7:0] cam1_data,
 output cam2_pwdn,cam2_rst_n,cam2_xclk,cam2_scl,inout cam2_sda,
 input cam2_pclk,cam2_href,cam2_vsync,input [7:0] cam2_data
);
 assign motor_pwm=0;assign motor_in1=0;assign motor_in2=0;assign uart_tx=1;
 reg [20:0] startup=0;always @(posedge clk50m)if(!startup[20])startup<=startup+1'b1;
 wire tx_clk,cfg_clk,xclk,pll_locked;wire async_reset_n=pll_locked && startup[20];
 Gowin_PLL pll(.clkin(clk50m),.clkout0(tx_clk),.clkout1(cfg_clk),.clkout2(xclk),.lock(pll_locked),.mdclk(clk50m),.reset(!startup[20]));
 wire cfg_reset_n,tx_reset_n,p1_reset_n,p2_reset_n;
 camera_reset_release cfg_reset(.clk(cfg_clk),.async_reset_n(async_reset_n),.reset_n(cfg_reset_n));
 camera_reset_release tx_reset(.clk(tx_clk),.async_reset_n(async_reset_n),.reset_n(tx_reset_n));
 camera_reset_release p1_reset(.clk(cam1_pclk),.async_reset_n(async_reset_n),.reset_n(p1_reset_n));
 camera_reset_release p2_reset(.clk(cam2_pclk),.async_reset_n(async_reset_n),.reset_n(p2_reset_n));
 assign cam1_pwdn=0;assign cam2_pwdn=0;assign cam1_rst_n=cfg_reset_n;assign cam2_rst_n=cfg_reset_n;
 assign cam1_xclk=xclk;assign cam2_xclk=xclk;
 wire wr0,rd0,done0,nack0,wr1,rd1,done1,nack1;
 wire [15:0] addr0,addr1,failed0,failed1;
 wire [7:0] data0,data1,read0,read1,error0,error1,stage0,stage1;
 wire locked0,locked1;wire [511:0] snapshot0,snapshot1;
 raw_camera_setup setup0(.clk(cfg_clk),.rst_n(cfg_reset_n),.rddata(read0),.done(done0),.nack(nack0),
  .wr(wr0),.rd(rd0),.addr(addr0),.data(data0),.locked(locked0),.error(error0),.stage(stage0),.failed_addr(failed0),.snapshot(snapshot0));
 raw_camera_setup setup1(.clk(cfg_clk),.rst_n(cfg_reset_n),.rddata(read1),.done(done1),.nack(nack1),
  .wr(wr1),.rd(rd1),.addr(addr1),.data(data1),.locked(locked1),.error(error1),.stage(stage1),.failed_addr(failed1),.snapshot(snapshot1));
 i2c_control i2c0(.Clk(cfg_clk),.Rst_n(cfg_reset_n),.wrreg_req(wr0),.rdreg_req(rd0),.addr(addr0),.addr_mode(1'b1),.wrdata(data0),.rddata(read0),
  .device_id(8'h78),.RW_Done(done0),.ack(nack0),.dly_cnt_max(32'd0),.i2c_sclk(cam1_scl),.i2c_sdat(cam1_sda));
 i2c_control i2c1(.Clk(cfg_clk),.Rst_n(cfg_reset_n),.wrreg_req(wr1),.rdreg_req(rd1),.addr(addr1),.addr_mode(1'b1),.wrdata(data1),.rddata(read1),
  .device_id(8'h78),.RW_Done(done1),.ack(nack1),.dly_cnt_max(32'd0),.i2c_sclk(cam2_scl),.i2c_sdat(cam2_sda));
 // Both terminal snapshots are held for the remainder of this power/session.
 wire terminal=(locked0 || error0!=0) && (locked1 || error1!=0);
 wire [1215:0] cfg_profile={32'h43414c31,8'd1,{5'd0,terminal,locked1,locked0},16'd960,16'd1080,8'd0,8'd0,
  32'h00004101,stage0,error0,stage1,error1,failed0,failed1,snapshot0,snapshot1};
 (* syn_async_reg=1 *) reg [1:0] profile_sync=0;
 reg profile_captured;reg [1215:0] profile_saved;
 always @(posedge tx_clk or negedge tx_reset_n)begin
  if(!tx_reset_n)begin profile_sync<=0;profile_captured<=0;profile_saved<={32'h43414c31,8'd1,8'd0,16'd960,16'd1080,8'd0,8'd0,32'h00004101,1088'd0};end
  else begin
   profile_sync<={profile_sync[0],terminal};
   if(profile_sync[1] && !profile_captured)begin profile_saved<=cfg_profile;profile_captured<=1;end
  end
 end
 (* syn_async_reg=1 *) reg [1:0] ready1=0,ready2=0;
 always @(posedge cam1_pclk or negedge p1_reset_n)if(!p1_reset_n)ready1<=0;else ready1<={ready1[0],locked0 && locked1};
 always @(posedge cam2_pclk or negedge p2_reset_n)if(!p2_reset_n)ready2<=0;else ready2<={ready2[0],locked0 && locked1};
 wire clear1,clear2,write1,write2;wire [7:0] raw1,raw2;
 Camera_ETH_Formator format1(.PCLK(cam1_pclk),.Rst_n(p1_reset_n),.Init_Done(ready1[1]),.HREF(cam1_href),.VSYNC(cam1_vsync),.DATA(cam1_data),.fifo_aclr(clear1),.wrdata(raw1),.wrreq(write1));
 Camera_ETH_Formator format2(.PCLK(cam2_pclk),.Rst_n(p2_reset_n),.Init_Done(ready2[1]),.HREF(cam2_href),.VSYNC(cam2_vsync),.DATA(cam2_data),.fifo_aclr(clear2),.wrdata(raw2),.wrreq(write2));
 wire write_clk,clear_out,row_write,pair_fault;wire [7:0] row_data;
 controller_top rows(.profile_info(profile_saved),.pair_fault(pair_fault),.clk(tx_clk),.rst_n(tx_reset_n),
  .wrclk1(cam1_pclk),.fifo1_aclr(clear1),.wrdata1(raw1),.wrreq1(write1),
  .wrclk2(cam2_pclk),.fifo2_aclr(clear2),.wrdata2(raw2),.wrreq2(write2),
  .wrclk(write_clk),.fifo_aclr(clear_out),.fifo_wrreq(row_write),.fifo_wrdata(row_data));
 wire [7:0] tx_data;wire tx_valid;
 UDP_Send sender(.tx_allow(1'b1),.tx_busy(),.Clk(1'b0),.GMII_GTXC(tx_clk),.GMII_TXD(tx_data),.GMII_TXEN(tx_valid),.Rst_n(tx_reset_n),.Tx_Done(),
  .data_length(16'd1922),.des_ip(32'hc0a80003),.des_mac(48'hffffffffffff),.des_port(16'd6102),
  .src_ip(32'hc0a80002),.src_mac(48'h000a3501fec0),.src_port(16'd5000),.wrclk(write_clk),.wrdata(row_data),.wrreq(row_write),.aclr(clear_out),.wrusedw());
 gmii_to_rgmii serializer(.reset_n(tx_reset_n),.gmii_tx_clk(tx_clk),.gmii_txd(tx_data),.gmii_txen(tx_valid),.gmii_txer(1'b0),
  .rgmii_tx_clk(rgmii_tx_clk),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_tx_ctl));
 reg [24:0] beat=0;always @(posedge cfg_clk)beat<=beat+1'b1;
 assign led={error0!=0 || error1!=0,locked1,locked0,beat[24]};
endmodule
module camera_reset_release(input wire clk,async_reset_n,output wire reset_n);
 (* syn_async_reg=1 *)reg [1:0] release_sync=0;
 always @(posedge clk or negedge async_reset_n)if(!async_reset_n)release_sync<=0;else release_sync<={release_sync[0],1'b1};
 assign reset_n=release_sync[1];
endmodule
