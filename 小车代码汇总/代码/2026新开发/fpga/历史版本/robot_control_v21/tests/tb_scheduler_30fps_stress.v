`timescale 1ns/1ps
module eth_dcfifo(input [7:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [12:0] Wnum,Rnum,output [7:0] Q,output Empty,Full);
 reg [7:0] mem[0:8191]; integer wp=0,rp=0;
 assign Wnum=wp-rp; assign Rnum=wp-rp; assign Q=mem[rp%8192];
 assign Empty=(wp==rp); assign Full=(wp-rp>=4096);
 always @(posedge WrClk or posedge Reset)if(Reset)wp<=0;else if(WrEn)begin if(Full)$fatal(1,"FIFO overflow");mem[wp%8192]<=Data;wp<=wp+1;end
 always @(posedge RdClk or posedge Reset)if(Reset)rp<=0;else if(RdEn)begin if(Empty)$fatal(1,"FIFO underflow");rp<=rp+1;end
endmodule
module tb_scheduler_30fps_stress;
reg clk=0,src_clk=0,rst_n=0,wr=0;reg[7:0]data=0;
always #4 clk=~clk;always #10 src_clk=~src_clk;
wire cam_busy,cam_valid,cam_allow,imu_ready,imu_allow,imu_valid,dv,ack_pending,ack_busy,ack_valid,ack_allow;
wire[7:0]cam_data,imu_data,ack_data,out_data;
reg imu_publish=0,ack_publish=0;reg[511:0]imu_payload=0;reg[127:0]ack_payload=0;
wire[511:0]imu_wire;wire imu_pending;
UDP_Send camera(.tx_allow(cam_allow),.tx_busy(cam_busy),.Clk(clk),.Rst_n(rst_n),.Tx_Done(),.des_mac(48'hffffffffffff),.src_mac(48'h000a3501fec0),.des_port(16'd6102),.src_port(16'd5000),.des_ip(32'hc0a80003),.src_ip(32'hc0a80002),.data_length(16'd1922),.GMII_GTXC(clk),.GMII_TXD(cam_data),.GMII_TXEN(cam_valid),.wrreq(wr),.wrdata(data),.wrclk(clk),.aclr(!rst_n),.wrusedw());
imu_snapshot_cdc icdc(.src_clk(src_clk),.src_rst_n(rst_n),.src_valid(imu_publish),.src_data(imu_payload),.src_ready(),.dst_clk(clk),.dst_rst_n(rst_n),.dst_ready(imu_ready&&imu_allow),.dst_valid(imu_pending),.dst_data(imu_wire));
imu_ethernet_tx imu(.clk(clk),.rst_n(rst_n),.valid(imu_pending&&imu_allow),.payload(imu_wire),.ready(imu_ready),.gmii_data(imu_data),.gmii_valid(imu_valid));
motor_status_gmii ack(.source_clk(src_clk),.source_rst_n(rst_n),.status_valid(ack_publish),.status_data(ack_payload),.tx_clk(clk),.tx_rst_n(rst_n),.tx_allow(ack_allow),.tx_pending(ack_pending),.tx_busy(ack_busy),.gmii_valid(ack_valid),.gmii_data(ack_data));
gmii_three_source_share share(.camera_busy(cam_busy),.camera_valid(cam_valid),.camera_data(cam_data),.imu_pending(imu_pending),.imu_busy(!imu_ready),.imu_valid(imu_valid),.imu_data(imu_data),.motor_pending(ack_pending),.motor_busy(ack_busy),.motor_valid(ack_valid),.motor_data(ack_data),.camera_allow(cam_allow),.imu_allow(imu_allow),.motor_allow(ack_allow),.gmii_data(out_data),.gmii_valid(dv));
integer camera_count=0,imu_count=0,ack_count=0,gap=999,count=0,owner,active_owner;
integer imu_wait=0,ack_wait=0,max_imu_wait=0,max_ack_wait=0;
reg previous=0;
always @(negedge clk)if(rst_n)begin
 if((cam_valid&&imu_valid)||(cam_valid&&ack_valid)||(imu_valid&&ack_valid))$fatal(1,"overlapping frames");
 if(imu_pending&&!imu_valid)imu_wait=imu_wait+1;else begin if(imu_wait>max_imu_wait)max_imu_wait=imu_wait;imu_wait=0;end
 if(ack_pending&&!ack_valid)ack_wait=ack_wait+1;else begin if(ack_wait>max_ack_wait)max_ack_wait=ack_wait;ack_wait=0;end
 owner=ack_valid?2:imu_valid?1:0;
 if(dv)begin
  if(!previous)begin if(gap<12)$fatal(1,"IFG below12");active_owner=owner;end
  if(active_owner!=owner)$fatal(1,"frame interrupted");count=count+1;gap=0;
 end else begin
  gap=gap+1;
  if(previous)begin
   if(active_owner==0)begin if(count!=1976)$fatal(1,"camera length");camera_count=camera_count+1;end
   if(active_owner==1)begin if(count!=90)$fatal(1,"IMU length");imu_count=imu_count+1;end
   if(active_owner==2)begin if(count!=72)$fatal(1,"ACK length");ack_count=ack_count+1;end
   count=0;
  end
 end
 previous=dv;
end
integer f,row,j;
initial begin
 #31;rst_n=1;
 for(f=0;f<3;f=f+1)for(row=0;row<1120;row=row+1)begin
  for(j=0;j<3714;j=j+1)begin @(negedge clk);wr=row<1080&&j<1922;data=j==0?row[7:0]:j==1?row[15:8]:(j+row)%251;end
 end
 @(negedge clk);wr=0;repeat(3000)@(negedge clk);
 if(camera_count!=3240||imu_count<19||ack_count<2)$fatal(1,"unexpected totals cam=%0d imu=%0d ack=%0d",camera_count,imu_count,ack_count);
 if(max_imu_wait>2300||max_ack_wait>2100)$fatal(1,"starvation imu=%0d ack=%0d",max_imu_wait,max_ack_wait);
 $display("PASS sustained nominal30FPS: camera_rows=%0d (3 complete frames), IMU=%0d ACK=%0d, max_pending_wait IMU=%0d/%0dns ACK=%0d/%0dns, wire lengths1976/90/72, IFG>=12, no collision or FIFO overflow",camera_count,imu_count,ack_count,max_imu_wait,max_imu_wait*8,max_ack_wait,max_ack_wait*8);$finish;
end
integer cycles=0;
always @(negedge src_clk)if(rst_n)begin
 imu_publish=0;ack_publish=0;
 if(cycles%250000==0)begin imu_publish=1;imu_payload={32'h494d5539,480'b0}|cycles;end
 if(cycles%2500000==0)begin ack_publish=1;ack_payload={32'h41434b31,96'b0}|cycles;end
 cycles=cycles+1;
end
initial begin #110000000;$fatal(1,"timeout");end
endmodule
