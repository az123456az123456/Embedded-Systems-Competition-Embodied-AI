`timescale 1ns/1ps
// Behavioral FWFT model for encrypted vendor FIFO; production keeps Gowin IP.
module eth_dcfifo(input [7:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [12:0] Wnum,Rnum,output [7:0] Q,output Empty,Full);
 reg [7:0] mem[0:8191]; integer wp=0,rp=0;
 assign Wnum=wp-rp; assign Rnum=wp-rp; assign Q=mem[rp%8192];
 assign Empty=(wp==rp); assign Full=(wp-rp>=4096);
 always @(posedge WrClk or posedge Reset)
  if(Reset)wp<=0;else if(WrEn)begin if(Full)$fatal(1,"FIFO overflow");mem[wp%8192]<=Data;wp<=wp+1;end
 always @(posedge RdClk or posedge Reset)
  if(Reset)rp<=0;else if(RdEn)begin if(Empty)$fatal(1,"FIFO underflow");rp<=rp+1;end
endmodule
module tb_shared_tx;
 reg clk=0,rst_n=0;always #4 clk=~clk;
 reg wr=0;reg[7:0] data=0;
 wire cam_busy,cam_valid,cam_allow,imu_ready,imu_allow,imu_valid,dv;
 wire [7:0] cam_data,imu_data,out_data,ack_data;
 wire ack_ready,ack_allow,ack_valid;reg ack_pending=0;reg[127:0]ack_payload=0;
 reg pending=0;reg[511:0] payload=0;
 UDP_Send camera(.tx_allow(cam_allow),.tx_busy(cam_busy),.Clk(clk),.Rst_n(rst_n),.Tx_Done(),
 .des_mac(48'hffffffffffff),.src_mac(48'h000a3501fec0),.des_port(16'd6102),.src_port(16'd5000),
 .des_ip(32'hc0a80003),.src_ip(32'hc0a80002),.data_length(16'd1922),
 .GMII_GTXC(clk),.GMII_TXD(cam_data),.GMII_TXEN(cam_valid),
 .wrreq(wr),.wrdata(data),.wrclk(clk),.aclr(!rst_n),.wrusedw());
 imu_ethernet_tx imu(.clk(clk),.rst_n(rst_n),.valid(pending&&imu_allow),.payload(payload),
 .ready(imu_ready),.gmii_data(imu_data),.gmii_valid(imu_valid));
 motor_status_tx ack(.clk(clk),.rst_n(rst_n),.valid(ack_pending&&ack_allow),.payload(ack_payload),
 .ready(ack_ready),.gmii_data(ack_data),.gmii_valid(ack_valid));
 gmii_three_source_share share(.camera_busy(cam_busy),.camera_valid(cam_valid),.camera_data(cam_data),
 .imu_pending(pending),.imu_busy(!imu_ready),.imu_valid(imu_valid),.imu_data(imu_data),
 .motor_pending(ack_pending),.motor_busy(!ack_ready),.motor_valid(ack_valid),.motor_data(ack_data),
 .camera_allow(cam_allow),.imu_allow(imu_allow),.motor_allow(ack_allow),.gmii_data(out_data),.gmii_valid(dv));
 integer f,frames=0,gap=1000,count=0,cam_frames=0,imu_frames=0,ack_frames=0;
 reg previous=0;reg[1:0]frame_owner=0;
 wire[1:0] owner=ack_valid?2:imu_valid?1:0;
 always @(negedge clk)if(rst_n)begin
  if((cam_valid&&imu_valid)||(cam_valid&&ack_valid)||(imu_valid&&ack_valid))$fatal(1,"Transmit collision");
  if(dv)begin
   if(!previous)begin if(gap<12)$fatal(1,"Short cross-source IFG: %d",gap);frame_owner=owner;end
   if(frame_owner!=owner)$fatal(1,"Mid-frame ownership change");
   $fwrite(f,"%02x\n",out_data);count=count+1;gap=0;
  end else begin
   gap=gap+1;
   if(previous)begin
    if(frame_owner==2)begin if(count!=72)$fatal(1,"ACK length");ack_frames=ack_frames+1;end
    else if(frame_owner==1)begin if(count!=90)$fatal(1,"IMU length");imu_frames=imu_frames+1;end
    else begin if(count!=1976)$fatal(1,"Camera length %d",count);cam_frames=cam_frames+1;end
    $fwrite(f,"END\n");frames=frames+1;count=0;
   end
  end
  previous=dv;
 end
 integer row,i;
 initial begin
  f=$fopen("build/shared_frames.hex","w");
  #20;rst_n=1;
  for(row=0;row<8;row=row+1)begin
   wait(camera.eth_dcfifo.Wnum<2000);
   for(i=0;i<1922;i=i+1)begin
    @(negedge clk);wr=1;
    if(i==0)data=row;else if(i==1)data=0;else data=(i+row)%251;
   end
   @(negedge clk);wr=0;
   // Give the FIFO time to free a whole row before another producer burst.
   repeat(30)@(negedge clk);
  end
 end
 integer n;
 initial begin
  wait(rst_n);
  // IMU while camera idle, during a camera frame, and at subsequent boundaries.
  for(n=0;n<8;n=n+1)begin
   if(n!=0)begin wait(cam_valid);repeat(50+n*3)@(negedge clk);end
   @(negedge clk);payload={32'h494d5539,480'b0}|n;pending=1;
   @(posedge clk);while(!(imu_ready&&imu_allow))@(posedge clk);
   @(negedge clk);pending=0;
   wait(!imu_ready);wait(imu_ready);
  end
  wait(cam_frames==8 && imu_frames==8 && ack_frames==8);repeat(20)@(negedge clk);
  $fclose(f);$display("PASS complete camera/IMU/ACK frames, arbitration, backpressure, cross-source IFG");$finish;
 end
 integer ackn;
 initial begin
  wait(rst_n);
  for(ackn=0;ackn<8;ackn=ackn+1)begin
   if(ackn!=0)begin wait(cam_valid);repeat(71+ackn*3)@(negedge clk);end
   @(negedge clk);ack_payload={32'h41434b31,8'd1,8'd5,32'(ackn),16'(100+ackn),16'(200+ackn),8'd3,8'd12};ack_pending=1;
   @(posedge clk);while(!(ack_ready&&ack_allow))@(posedge clk);
   @(negedge clk);ack_pending=0;wait(!ack_ready);wait(ack_ready);
  end
 end
 initial begin #1000000;$fatal(1,"timeout camera=%d imu=%d",cam_frames,imu_frames);end
endmodule
