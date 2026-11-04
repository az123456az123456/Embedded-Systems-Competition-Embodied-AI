`timescale 1ns/1ps
module eth_dcfifo(input [7:0] Data,input Reset,WrClk,RdClk,WrEn,RdEn,
 output [12:0] Wnum,Rnum,output [7:0] Q,output Empty,Full);
 reg [7:0] mem[0:8191]; integer wp=0,rp=0;
 assign Wnum=wp-rp; assign Rnum=wp-rp; assign Q=mem[rp%8192];
 assign Empty=(wp==rp); assign Full=(wp-rp>=4096);
 always @(posedge WrClk or posedge Reset)if(Reset)wp<=0;else if(WrEn)begin if(Full)$fatal(1,"FIFO overflow");mem[wp%8192]<=Data;wp<=wp+1;end
 always @(posedge RdClk or posedge Reset)if(Reset)rp<=0;else if(RdEn)begin if(Empty)$fatal(1,"FIFO underflow");rp<=rp+1;end
endmodule
module tb_scheduler_planner_stress #(parameter integer SENSOR_PERIOD_CYCLES=2500000, parameter integer LINE_CYCLES=2793);
reg clk=0,src_clk=0,rst_n=0,wr=0;reg[7:0]data=0;
always #4 clk=~clk;always #10 src_clk=~src_clk;
wire cam_busy,cam_valid,cam_allow,imu_ready,imu_allow,imu_valid,dv,ack_pending,ack_busy,ack_valid,ack_allow;
wire[7:0]cam_data,imu_data,ack_data,out_data;
reg sensor_publish=0;wire sensor_pending,sensor_busy,sensor_valid,sensor_allow;wire [7:0] sensor_data;
wire sonar_pending,sonar_busy,sonar_valid,sonar_allow;wire[7:0]sonar_data;
reg plan_publish=0;wire plan_pending,plan_busy,plan_valid,plan_allow;wire[7:0]plan_data;
assign sensor_pending=sonar_pending||plan_pending;
assign sensor_busy=sonar_busy||plan_busy;
assign sensor_valid=sonar_valid||plan_valid;
assign sensor_data=sonar_valid?sonar_data:plan_data;
nav_message_gmii planner(.source_clk(src_clk),.source_rst_n(rst_n),.message_valid(plan_publish),
.message_data({32'h4e415632,8'd1,8'h85,16'd0,448'b0}),.message_ready(),
.tx_clk(clk),.tx_rst_n(rst_n),.tx_allow(plan_allow),.tx_pending(plan_pending),.tx_busy(plan_busy),.gmii_valid(plan_valid),.gmii_data(plan_data));
reg imu_publish=0,ack_publish=0;reg[511:0]imu_payload=0;reg[127:0]ack_payload=0;
wire[511:0]imu_wire;wire imu_pending;
UDP_Send camera(.tx_allow(cam_allow),.tx_busy(cam_busy),.Clk(clk),.Rst_n(rst_n),.Tx_Done(),.des_mac(48'hffffffffffff),.src_mac(48'h000a3501fec0),.des_port(16'd6102),.src_port(16'd5000),.des_ip(32'hc0a80003),.src_ip(32'hc0a80002),.data_length(16'd1922),.GMII_GTXC(clk),.GMII_TXD(cam_data),.GMII_TXEN(cam_valid),.wrreq(wr),.wrdata(data),.wrclk(clk),.aclr(!rst_n),.wrusedw());
imu_snapshot_cdc icdc(.src_clk(src_clk),.src_rst_n(rst_n),.src_valid(imu_publish),.src_data(imu_payload),.src_ready(),.dst_clk(clk),.dst_rst_n(rst_n),.dst_ready(imu_ready&&imu_allow),.dst_valid(imu_pending),.dst_data(imu_wire));
imu_ethernet_tx imu(.clk(clk),.rst_n(rst_n),.valid(imu_pending&&imu_allow),.payload(imu_wire),.ready(imu_ready),.gmii_data(imu_data),.gmii_valid(imu_valid));
motor_status_gmii #(.NAV_ENABLE(1)) ack(.source_clk(src_clk),.source_rst_n(rst_n),.status_valid(ack_publish),.status_data(ack_payload),.nav_data({32'h4e415631,8'd1,8'd3,16'd127,32'd5,32'd100000,32'd127,32'd1,16'd0}),.tx_clk(clk),.tx_rst_n(rst_n),.tx_allow(ack_allow),.tx_pending(ack_pending),.tx_busy(ack_busy),.gmii_valid(ack_valid),.gmii_data(ack_data));
sensor_status_gmii sensors(.source_clk(src_clk),.source_rst_n(rst_n),.sensor_valid(sensor_publish),
.sensor_data({32'h534e5331,8'd1,8'd7,8'd0,8'd0,32'd1,32'd100,128'd0,128'd0,16'd1234,16'd2000,16'd4500,48'd0,(SENSOR_PERIOD_CYCLES==50000?16'd0:16'd50),16'd0}),
.tx_clk(clk),.tx_rst_n(rst_n),.tx_allow(sonar_allow),.tx_pending(sonar_pending),.tx_busy(sonar_busy),.gmii_valid(sonar_valid),.gmii_data(sonar_data));
gmii_navigation_share share(.clk(clk),.rst_n(rst_n),.sensor_pending(sonar_pending),.sensor_busy(sonar_busy),.sensor_valid(sonar_valid),.sensor_data(sonar_data),.sensor_allow(sonar_allow),.planner_pending(plan_pending),.planner_busy(plan_busy),.planner_valid(plan_valid),.planner_data(plan_data),.planner_allow(plan_allow),.camera_busy(cam_busy),.camera_valid(cam_valid),.camera_data(cam_data),.imu_pending(imu_pending),.imu_busy(!imu_ready),.imu_valid(imu_valid),.imu_data(imu_data),.motor_pending(ack_pending),.motor_busy(ack_busy),.motor_valid(ack_valid),.motor_data(ack_data),.camera_allow(cam_allow),.imu_allow(imu_allow),.motor_allow(ack_allow),.gmii_data(out_data),.gmii_valid(dv));
integer plan_count=0,plan_wait=0,max_plan_wait=0;
integer nav_count=0,status_type=0,sensor_count=0,sensor_wait=0,max_sensor_wait=0;
integer camera_count=0,imu_count=0,ack_count=0,gap=999,count=0,owner,active_owner;
integer imu_wait=0,ack_wait=0,max_imu_wait=0,max_ack_wait=0;
reg previous=0;
reg [31:0] frame_crc;
function [31:0] crc_step;
 input [31:0] previous_crc;input [7:0] value;
 reg [31:0] c;integer k;
 begin c=previous_crc^{24'b0,value};for(k=0;k<8;k=k+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_step=c;end
endfunction
always @(negedge clk)if(rst_n)begin
 if((cam_allow?1:0)+(imu_allow?1:0)+(ack_allow?1:0)+(sonar_allow?1:0)+(plan_allow?1:0)>1)$fatal(1,"launch grant not exclusive");
 if((cam_valid&&!cam_busy)||(imu_valid&&imu_ready)||(ack_valid&&!ack_busy)||(sonar_valid&&!sonar_busy)||(plan_valid&&!plan_busy))$fatal(1,"Busy failed to cover GMII valid");
 if(sonar_valid&&plan_valid)$fatal(1,"NAV2 SNS1 collision");
 if(plan_pending&&!plan_valid)plan_wait=plan_wait+1;else begin if(plan_wait>max_plan_wait)max_plan_wait=plan_wait;plan_wait=0;end
 if((cam_valid?1:0)+(imu_valid?1:0)+(ack_valid?1:0)+(sensor_valid?1:0)>1)$fatal(1,"overlapping frames");
 if(imu_pending&&!imu_valid)imu_wait=imu_wait+1;else begin if(imu_wait>max_imu_wait)max_imu_wait=imu_wait;imu_wait=0;end
 if(ack_pending&&!ack_valid)ack_wait=ack_wait+1;else begin if(ack_wait>max_ack_wait)max_ack_wait=ack_wait;ack_wait=0;end
 if(sensor_pending&&!sensor_valid)sensor_wait=sensor_wait+1;else begin if(sensor_wait>max_sensor_wait)max_sensor_wait=sensor_wait;sensor_wait=0;end
 owner=ack_valid?2:sensor_valid?3:imu_valid?1:0;
 if(dv)begin
  if(!previous)begin if(gap<12)$fatal(1,"IFG below12");active_owner=owner;frame_crc=32'hffffffff;end
  if(active_owner!=owner)$fatal(1,"frame interrupted");count=count+1;gap=0;if(count>=9)frame_crc=crc_step(frame_crc,out_data);
  if((owner==2||owner==3) && count==22)status_type=out_data;
  // Verify source row identity/pixel bytes independently of the generated FCS.
  if(owner==0 && count==51 && out_data!==((camera_count%1080)&255))$fatal(1,"camera row low byte corrupt");
  if(owner==0 && count==52 && out_data!==((camera_count%1080)>>8))$fatal(1,"camera row high byte corrupt");
  if(owner==0 && count>=53 && count<=1972 && out_data!==((count-51+camera_count%1080)%251))$fatal(1,"camera pixel corrupt row=%0d offset=%0d",camera_count%1080,count-51);
 end else begin
  gap=gap+1;
  if(previous)begin
   if(frame_crc!=32'hdebb20e3)$fatal(1,"shared output Ethernet FCS corrupt owner=%d",active_owner);
   if(active_owner==0)begin if(count!=1976)$fatal(1,"camera length");camera_count=camera_count+1;end
   if(active_owner==1)begin if(count!=90)$fatal(1,"IMU length");imu_count=imu_count+1;end
   if(active_owner==2)begin if(count!=72)$fatal(1,"ACK length");if(status_type==8'hb6)ack_count=ack_count+1;else if(status_type==8'hb7)nav_count=nav_count+1;else $fatal(1,"unknown status EtherType");end
   if(active_owner==3)begin if(count!=92)$fatal(1,"SNS1 length");if(status_type==8'hb8)sensor_count=sensor_count+1;else if(status_type==8'hb9)plan_count=plan_count+1;else $fatal(1,"Unknown sensor/planner frame");end
   count=0;
  end
 end
 previous=dv;
end
integer f,row,j;
initial begin
 #31;rst_n=1;
 for(f=0;f<3;f=f+1)for(row=0;row<1120;row=row+1)begin
  for(j=0;j<LINE_CYCLES;j=j+1)begin @(negedge clk);wr=row<1080&&j<1922;data=j==0?row[7:0]:j==1?row[15:8]:(j+row)%251;end
 end
 @(negedge clk);wr=0;repeat(3000)@(negedge clk);
 if(SENSOR_PERIOD_CYCLES==50000 && sensor_count<70)$fatal(1,"fast sonar event loss count=%0d",sensor_count);
 if(plan_count<70||max_plan_wait>3000)$fatal(1,"NAV2 starvation count=%0d wait=%0d",plan_count,max_plan_wait);
 if(camera_count!=3240||imu_count<14||ack_count<2||nav_count<2||sensor_count<2)$fatal(1,"unexpected totals cam=%0d imu=%0d ack=%0d",camera_count,imu_count,ack_count);
 if(max_imu_wait>3000||max_ack_wait>2100||max_sensor_wait>3000)$fatal(1,"starvation imu=%0d ack=%0d",max_imu_wait,max_ack_wait);
 $display("PASS NAV2 frames=%0d max wait=%0d cycles while nominal40FPS",plan_count,max_plan_wait);
 $display("PASS SNS1 coexistence: sensor_frames=%0d max_sensor_wait_cycles=%0d",sensor_count,max_sensor_wait);
 $display("PASS sustained nominal40FPS with separate NAV1 frames: camera_rows=%0d (3 complete frames), IMU=%0d ACK=%0d, max_pending_wait IMU=%0d/%0dns ACK=%0d/%0dns, NAV=%0d wire lengths1976/90/72, IFG>=12, all Ethernet FCS valid, exclusive grants, no collision or FIFO overflow",camera_count,imu_count,ack_count,max_imu_wait,max_imu_wait*8,max_ack_wait,max_ack_wait*8,nav_count);$finish;
end
integer cycles=0;
always @(negedge src_clk)if(rst_n)begin
 imu_publish=0;ack_publish=0;sensor_publish=0;plan_publish=0;
 if(cycles%50000==0)plan_publish=1;
 if(cycles%250000==0)begin imu_publish=1;imu_payload={32'h494d5539,480'b0}|cycles;end
 if(cycles%2500000==0)begin ack_publish=1;ack_payload={32'h41434b31,96'b0}|cycles;end
 if(cycles%SENSOR_PERIOD_CYCLES==0)sensor_publish=1;
 cycles=cycles+1;
end
initial begin #110000000;$fatal(1,"timeout");end
endmodule
