`timescale 1ns/1ps
module v28_1_tb_source_mount;
 reg clk=0,reset_n=0,sample_pulse=0;
 always #10 clk=~clk;
 tri1 scl,sda;
 wire monitor_valid,trap,ready;wire [511:0] monitor;
 wire [63:0] pwm;wire [127:0] diag;
 imu_gmii_source source(.clk_50m(clk),.tx_clk(clk),.reset_n(reset_n),.imu_scl(scl),.imu_sda(sda),
  .led(),.tx_allow(1'b0),.tx_pending(),.tx_busy(),.gmii_data(),.gmii_valid(),.monitor_valid(monitor_valid),.monitor_packet(monitor));
 robot_nav_soc soc(.clk(clk),.reset_n(reset_n),.imu_valid(monitor_valid),.imu_packet(monitor),
  .motor_valid(1'b0),.motor_packet(128'b0),.sensor_valid(1'b0),.sensor_packet(512'b0),
  .nav_request_valid(1'b0),.nav_request_packet(512'b0),.nav_request_ready(),.nav_reply_valid(),.nav_reply_packet(),.nav_reply_ready(1'b1),
  .nav_status(),.cpu_trap(trap),.control_flags(5'd2),.control_epoch(32'd0),.control_sequence(32'd1),.control_motion(32'd0),.control_masks(8'hfa),
  .encoder_positions(128'd0),.encoder_illegal(4'd0),.sonar_mm({16'd1500,16'd1500,16'd1500}),.sonar_seq({32'd1,32'd1,32'd1}),.sonar_stamp(96'd0),.sonar_online(3'b111),
  .control_pwm(pwm),.control_ready(ready),.filter_mm(),.filter_seq(),.filter_valid(),.control_diagnostics(diag));
 integer cycles=0,frames=0;
 always @(negedge clk)if(reset_n)begin
  cycles=cycles+1;sample_pulse=(cycles%500000)==0;
  if(trap)$fatal(1,"CPU trap cycle%0d addr=%08x status=%x error=%x",cycles,soc.mem_addr,soc.status,soc.fw_error);
  if(monitor_valid)begin
   frames=frames+1;
   if(frames<=3)$display("PRODUCTION packet=%0128x",monitor);
  end
 end
 initial begin
  force source.raw_ax=16'd1561;force source.raw_ay=16'd497;force source.raw_az=-16'sd16384;
  force source.raw_temp=16'd0;force source.raw_gx=-16'sd300;force source.raw_gy=-16'sd304;force source.raw_gz=16'd30;
  force source.raw_ag_valid=sample_pulse;force source.mpu_init=1'b1;
  repeat(5)@(negedge clk);reset_n=1;
  wait(soc.status==3);wait(ready);
  if(pwm!==0)$fatal(1,"unarmed must remain zero");
  $display("PASS actual source/filter/IMU9 monitor->MMIO->publishedRV32 with real-log negative-Z/non-symmetricXYZ bias t=%0d cycles=%0d frames=%0d committed=%x diag=%032x",soc.uptime_ms,cycles,frames,soc.committed_flags,diag);
  @(negedge clk);
  // True CCW90deg/s about the calibrated gravity direction of the real
  // upside-down/tilted board. SensorZ alone would have the wrong sign.
  force source.raw_gx=16'sd818;force source.raw_gy=16'sd50;force source.raw_gz=-16'sd11700;
  wait(soc.uptime_ms>=3024);
  @(negedge clk);
  force source.raw_gx=-16'sd300;force source.raw_gy=-16'sd304;force source.raw_gz=16'd30;
  // The existing unity-DC gyro IIR and20ms output commit deliberately lag
  // a rate step. Check integrated area after its physical stop-tail settles,
  // never compare an old published sample at the exact end of motion.
  wait(soc.uptime_ms>=3224);
  if($signed(diag[31:0])<89500 || $signed(diag[31:0])>90500)
   $fatal(1,"wrong world yaw after negativeZ physical90deg turn yaw=%0d",$signed(diag[31:0]));
  if(!ready || !soc.committed_flags[25])$fatal(1,"lost readiness/mount negativeZ flag");
  $display("PASS v28.1 actual source+RV32 positive physical90deg turn with negativeZ mount: yaw_mdeg=%0d flags=%08x cycles=%0d",$signed(diag[31:0]),soc.committed_flags,cycles);
  $finish;
 end
 initial begin #4000000000;$fatal(1,"notready t=%0d status=%x err=%x hb=%0d committed=%x diag=%032x trap=%b imu=%0128x",soc.uptime_ms,soc.status,soc.fw_error,soc.heartbeat,soc.committed_flags,diag,trap,monitor);end
endmodule
