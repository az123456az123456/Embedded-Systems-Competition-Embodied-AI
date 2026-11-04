`timescale 1ns/1ps
`include "softcore/firmware/firmware_symbols.vh"
module v28_tb_layered_firmware;
 reg clk=0,reset_n=0,imu_valid=0;
 always #10 clk=~clk;
 reg[511:0] imu_packet=0,request=0;
 reg request_valid=0;wire request_ready,reply_valid;
 wire[511:0] reply;wire[207:0] nav_status;wire trap,core_ready;
 reg[4:0] flags=2;reg[31:0] motion=0,epoch=0,sequence_number=0;
 reg[127:0] positions=0;
 reg[47:0] sonar_mm={16'd2000,16'd2000,16'd2000};
 reg[95:0] sonar_seq=0,sonar_stamp=0;
 wire[63:0] pwm;wire[127:0] diagnostics;wire[47:0] filter_mm;
 wire[95:0] filter_seq;wire[2:0] filter_valid;
 robot_nav_soc dut(.clk(clk),.reset_n(reset_n),.imu_valid(imu_valid),.imu_packet(imu_packet),
  .motor_valid(1'b0),.motor_packet(128'b0),.sensor_valid(1'b0),.sensor_packet(512'b0),
  .nav_request_valid(request_valid),.nav_request_packet(request),.nav_request_ready(request_ready),
  .nav_reply_valid(reply_valid),.nav_reply_packet(reply),.nav_reply_ready(1'b1),
  .nav_status(nav_status),.cpu_trap(trap),.control_flags(flags),.control_epoch(epoch),
  .control_sequence(sequence_number),.control_motion(motion),.control_masks(8'hfa),
  .encoder_positions(positions),.encoder_illegal(4'b0),.sonar_mm(sonar_mm),
  .sonar_seq(sonar_seq),.sonar_stamp(sonar_stamp),.sonar_online(3'b111),
  .control_pwm(pwm),.control_ready(core_ready),.filter_mm(filter_mm),.filter_seq(filter_seq),
  .filter_valid(filter_valid),.control_diagnostics(diagnostics));
 integer clock_count=0,last_commit_cycle=0,max_commit_cycles=0,commit_count=0;
 integer k,tick_count=0,gyro_raw=26;
 integer fd,previous_commit=0;
 real velocity[0:3],position_accum[0:3],gain[0:3],desired,yaw_rate_mdps;
 integer raw_count,raw_velocity,seq=0;
 reg [7:0] map[0:511];reg[31:0] crc;integer b;
 always @(negedge clk)if(reset_n)begin
  clock_count=clock_count+1;imu_valid=0;
  if(trap)$fatal(1,"RV32 trap cycle%0d address%08x",clock_count,dut.mem_addr);
  if(dut.commit_value!=previous_commit)begin
   if(last_commit_cycle && clock_count-last_commit_cycle>max_commit_cycles)
    max_commit_cycles=clock_count-last_commit_cycle;
   if(last_commit_cycle && clock_count-last_commit_cycle>1200000)
    $fatal(1,"control commit gap%0d >24ms during actual RV32 execution",clock_count-last_commit_cycle);
   last_commit_cycle=clock_count;previous_commit=dut.commit_value;commit_count=commit_count+1;
  end
  if(clock_count%50000==0)begin
   tick_count=tick_count+1;
   for(k=0;k<4;k=k+1)begin
    raw_velocity=$signed(pwm[k*16 +:16]);desired=raw_velocity*gain[k];
    velocity[k]=velocity[k]+(desired-velocity[k])/120.0;
    position_accum[k]=position_accum[k]+velocity[k]/3.388;
    raw_count=$rtoi(position_accum[k]);if(k==0||k==2)raw_count=-raw_count;
    positions[k*32 +:32]=raw_count;
   end
   if(tick_count%10==0)begin
    yaw_rate_mdps=(((velocity[0]+velocity[2])-(velocity[1]+velocity[3]))/2.0)/180.635*57295.7795;
    gyro_raw=26+$rtoi(yaw_rate_mdps*131.0/1000.0);
    seq=seq+1;imu_packet=0;imu_packet[511 -:32]=32'h494d5539;
    imu_packet[479 -:32]=32'h0192000f;imu_packet[447 -:32]=seq;
    imu_packet[415 -:32]=dut.uptime_ms;
    imu_packet[351 -:32]={16'd16384,16'd0};
    imu_packet[287 -:32]={gyro_raw[15:0],16'd0};
    imu_packet[63 -:32]=32'd100;imu_valid=1;
   end
   if(tick_count%100==0)begin
    for(k=0;k<3;k=k+1)begin
     sonar_seq[k*32 +:32]=sonar_seq[k*32 +:32]+1;
     sonar_stamp[k*32 +:32]=dut.uptime_ms;
    end
   end
  end
 end
 task message(input[7:0]op,input[31:0]id,input[31:0]a,input[31:0]bb,input[7:0]len,input integer offset,input[7:0]expected);
  integer j;
  begin
   wait(request_ready);@(negedge clk);request=0;
   request[511 -:32]=32'h4e415632;request[479 -:8]=1;request[471 -:8]=op;
   request[455 -:8]=len;request[447 -:32]=id;request[415 -:32]=1;
   request[383 -:32]=a;request[351 -:32]=bb;
   if(op==2)for(j=0;j<32;j=j+1)request[255-j*8 -:8]=map[offset+j];
   request_valid=1;@(negedge clk);request_valid=0;
   wait(reply_valid);
   if(reply[463 -:8]!=expected)$fatal(1,"NAV op%0d status%0d expected%0d",op,reply[463 -:8],expected);
   @(negedge clk);
  end
 endtask
 initial begin
  for(k=0;k<4;k=k+1)begin velocity[k]=0;position_accum[k]=0;end
  gain[0]=0.50;gain[1]=0.47;gain[2]=0.52;gain[3]=0.46;
  #1;dut.ram[(`NAV_FW_STACK_BOTTOM>>2)-1]=32'hc001c0de;
  dut.ram[`NAV_FW_STACK_BOTTOM>>2]=32'h5a7ac001;
  repeat(5)@(negedge clk);reset_n=1;
  wait(dut.status==3);wait(core_ready);
  if(pwm!==0)$fatal(1,"Unarmed calibration must never move wheels");
  $display("CALIBRATED t_ms=%0d bias_mdps=%0d",dut.uptime_ms,$signed(diagnostics[63:32]));
  @(negedge clk);flags=7;motion={16'd0,16'd200};epoch=1;sequence_number=1;
  wait(dut.uptime_ms>4300);
  for(k=0;k<4;k=k+1)begin
   if(velocity[k]<180.0||velocity[k]>220.0)$fatal(1,"wheel%0d speed=%f outside PI regulation",k,velocity[k]);
  end
  $display("PI unequal plant wheel_mmps=(%.3f,%.3f,%.3f,%.3f) t_ms=%0d",velocity[0],velocity[1],velocity[2],velocity[3],dut.uptime_ms);
  for(b=0;b<512;b=b+1)map[b]=(b>=256&&b<264)?8'hff:8'h00;
  crc=32'hffffffff;
  for(b=0;b<512;b=b+1)begin
   crc=crc^map[b];for(k=0;k<8;k=k+1)crc=(crc>>1)^((crc&1)?32'hedb88320:0);
  end
  crc=~crc;
  message(1,1,crc,0,0,0,0);
  for(b=0;b<16;b=b+1)message(2,b+2,b*32,0,32,b*32,0);
  message(3,18,0,0,0,0,0);
  message(4,19,0,4095,0,0,6);
  if(!core_ready)$fatal(1,"A* stole control deadline/IMU readiness");
  if(dut.ram[`NAV_FW_SERVICE_MAX_GAP>>2]>500000)
   $fatal(1,"service callback gap>10ms actualcycles=%0d",dut.ram[`NAV_FW_SERVICE_MAX_GAP>>2]);
  @(negedge clk);motion=0;sequence_number=2;
  repeat(150000)@(negedge clk);
  if(pwm!==0)$fatal(1,"Zero MOT4 target did not produce immediate hard zero");
  if(dut.ram[(`NAV_FW_STACK_BOTTOM>>2)-1]!==32'hc001c0de || dut.ram[`NAV_FW_STACK_BOTTOM>>2]!==32'h5a7ac001)
   $fatal(1,"Firmware stack/workspace guard overwritten");
  fd=$fopen("reports/v28_firmware_rv32.json","w");
  $fwrite(fd,"{\"passed\":true,\"clock_hz\":50000000,\"actual_rv32_cycles\":%0d,\"commits\":%0d,\"max_commit_gap_cycles\":%0d,\"max_commit_gap_ms\":%.5f,\"max_service_gap_cycles\":%0d,\"astar_expanded\":%0d,\"astar_cycles\":%0d,\"hardware_tested\":false}\n",clock_count,commit_count,max_commit_cycles,max_commit_cycles/50000.0,dut.ram[`NAV_FW_SERVICE_MAX_GAP>>2],dut.expanded,dut.astar_cycles);
  $fclose(fd);
  $display("PASS actual RV32IM layered firmware:cycles=%0d commits=%0d max_commit_ms=%.5f max_service_cycles=%0d Astar_nodes=%0d cycles=%0d",clock_count,commit_count,max_commit_cycles/50000.0,dut.ram[`NAV_FW_SERVICE_MAX_GAP>>2],dut.expanded,dut.astar_cycles);
  $finish;
 end
 initial begin #10000000000;$fatal(1,"actual RV32 layered firmware timeout");end
endmodule
