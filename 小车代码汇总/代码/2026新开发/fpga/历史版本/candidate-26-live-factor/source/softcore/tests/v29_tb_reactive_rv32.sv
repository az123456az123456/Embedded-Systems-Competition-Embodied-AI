`timescale 1ns/1ps
// Exact production producer/filter/IMU9 monitor and real RV32 firmware.
// Only raw MPU reader samples and external raw sonar/encoder inputs are driven.
module v29_tb_reactive_rv32;
 reg clk=0,reset_n=0,sample_pulse=0;always #10 clk=~clk;
 tri1 scl,sda;wire monitor_valid,trap,ready;wire[511:0]monitor;
 wire[63:0]pwm;wire[127:0]diag;
 reg[4:0]flags=2;reg[31:0]motion=0,epoch=0,sequence_number=1;
 reg[127:0]positions=0;reg[47:0]ranges={16'd250,16'd250,16'd250};
 reg[95:0]source_seq=0,source_stamp=0;reg[2:0]online=7;reg stale_front=0;
 imu_gmii_source source(.clk_50m(clk),.tx_clk(clk),.reset_n(reset_n),.imu_scl(scl),.imu_sda(sda),
 .led(),.tx_allow(1'b0),.tx_pending(),.tx_busy(),.gmii_data(),.gmii_valid(),.monitor_valid(monitor_valid),.monitor_packet(monitor));
 robot_nav_soc soc(.clk(clk),.reset_n(reset_n),.imu_valid(monitor_valid),.imu_packet(monitor),
 .motor_valid(1'b0),.motor_packet(128'b0),.sensor_valid(1'b0),.sensor_packet(512'b0),
 .nav_request_valid(1'b0),.nav_request_packet(512'b0),.nav_request_ready(),.nav_reply_valid(),.nav_reply_packet(),.nav_reply_ready(1'b1),
 .nav_status(),.cpu_trap(trap),.control_flags(flags),.control_epoch(epoch),.control_sequence(sequence_number),.control_motion(motion),.control_masks(8'hfa),
 .encoder_positions(positions),.encoder_illegal(4'd0),.sonar_mm(ranges),.sonar_seq(source_seq),.sonar_stamp(source_stamp),.sonar_online(online),
 .control_pwm(pwm),.control_ready(ready),.filter_mm(),.filter_seq(),.filter_valid(),.control_diagnostics(diag));
 integer cycles=0,frames=0,commits=0,max_gap=0,last_cycle=0,k;reg[31:0]previous_commit=0;
 always @(negedge clk)if(reset_n)begin
 cycles++;sample_pulse=(cycles%500000)==0;
 if(trap)$fatal(1,"actual RV32 trap cycle%0d pcaddr%h",cycles,soc.mem_addr);
 if(monitor_valid)frames++;
 if(cycles%5000000==0)for(k=0;k<3;k++)if(k!=0||!stale_front)begin source_seq[k*32+:32]++;source_stamp[k*32+:32]=soc.uptime_ms;end
 if(soc.commit_value!=previous_commit)begin
  previous_commit=soc.commit_value;commits++;
  if(last_cycle && cycles-last_cycle>max_gap)max_gap=cycles-last_cycle;
  if(last_cycle && cycles-last_cycle>1200000)$fatal(1,"control service gap exceeded24ms");last_cycle=cycles;
 end
 end
 task hold_ms(input integer n);integer until_ms;begin until_ms=soc.uptime_ms+n;wait(soc.uptime_ms>=until_ms);@(negedge clk);end endtask
 task zero;begin if(pwm!==0)$fatal(1,"must zero PWM t%0d value%h",soc.uptime_ms,pwm);end endtask
 task healthy;begin if(!ready || !soc.committed_flags[25] || diag[95:64]!=100)$fatal(1,"sonar WAIT lost negativeZ IMUcal/readiness t%0d flags%h",soc.uptime_ms,soc.committed_flags);end endtask
 task drive_sign(input integer direction);begin
 if(direction==1)begin for(integer i=0;i<4;i++)if($signed(pwm[i*16+:16])<=0)$fatal(1,"low speed must move at250mm");end
 if(direction==2 && !($signed(pwm[15:0])>0 && $signed(pwm[31:16])<0 && $signed(pwm[47:32])>0 && $signed(pwm[63:48])<0))$fatal(1,"CCW pureturn should move both sides at250mm");
 if(direction==3 && !($signed(pwm[15:0])<0 && $signed(pwm[31:16])>0 && $signed(pwm[47:32])<0 && $signed(pwm[63:48])>0))$fatal(1,"CW opposite side should recover");
 healthy();end endtask
 task command(input integer v,input integer w);begin @(negedge clk);motion={w[15:0],v[15:0]};sequence_number++;end endtask
 initial begin
 force source.raw_ax=16'd1561;force source.raw_ay=16'd497;force source.raw_az=-16'sd16384;
 force source.raw_temp=16'd0;force source.raw_gx=-16'sd300;force source.raw_gy=-16'sd304;force source.raw_gz=16'd30;
 force source.raw_ag_valid=sample_pulse;force source.mpu_init=1'b1;
 repeat(5)@(negedge clk);reset_n=1;wait(soc.status==3);wait(ready);zero();healthy();
 flags=7;epoch=1;command(20,0);hold_ms(45);drive_sign(1);
 command(300,0);hold_ms(8);zero();healthy();
 command(0,600);hold_ms(45);drive_sign(2);
 ranges[31:16]=190;hold_ms(8);zero();healthy();
 command(0,-600);hold_ms(45);drive_sign(3);
 online=6;hold_ms(8);zero();healthy();hold_ms(80);zero();healthy();
 online=7;hold_ms(45);drive_sign(3);
 stale_front=1;source_stamp[31:0]=soc.uptime_ms-751;hold_ms(8);zero();healthy();
 if(soc.uptime_ms-source_stamp[31:0]<751)$fatal(1,"test did not preserve actual stale source timestamp");
 stale_front=0;hold_ms(130);drive_sign(3);
 // Existing physically impossible encoder-jump gate is not weakened.
 positions[31:0]=32'h80000000;hold_ms(21);zero();healthy();
 flags=8;hold_ms(8);zero();healthy();
 $display("PASS v29 actual source/filter/IMU9->MMIO->RV32 reactive control:negativeZ ready;250mm20mmpsdrive+pureturn;300mmpsdynamicstop;190mmside/oppositerecovery;sonarWAIT/sourceage/calpreserved;trueencoderjump/lease;cycles=%0d frames=%0d commits=%0d max_commit_cycles=%0d",cycles,frames,commits,max_gap);
 $finish;
 end
 initial begin #4000000000;$fatal(1,"timeout t%0d status%h ready%b",soc.uptime_ms,soc.status,ready);end
endmodule
