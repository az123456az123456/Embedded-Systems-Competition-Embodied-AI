`timescale 1ns/1ps
// Clocks scaled together (100 kHz:250 kHz), preserving production50:125 ratio.
// Actual50MHz/20kHz period/phase is separately covered by tb_real_clock.v.
module tb_angle_crosswire;
GSR GSR(.GSRI(1'b1));
reg motor_clk=0,rx_clk=0,rst_n=0;reg[3:0]rxd=0;reg ctl=0;
always #5000 motor_clk=~motor_clk;always #2000 rx_clk=~rx_clk;
wire cv,rx_reset_n,src_ready,cmd_valid;wire[127:0]rx_cmd,cmd,status;
wire[3:0]pwm,in1,in2,flags;wire signed[15:0]left,right;wire[31:0]seqno;
wire sv,ack_allow,ack_pending,ack_busy,ack_valid;assign ack_allow=1'b1;wire[7:0]ack_data;
// P3 aliases deliberately express the physical output wiring under test.
wire p3_1,p3_6,p3_10,p3_16,p3_2,p3_7,p3_28,p3_19,p3_5,p3_9,p3_31,p3_39;
assign pwm={p3_16,p3_10,p3_6,p3_1};assign in1={p3_19,p3_28,p3_7,p3_2};assign in2={p3_39,p3_31,p3_9,p3_5};
motor_rgmii_rx receiver(.reset_n(rst_n),.rgmii_rx_clk(rx_clk),.rgmii_rxd(rxd),.rgmii_rx_ctl(ctl),.command_valid(cv),.command_data(rx_cmd),.receive_reset_n(rx_reset_n));
motor_command_cdc cdc(.src_clk(rx_clk),.src_rst_n(rx_reset_n),.src_valid(cv),.src_data(rx_cmd),.src_ready(src_ready),.dst_clk(motor_clk),.dst_rst_n(rst_n),.dst_valid(cmd_valid),.dst_data(cmd));
motor_remote_control #(.CLK_HZ(100000),.PWM_HZ(1000)) motors(.clk(motor_clk),.rst_n(rst_n),.command_valid(cmd_valid),.command_data(cmd),.motor_pwm({p3_16,p3_10,p3_6,p3_1}),.motor_in1({p3_19,p3_28,p3_7,p3_2}),.motor_in2({p3_39,p3_31,p3_9,p3_5}),.status_flags(flags),.status_valid(sv),.status_data(status),.last_sequence(seqno),.applied_left(left),.applied_right(right));
motor_status_gmii #(.NAV_ENABLE(1)) ack(.source_clk(motor_clk),.source_rst_n(rst_n),.status_valid(sv),.status_data(status),.nav_data({32'h4e415631,8'd1,8'd3,16'd127,32'd5,32'd100000,32'd127,32'd1,16'd0}),.tx_clk(rx_clk),.tx_rst_n(rst_n),.tx_allow(ack_allow),.tx_pending(ack_pending),.tx_busy(ack_busy),.gmii_valid(ack_valid),.gmii_data(ack_data));
reg[7:0]frames[0:1215];integer fh,j,n,i,highs[0:3];reg previous=0;reg[7:0]captured[0:71];integer capture_n=0,ci,nav_frames=0;
always @(negedge rx_clk)begin
 if(ack_valid)begin captured[capture_n]=ack_data;capture_n=capture_n+1;end
 else if(previous)begin
  if(capture_n!=72)$fatal(1,"status frame length");
  if(captured[21]==8'hb6)begin
   for(ci=0;ci<capture_n;ci=ci+1)$fwrite(fh,"%02x\n",captured[ci]);
   $fwrite(fh,"END\n");
  end else if(captured[21]==8'hb7)nav_frames=nav_frames+1;
  else $fatal(1,"unknown status frame");
  capture_n=0;
 end
 previous=ack_valid;
end
task cycles;input integer count;begin repeat(count)@(negedge motor_clk);end endtask
task byte_tx;input[7:0]value;begin
 @(negedge rx_clk);#1;rxd=value[3:0];ctl=1;
 @(posedge rx_clk);#1;rxd=value[7:4];ctl=1;
end endtask
task frame_tx;input integer index;begin
 for(j=0;j<8;j=j+1)byte_tx(j==7?8'hd5:8'h55);
 for(j=0;j<64;j=j+1)byte_tx(frames[index*64+j]);
 @(negedge rx_clk);#1;ctl=0;rxd=0;cycles(20);
 if(seqno!==index+1)$fatal(1,"MOT1/MOT2 did not pass IDDR/parser/CDC seq%0d got%0d",index+1,seqno);
end endtask
task check_outputs;input integer l,r;input[3:0]expected_in1,expected_in2;begin
 if(left!==l||right!==r||in1!==expected_in1||in2!==expected_in2)$fatal(1,"P3 direction/output mismatch seq%0d L%0d R%0d IN1%x IN2%x",seqno,left,right,in1,in2);
 for(i=0;i<4;i=i+1)highs[i]=0;
 for(n=0;n<100;n=n+1)begin cycles(1);for(i=0;i<4;i=i+1)highs[i]=highs[i]+pwm[i];end
 for(i=0;i<4;i=i+1)if(highs[i]!==((i==1||i==3)?(l<0?-l:l):(r<0?-r:r))/10)$fatal(1,"P3 PWM duty mismatch motor%0d highs%0d",i+1,highs[i]);
 $display("PASS P3 seq%0d L=%0d R=%0d IN1=%x IN2=%x PWM highs[%0d,%0d,%0d,%0d]/100",seqno,l,r,in1,in2,highs[0],highs[1],highs[2],highs[3]);
end endtask
initial begin
 $readmemh("build/angle_crosswire/mot2.hex",frames);fh=$fopen("build/angle_crosswire/ack2.hex","w");
 cycles(5);rst_n=1;cycles(5);
 for(i=0;i<19;i=i+1)begin frame_tx(i);cycles(40000);end
 cycles(11000);if(flags!==4'b0010)$fatal(1,"watchdog did not clear arm and config");
 if(pwm!=0 || left!=0 || right!=0)$fatal(1,"timeout output not zero");cycles(10000);
 if(nav_frames<2)$fatal(1,"NAV diagnostics absent");
 $fclose(fh);$display("PASS actual RK MOT1/MOT2 through Gowin IDDR/parser/CDC and ACK1/ACK2/NAV1 CDC, timeout outputs zero");$finish;
end
initial begin #10000000000;$fatal(1,"timeout");end
endmodule
