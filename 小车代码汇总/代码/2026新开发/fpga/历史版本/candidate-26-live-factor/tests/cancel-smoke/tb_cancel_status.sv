`timescale 1ns/1ps
// Candidate18 cancellation smoke. Frozen real modules; no design force/write.
// Direct owner->motor, real complete telemetry buffer + real status consumers.
module tb_pipeline_measurement;
 reg rx_clk=0,clk=0,reset_n=0; real rx_half=4.0,cpu_half=10.0,cpu_phase=0.0;
 integer key_case=0;
 integer shift_bytes=0,phase_ps=0,phase_arg=0,pwm_sign=1,mode=0,observing=0,frame_id=0,rx_frame_id=0,request_frame_id=0,fd,j,k,n;
 integer duty=600, target_bit=-1, low_phase=0, settle=0, busy_mode=0, round_id=0;
 integer cdc_generation=0, tx_kind=0, next_kind=0, pending_kind=0, stream_count=0;
 reg busy_engine=0,pending_tx=0,tx_done=0,stream_stop=0;
 reg [7:0] queue_packet[0:283], bg_packet[0:283], emit_packet[0:283];
 reg[31:0] device_ms=100;reg cpu_trap=0;
 reg[3:0] rgmii_rxd=0;reg rgmii_rx_ctl=0;
 wire[7:0] gmii_data;wire gmii_valid,gmii_error,receive_reset_n;
 wire rx_command_valid;wire[127:0]rx_command_data;
 reg command_valid=0;reg[127:0]command_data=0;
 wire mot_accept,manual_quiet;
 reg[47:0]raw_sonar_mm={16'd2000,16'd2000,16'd2000};reg[2:0]raw_sonar_online=7;
 wire[3:0]motor_pwm,motor_in1,motor_in2;
 reg owner_bus_req=0;reg[11:0]owner_bus_offset=0;reg[31:0]owner_bus_wdata=0;reg[3:0]owner_bus_wstrb=0;
 wire owner_bus_rsp_valid;wire[31:0]owner_bus_rsp_data;
 reg ing_bus_req=0;reg[11:0]ing_bus_offset=0;reg[31:0]ing_bus_wdata=0;reg[3:0]ing_bus_wstrb=0;
 wire ing_bus_rsp_valid;wire[31:0]ing_bus_rsp_data;
 wire[127:0]pin_tokens,pin_rx_ms;wire[15:0]pin_flags;wire[35:0]pin_bytes;
 wire source_req,source_grant,source_valid;wire[1:0]source_lane;wire[5:0]source_word;wire[31:0]source_data;
 wire cancel_valid,cancel_ready;wire[417:0]cancel_desc;
 wire nav_owned,nav_live;wire[63:0]nav_pwm,nav_epoch;wire[15:0]nav_linear,nav_angular;wire[31:0]context_generation;
 wire[11:0]all_outputs={motor_in2,motor_in1,motor_pwm};
 GSR GSR(.GSRI(1'b1));
 robot_rgmii_rx receiver(.reset_n(reset_n),.rgmii_rx_clk(rx_clk),.rgmii_rxd(rgmii_rxd),.rgmii_rx_ctl(rgmii_rx_ctl),
 .command_valid(rx_command_valid),.command_data(rx_command_data),.nav_valid(),.nav_data(),.rgb_valid(),.rgb_epoch(),
 .receive_reset_n(receive_reset_n),.exec_gmii_data(gmii_data),.exec_gmii_valid(gmii_valid),.exec_gmii_error(gmii_error));
 nav3_ingress ingress(.rx_clk(rx_clk),.rx_reset_n(receive_reset_n),.cpu_clk(clk),.cpu_reset_n(reset_n),.device_ms(device_ms),
 .gmii_data(gmii_data),.gmii_valid(gmii_valid),.gmii_error(gmii_error),.bus_req(ing_bus_req),.bus_offset(ing_bus_offset),.bus_wdata(ing_bus_wdata),.bus_wstrb(ing_bus_wstrb),.bus_rsp_valid(ing_bus_rsp_valid),.bus_rsp_data(ing_bus_rsp_data),
 .pin_tokens(pin_tokens),.pin_rx_ms(pin_rx_ms),.pin_flags(pin_flags),.pin_bytes(pin_bytes),.source_req(source_req),.source_lane(source_lane),.source_word(source_word),.source_grant(source_grant),.source_valid(source_valid),.source_data(source_data),.cancel_valid(cancel_valid),.cancel_desc(cancel_desc),.cancel_ready(cancel_ready));
 nav_owner_gate owner(.clk(clk),.reset_n(reset_n),.device_ms(device_ms),.cpu_trap(cpu_trap),.mot_accept(mot_accept),.manual_quiet(manual_quiet),.raw_sonar_mm(raw_sonar_mm),.raw_sonar_online(raw_sonar_online),
 .bus_req(owner_bus_req),.bus_offset(owner_bus_offset),.bus_wdata(owner_bus_wdata),.bus_wstrb(owner_bus_wstrb),.bus_rsp_valid(owner_bus_rsp_valid),.bus_rsp_data(owner_bus_rsp_data),
 .pin_tokens(pin_tokens),.pin_rx_ms(pin_rx_ms),.pin_flags(pin_flags),.pin_bytes(pin_bytes),.source_req(source_req),.source_lane(source_lane),.source_word(source_word),.source_grant(source_grant),.source_valid(source_valid),.source_data(source_data),.cancel_valid(cancel_valid),.cancel_desc(cancel_desc),.cancel_ready(cancel_ready),.last_tx_ticket(32'd0),.nav_owned(nav_owned),.nav_live(nav_live),.nav_pwm(nav_pwm),.nav_linear(nav_linear),.nav_angular(nav_angular),.nav_epoch(nav_epoch),.context_generation(context_generation));
 wire drive_owned,drive_live; wire [63:0] drive_pwm; wire [15:0] drive_linear,drive_angular;
 assign {drive_owned,drive_live,drive_pwm,drive_linear,drive_angular}={nav_owned,nav_live,nav_pwm,nav_linear,nav_angular};
 wire motor_status_valid;wire[127:0] motor_status_data;
 motor_remote_control #(.ENCODER_SYNC_ENABLE(1),.NAV_EXEC_ENABLE(1)) motors(.clk(clk),.rst_n(reset_n),.command_valid(command_valid),.command_data(command_data),.encoder_valid(1'b0),.encoder_deltas(128'd0),.encoder_illegal(4'd0),.motor_pwm(motor_pwm),.motor_in1(motor_in1),.motor_in2(motor_in2),.status_flags(),.status_valid(motor_status_valid),.status_data(motor_status_data),.last_sequence(),.applied_left(),.applied_right(),.cpu_pwm(64'd0),.cpu_ready(1'b1),.cpu_trap(cpu_trap),.raw_sonar_mm(raw_sonar_mm),.raw_sonar_online(raw_sonar_online),.control_flags(),.control_epoch(),.control_sequence(),.control_motion(),.control_masks(),.nav_owned(drive_owned),.nav_live(drive_live),.nav_pwm(drive_pwm),.nav_linear(drive_linear),.nav_angular(drive_angular),.accepted_external_mot(mot_accept),.manual_quiet(manual_quiet));

 // Production candidate22 337-bit telemetry stage and real mailbox/serializer.
 // NAV1 test payload is an explicit synthetic changing marker: no RV32 claim.
 reg[31:0]telemetry_tick=0;
 wire[207:0]test_nav_status={32'h4e415631,8'd1,8'h03,16'd0,telemetry_tick,owner.event_id[31:0],owner.event_reason,context_generation,16'd0};
 wire buffered_valid;wire[127:0]buffered_status;wire[207:0]buffered_nav;
 wire ref_valid,dut_valid;wire[7:0]ref_byte,dut_byte;
 integer telemetry_ref_fd,telemetry_dut_fd,telemetry_in_fd,telemetry_samples=0;
 motor_status_align status_align(.clk(clk),.reset_n(reset_n),.in_valid(motor_status_valid),.in_status(motor_status_data),.in_nav(test_nav_status),.out_valid(buffered_valid),.out_status(buffered_status),.out_nav(buffered_nav));
 motor_status_gmii #(.NAV_ENABLE(1)) reference_status(.source_clk(clk),.source_rst_n(reset_n),.status_valid(motor_status_valid),.status_data(motor_status_data),.nav_data(test_nav_status),.tx_clk(rx_clk),.tx_rst_n(reset_n),.tx_allow(1'b1),.tx_pending(),.tx_busy(),.gmii_valid(ref_valid),.gmii_data(ref_byte));
 motor_status_gmii #(.NAV_ENABLE(1)) candidate_status(.source_clk(clk),.source_rst_n(reset_n),.status_valid(buffered_valid),.status_data(buffered_status),.nav_data(buffered_nav),.tx_clk(rx_clk),.tx_rst_n(reset_n),.tx_allow(1'b1),.tx_pending(),.tx_busy(),.gmii_valid(dut_valid),.gmii_data(dut_byte));
 reg[336:0]expected_telemetry;
 always @(posedge clk)begin
  if(!reset_n)begin telemetry_tick<=0;expected_telemetry=0;end
  else begin
   telemetry_tick<=telemetry_tick+1;
   expected_telemetry={motor_status_valid,motor_status_data,test_nav_status};
   if(motor_status_valid)begin
    telemetry_samples=telemetry_samples+1;
    $fdisplay(telemetry_in_fd,"{\"time_ps\":%0.0f,\"status\":\"%032h\",\"nav\":\"%052h\",\"cancel_observing\":%s}",$realtime*1000,motor_status_data,test_nav_status,observing?"true":"false");
   end
  end
  #0.001;if({buffered_valid,buffered_status,buffered_nav}!==expected_telemetry)$fatal(1,"337-bit telemetry stage mismatch");
 end
 always @(posedge rx_clk)begin #0.001;
  $fdisplay(telemetry_ref_fd,"%0.0f,%0d,%02h",$realtime*1000,ref_valid,ref_byte);
  $fdisplay(telemetry_dut_fd,"%0.0f,%0d,%02h",$realtime*1000,dut_valid,dut_byte);
 end
 initial begin if(!$value$plusargs("PHASE=%d",phase_arg))phase_arg=0;if(!$value$plusargs("SIGN=%d",pwm_sign))pwm_sign=1;if(!$value$plusargs("MODE=%d",mode))mode=0;if(!$value$plusargs("SHIFT=%d",shift_bytes))shift_bytes=0;if(!$value$plusargs("PHASE_PS=%d",phase_ps))phase_ps=0;if(!$value$plusargs("DUTY=%d",duty))duty=600;
 if(!$value$plusargs("TARGET_BIT=%d",target_bit))target_bit=-1;
 if(!$value$plusargs("LOW_PHASE=%d",low_phase))low_phase=0;
 if(!$value$plusargs("SETTLE=%d",settle))settle=0;
 if(!$value$plusargs("BUSY=%d",busy_mode))busy_mode=0;
  if(!$value$plusargs("KEY_CASE=%d",key_case))key_case=0;
 cpu_phase=phase_arg+phase_ps/1000.0;#(cpu_phase);forever #(cpu_half)clk=~clk;end
 initial forever #(rx_half)rx_clk=~rx_clk;
 reg[31:0]body[0:255],a[0:48],p[0:36];reg[31:0]ticket=0,token=0;reg[63:0]wire_seq=0;
 reg[63:0]test_phone=1,test_auth=1,test_session=17,test_run=2,test_map=1,test_rev=1,test_loc=1,test_plan=1;
 reg[127:0]test_goal=128'h0102030405060708090a0b0c0d0e0f10;reg[31:0]result,r;
 reg[7:0]packet[0:283];reg[15:0]crc16;reg[31:0]crc32;integer bi,bitno;
 task event_log(input string name);integer assoc;begin
  assoc=frame_id;
  if(name=="gmii_eof"||name=="parser_first_idle"||name=="accept_pending"||name=="cancel_pulse"||name=="request_toggle")assoc=rx_frame_id;
  if(name=="req_sync0_toggle"||name=="req_sync1_toggle"||name=="dst_valid"||name=="payload_valid"||name=="hard_event"||name=="nav_live_low"||name=="output_change")assoc=request_frame_id;
  if(observing)$fdisplay(fd,"{\"frame_id\":%0d,\"wire_frame_id\":%0d,\"event\":\"%s\",\"time_ps\":%0.0f,\"outputs_hex\":\"%03h\",\"outputs_bits\":\"%012b\",\"cdc_generation\":%0d,\"nav_live\":%0d,\"cancel_valid\":%0d,\"owner\":%0d,\"event_reason\":%0d,\"cancel_session\":%0d,\"cancel_run\":%0d,\"cancel_sequence\":%0d}",assoc,frame_id,name,$realtime*1000,all_outputs,all_outputs,cdc_generation,nav_live,cancel_valid,owner.owner,owner.event_reason,cancel_desc[65:2],cancel_desc[129:66],cancel_desc[321:258]);
 end endtask

 // Stable event timestamps use signal transition observation after NBA, not a
 // posedge CPU testbench read of the old value (which used to add 20ns).
 always @(posedge ingress.parser.accept_pending)event_log("accept_pending");
 always @(posedge ingress.cancel_pulse)event_log("cancel_pulse");
 always @(ingress.cancel_cross.request)begin request_frame_id=rx_frame_id;if(observing)cdc_generation=cdc_generation+1;event_log("request_toggle");end
 always @(ingress.cancel_cross.req_sync[0])event_log("req_sync0_toggle");
 always @(ingress.cancel_cross.req_sync[1])event_log("req_sync1_toggle");
 always @(posedge cancel_valid)begin event_log("dst_valid");event_log("payload_valid");end
 always @(posedge owner.hard_event)event_log("hard_event");
 always @(negedge nav_live)event_log("nav_live_low");
 always @(negedge gmii_valid)event_log("gmii_eof");
 always @(all_outputs)event_log("output_change");
 always @(posedge clk)begin #0.001;event_log("cpu_sample");end
 always @(posedge rx_clk)if(observing&&!gmii_valid&&ingress.parser.active&&!ingress.parser.accept_pending)event_log("parser_first_idle");
 task drive_byte(input[7:0]v,input bit valid,input integer idx);begin
  // Entry is already one ns after a negative RX edge.  No hidden alignment
  // wait between bytes or between frame/12 idle bytes/next frame.
  rgmii_rxd=v[3:0];rgmii_rx_ctl=valid;
  @(posedge rx_clk);if(idx==0)rx_frame_id=frame_id;if(observing)begin if(idx==0)event_log("rgmii_preamble_low");if(idx==7)event_log("rgmii_sfd_low");if(idx==283)event_log("rgmii_last_fcs_low");if(idx==284)event_log("rgmii_first_idle_low");end
  #1;rgmii_rxd=v[7:4];rgmii_rx_ctl=valid;
  @(negedge rx_clk);if(observing)begin if(idx==7)event_log("rgmii_sfd_high");if(idx==283)event_log("rgmii_last_fcs_high");if(idx==284)event_log("rgmii_first_idle_high");end
  #1;
 end endtask
 task make_frame(input integer lane,input integer corrupt);begin
  for(bi=0;bi<7;bi=bi+1)packet[bi]=8'h55;packet[7]=8'hd5;
  packet[8]=2;packet[9]=8'hac;packet[10]=8'h13;packet[11]=8'h80;packet[12]=0;packet[13]=1;
  packet[14]=2;packet[15]=8'hac;packet[16]=8'h13;packet[17]=8'h80;packet[18]=0;packet[19]=2;
  packet[20]=8'h88;packet[21]=body[lane*64]==32'h4e564131 ? 8'hb6 : 8'hb9;
  for(bi=0;bi<256;bi=bi+1)packet[22+bi]=(body[lane*64+bi/4]>>(24-8*(bi%4)))&255;
  crc16=16'hffff;for(bi=22;bi<278;bi=bi+1)begin crc16=crc16^{packet[bi],8'd0};for(bitno=0;bitno<8;bitno=bitno+1)crc16=crc16[15]?(crc16<<1)^16'h1021:crc16<<1;end
  packet[278]=crc16[15:8];packet[279]=crc16[7:0];crc32=32'hffffffff;
  for(bi=8;bi<280;bi=bi+1)begin crc32=crc32^packet[bi];for(bitno=0;bitno<8;bitno=bitno+1)crc32=crc32[0]?(crc32>>1)^32'hedb88320:crc32>>1;end
  crc32=~crc32;for(bi=0;bi<4;bi=bi+1)packet[280+bi]=crc32>>(8*bi);if(corrupt)packet[283]=packet[283]^1;
 end endtask
 task send_packet;integer idx;begin
  if(observing)begin
   frame_id=frame_id+1;
   $fdisplay(fd,"{\"frame_id\":%0d,\"event\":\"frame_meta\",\"time_ps\":%0.0f,\"kind\":%0d,\"round\":%0d,\"nav_epoch\":%0d}",frame_id,$realtime*1000,tx_kind,round_id,nav_epoch);
   event_log("measurement_begin");
  end
  for(idx=0;idx<284;idx=idx+1)drive_byte(busy_engine?emit_packet[idx]:packet[idx],1,idx);
  // Exactly 12 idle RX byte times = 96ns, no repeat()+next-call extra byte.
  for(idx=0;idx<12;idx=idx+1)drive_byte(0,0,idx==0?284:-1);
 end endtask
 task ing_csr(input[11:0]off,input[31:0]v);begin
  @(negedge clk);ing_bus_req=1;ing_bus_offset=off;ing_bus_wdata=v;ing_bus_wstrb=15;
  @(negedge clk);ing_bus_req=0;ing_bus_wstrb=0;
 end endtask
 task release_lane(input integer lane);integer t;begin
  if(pin_flags[lane*4])begin
   case(lane)0:ing_csr(12'h430,pin_tokens[31:0]);1:ing_csr(12'h450,pin_tokens[63:32]);2:ing_csr(12'h468,pin_tokens[95:64]);3:ing_csr(12'h484,pin_tokens[127:96]);endcase
   t=0;while((pin_flags[lane*4]||pin_flags[lane*4+2])&&t<200)begin @(negedge clk);t=t+1;end
   if(t==200)$fatal(1,"release timeout lane %d",lane);
  end
 end endtask
 task send_and_pin(input integer lane);integer t;begin
  make_frame(lane,0);
  if(busy_engine)begin submit_packet(3);end
  else begin @(negedge rx_clk);#1;send_packet;end
  repeat(12)@(negedge clk);
  if(lane!=0)case(lane)1:ing_csr(12'h440,1);2:ing_csr(12'h458,1);3:ing_csr(12'h480,1);endcase
  t=0;while(!pin_flags[lane*4]&&t<200)begin @(negedge clk);t=t+1;end
  if(t==200)$fatal(1,"pin timeout lane %d bad=%d busy=%d",lane,ingress.bad_frames,ingress.busy_frames);
 end endtask
 `include "owner_setup_supplement.svh"
 // One wire transmitter. Setup traffic and CANCEL background share the same
 // physical frame train; each frame has exactly 12 sampled idle byte slots.
 task submit_packet(input integer kind);integer q;begin
  wait(!pending_tx);for(q=0;q<284;q=q+1)queue_packet[q]=packet[q];
  pending_kind=kind;tx_done=0;pending_tx=1;wait(tx_done);
 end endtask
 initial begin : busy_sender
  integer q;reg queued;
  wait(busy_engine);@(negedge rx_clk);#1;
  while(!stream_stop)begin
   queued=pending_tx;
   tx_kind=queued?pending_kind:0;
   for(q=0;q<284;q=q+1)emit_packet[q]=queued?queue_packet[q]:bg_packet[q];
   send_packet;stream_count=stream_count+1;
   if(queued)begin pending_tx=0;tx_done=1;end
  end
 end
 task cancellation(input integer family);begin
  header(3,family?32'h4e564131:32'h4e415633,family?8'h03:8'h22,family?16:8);
  if(family)begin body64(3,24,nav_epoch);body[218]=7;end else body[216]=7;
  make_frame(3,0);
 end endtask

 integer checks_fd=0,checks_passed=0;
 task check(input string name,input bit pass);begin
  $fdisplay(checks_fd,"{\"check\":\"%s\",\"pass\":%s,\"time_ps\":%0.0f,\"frame_id\":%0d,\"cdc_generation\":%0d,\"outputs\":\"%03h\",\"owner_reason\":%0d,\"owner_event_id\":%0d,\"nav_epoch\":%0d,\"nav_live\":%0d,\"drive_live\":%0d}",name,pass?"true":"false",$realtime*1000,frame_id,cdc_generation,all_outputs,owner.event_reason,owner.event_id,nav_epoch,nav_live,drive_live);
  if(!pass)$fatal(1,"boundary check failed: %s",name);checks_passed=checks_passed+1;
 end endtask
 integer direction_observations=0;
 always @(all_outputs)if(observing&&reset_n)begin
  direction_observations=direction_observations+1;
  if((motor_in1&motor_in2)!=0)$fatal(1,"both direction pins high");
 end
 // MOT input uses the real decoder. req_sync[1] leads destination publication
 // by one CPU edge, so decoded_valid and CANCEL become true on the same edge.
 initial begin : mot_competitor
  wait(observing);
  if(key_case==4)begin
   @(posedge ingress.cancel_cross.req_sync[1]);
   command_data={32'h4d4f5431,8'd1,8'd0,32'd1,16'd0,16'd0,8'h0a,8'h0f};command_valid=1;
   @(posedge clk);#0.001;check("MOT_accept_and_cancel_same_cycle",mot_accept&&cancel_valid);
   check("MOT_combinational_priority_stops_all_outputs",all_outputs==0);
   @(negedge clk);command_valid=0;
  end
 end

 initial begin : experiment
  integer q,family;
  telemetry_ref_fd=$fopen("reference-status-stream.csv","w");telemetry_dut_fd=$fopen("candidate-status-stream.csv","w");telemetry_in_fd=$fopen("telemetry-input.jsonl","w");
  checks_fd=$fopen("checks.jsonl","w");fd=$fopen("events.jsonl","w");$dumpfile("trace.vcd");$dumpvars(0,tb_pipeline_measurement);
  fresh(1,1);wait(nav_live);wait(motors.cpu_active==4'hf);
  if(settle||busy_mode)repeat(5000)@(negedge clk);
  if(target_bit>=0)begin wait(!motor_pwm[target_bit]);wait(motor_pwm[target_bit]);end
  else if(low_phase)wait(motor_pwm==0);
  else wait(motor_pwm!=0);
  if(key_case>=6)begin
   observing=1;
   check("boundary_initial_navigation_active",nav_live&&drive_live&&all_outputs!=0);
   if(key_case==6)begin
    @(negedge clk);#3;reset_n=0;#0.001;
    check("reset_clears_registered_bundle_and_all_outputs",!drive_owned&&!drive_live&&drive_pwm==0&&drive_linear==0&&drive_angular==0&&all_outputs==0);
    repeat(4)@(posedge clk);@(negedge clk);reset_n=1;repeat(5)@(posedge clk);#0.001;
    check("reset_release_does_not_restore_old_motion",!nav_owned&&!drive_owned&&all_outputs==0);
   end else if(key_case==7)begin
    @(negedge clk);#3;raw_sonar_mm[15:0]=200;#0.001;
    check("front_raw_sonar_gate_not_delayed_by_drive_align",all_outputs==0&&drive_live);
    repeat(5)@(posedge clk);#0.001;check("near_sonar_remains_off",all_outputs==0);
   end else if(key_case==8)begin
    @(negedge clk);#3;cpu_trap=1;#0.001;
    check("trap_gate_not_delayed_by_drive_align",all_outputs==0&&drive_live);
    repeat(5)@(posedge clk);#0.001;check("trap_retires_authority",all_outputs==0&&!nav_owned&&!drive_live);
   end else if(key_case==9)begin
    pwm_sign=-1;commit_pwm;check("reverse_pwm_committed",result==0);
    // Production REVERSAL_MS=1 at 50MHz: wait beyond its 50,000-cycle deadtime.
    repeat(60000)@(negedge clk);
    check("reverse_runs_after_original_deadtime",motors.cpu_direction==4'hf&&motors.cpu_active==4'hf&&drive_linear[15]);
    check("reverse_never_drove_both_directions",direction_observations>0&&(motor_in1&motor_in2)==0);
   end
   repeat(5010)@(negedge clk);
  end else if(busy_mode)begin
   // A structurally valid CANCEL with unmatched goal generates real receiver
   // and CDC load, without a legal motion-side event. No design forcing.
   cancellation(0);for(q=8;q<12;q=q+1)body[192+q]=0;body[203]=32'hdeadbeef;
   make_frame(3,0);for(q=0;q<284;q=q+1)bg_packet[q]=packet[q];
   observing=1;busy_engine=1;
   for(round_id=0;round_id<2;round_id=round_id+1)begin
    if(round_id>0)begin
     test_run=test_run+1;test_goal=test_goal+1;test_plan=test_plan+1;
     install_task;if(result!=0)$fatal(1,"new task failed %d",result);
     accept_pose(2+round_id,device_ms+250);if(result!=0)$fatal(1,"pose failed %d",result);
     test_phone=test_phone+1;
     accept_permit(1+round_id,device_ms+500,device_ms+30000);if(result!=0)$fatal(1,"permit failed %d",result);
     claim_initial;if(result!=0)$fatal(1,"claim failed %d",result);
     commit_pwm;if(result!=0)$fatal(1,"PWM failed %d",result);
     wait(nav_live);wait(motors.cpu_active==4'hf);repeat(5000)@(negedge clk);
    end
    // At least eight real background frames immediately precede cancellation.
    q=stream_count;wait(stream_count>=q+8);
    family=round_id%2;cancellation(family);submit_packet(1);
    // Keep engine busy while requiring a full two-PWM-period stop hold.
    repeat(5100)@(negedge clk);
   end
   stream_stop=1;repeat(200)@(negedge clk);
  end else begin
   if(key_case==5)begin pwm_descriptor;write_p;end
   cancellation(mode==3);
   if(key_case==2)make_frame(3,1);
   if(key_case==3)begin body64(3,4,test_session+1);make_frame(3,0);end
   @(negedge rx_clk);#1;repeat(shift_bytes)drive_byte(0,0,-1);
   observing=1;tx_kind=(key_case==2||key_case==3)?0:1;send_packet;
   if(key_case==1)begin tx_kind=2;send_packet;end
   if(key_case==5)begin
    repeat(5)@(negedge clk);check("cancel_completed_before_old_PWM_attempt",!nav_owned&&all_outputs==0);
    wr(12'h4c0,p[0],15);await_result(12'h4c4);check("cancelled_run_old_PWM_rejected",result!=0&&!nav_live&&!drive_live&&all_outputs==0);
   end
   repeat(5010)@(negedge clk);
  end
  if(key_case==4)begin
   check("MOT_and_cancel_merge_one_event",owner.event_id==1&&owner.cancel_count==1&&owner.event_reason==3);
   check("MOT_takeover_has_no_old_navigation_resurrection",owner.owner==1&&!nav_live&&!drive_owned&&all_outputs==0);
  end
  check("no_direction_overlap_observed",direction_observations>0&&(motor_in1&motor_in2)==0);
  event_log("observation_end");
  // Stop the cancellation measurement before a separate telemetry exercise.
  // A real unarmed MOT1 stop traverses the actual motor decoder and generates
  // a real ACK pulse without changing STATUS_MS or the cancellation samples.
  observing=0;
  @(negedge clk);command_data={32'h4d4f5431,8'd1,8'd0,32'd100,16'd0,16'd0,8'h0a,8'h0f};command_valid=1;
  @(negedge clk);command_valid=0;
  repeat(200)@(negedge clk);
  check("real_motor_status_and_NAV1_consumer_exercised",telemetry_samples>0);
  $fclose(telemetry_ref_fd);$fclose(telemetry_dut_fd);$fclose(telemetry_in_fd);
  $display("MEASUREMENT_DONE phase=%0d sign=%0d mode=%0d busy=%0d live=%0d outputs=%03h reason=%0d cancel_count=%0d rejected=%0d bad_frames=%0d",phase_arg,pwm_sign,mode,busy_mode,nav_live,all_outputs,owner.event_reason,owner.cancel_count,owner.cancel_rejected,ingress.bad_frames);
  $fclose(checks_fd);$fclose(fd);$finish;
 end
 initial begin #6000000;$fatal(1,"measurement timeout");end
endmodule
