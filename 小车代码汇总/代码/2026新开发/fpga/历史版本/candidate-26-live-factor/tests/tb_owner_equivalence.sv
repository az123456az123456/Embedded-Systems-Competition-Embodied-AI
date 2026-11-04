`timescale 1ns/1ps
module tb_owner;
 reg clk=0;always #10 clk=~clk;
 reg reset_n=0;reg[31:0]device_ms=100;reg cpu_trap=0,mot_accept=0,manual_quiet=1;
 reg[47:0]raw_sonar_mm={16'd2000,16'd2000,16'd2000};reg[2:0]raw_sonar_online=7;
 reg bus_req=0;reg[11:0]bus_offset=0;reg[31:0]bus_wdata=0;reg[3:0]bus_wstrb=0;
 wire bus_rsp_valid;wire[31:0]bus_rsp_data;
 reg[127:0]pin_tokens=0,pin_rx_ms=0;reg[15:0]pin_flags=0;reg[35:0]pin_bytes=0;
 wire source_req;wire[1:0]source_lane;wire[5:0]source_word;
 wire source_grant;reg source_valid=0;reg[31:0]source_data=0;
 reg cancel_valid=0;reg[417:0]cancel_desc=0;wire cancel_ready;
 reg[31:0]last_tx_ticket=0;wire nav_owned,nav_live;wire[63:0]nav_pwm,nav_epoch;wire[15:0]nav_linear,nav_angular;wire[31:0]context_generation;
 nav_owner_gate dut(.*); wire ref_bus_rsp_valid;
 wire [31:0] ref_bus_rsp_data;
 wire ref_source_req;
 wire [1:0] ref_source_lane;
 wire [5:0] ref_source_word;
 wire ref_cancel_ready;
 wire ref_nav_owned;
 wire ref_nav_live;
 wire [63:0] ref_nav_pwm;
 wire [15:0] ref_nav_linear;
 wire [15:0] ref_nav_angular;
 wire [63:0] ref_nav_epoch;
 wire [31:0] ref_context_generation;
 nav_owner_gate_before reference_dut(.clk(clk),.reset_n(reset_n),.device_ms(device_ms),.cpu_trap(cpu_trap),.mot_accept(mot_accept),.manual_quiet(manual_quiet),.raw_sonar_mm(raw_sonar_mm),.raw_sonar_online(raw_sonar_online),.bus_req(bus_req),.bus_offset(bus_offset),.bus_wdata(bus_wdata),.bus_wstrb(bus_wstrb),.pin_tokens(pin_tokens),.pin_rx_ms(pin_rx_ms),.pin_flags(pin_flags),.pin_bytes(pin_bytes),.source_grant(source_grant),.source_valid(source_valid),.source_data(source_data),.cancel_valid(cancel_valid),.cancel_desc(cancel_desc),.last_tx_ticket(last_tx_ticket),.bus_rsp_valid(ref_bus_rsp_valid),.bus_rsp_data(ref_bus_rsp_data),.source_req(ref_source_req),.source_lane(ref_source_lane),.source_word(ref_source_word),.cancel_ready(ref_cancel_ready),.nav_owned(ref_nav_owned),.nav_live(ref_nav_live),.nav_pwm(ref_nav_pwm),.nav_linear(ref_nav_linear),.nav_angular(ref_nav_angular),.nav_epoch(ref_nav_epoch),.context_generation(ref_context_generation));
 integer compared_cycles=0;
 always @(posedge clk)begin #0.001;if(reset_n)begin compared_cycles=compared_cycles+1;
   if(bus_rsp_valid!==ref_bus_rsp_valid)$fatal(1,"cycle output mismatch bus_rsp_valid at %t",$time);
   if(bus_rsp_data!==ref_bus_rsp_data)$fatal(1,"cycle output mismatch bus_rsp_data at %t",$time);
   if(source_req!==ref_source_req)$fatal(1,"cycle output mismatch source_req at %t",$time);
   if(source_lane!==ref_source_lane)$fatal(1,"cycle output mismatch source_lane at %t",$time);
   if(source_word!==ref_source_word)$fatal(1,"cycle output mismatch source_word at %t",$time);
   if(cancel_ready!==ref_cancel_ready)$fatal(1,"cycle output mismatch cancel_ready at %t",$time);
   if(nav_owned!==ref_nav_owned)$fatal(1,"cycle output mismatch nav_owned at %t",$time);
   if(nav_live!==ref_nav_live)$fatal(1,"cycle output mismatch nav_live at %t",$time);
   if(nav_pwm!==ref_nav_pwm)$fatal(1,"cycle output mismatch nav_pwm at %t",$time);
   if(nav_linear!==ref_nav_linear)$fatal(1,"cycle output mismatch nav_linear at %t",$time);
   if(nav_angular!==ref_nav_angular)$fatal(1,"cycle output mismatch nav_angular at %t",$time);
   if(nav_epoch!==ref_nav_epoch)$fatal(1,"cycle output mismatch nav_epoch at %t",$time);
   if(context_generation!==ref_context_generation)$fatal(1,"cycle output mismatch context_generation at %t",$time);
   if(dut.owner!==reference_dut.owner)$fatal(1,"cycle state mismatch owner at %t",$time);
   if(dut.started!==reference_dut.started)$fatal(1,"cycle state mismatch started at %t",$time);
   if(dut.continuation!==reference_dut.continuation)$fatal(1,"cycle state mismatch continuation at %t",$time);
   if(dut.revoked!==reference_dut.revoked)$fatal(1,"cycle state mismatch revoked at %t",$time);
   if(dut.exhausted!==reference_dut.exhausted)$fatal(1,"cycle state mismatch exhausted at %t",$time);
   if(dut.epoch!==reference_dut.epoch)$fatal(1,"cycle state mismatch epoch at %t",$time);
   if(dut.ctx!==reference_dut.ctx)$fatal(1,"cycle state mismatch ctx at %t",$time);
   if(dut.event_id!==reference_dut.event_id)$fatal(1,"cycle state mismatch event_id at %t",$time);
   if(dut.event_reason!==reference_dut.event_reason)$fatal(1,"cycle state mismatch event_reason at %t",$time);
   if(dut.event_new_epoch!==reference_dut.event_new_epoch)$fatal(1,"cycle state mismatch event_new_epoch at %t",$time);
   if(dut.event_old_epoch!==reference_dut.event_old_epoch)$fatal(1,"cycle state mismatch event_old_epoch at %t",$time);
   if(dut.pose_deadline!==reference_dut.pose_deadline)$fatal(1,"cycle state mismatch pose_deadline at %t",$time);
   if(dut.permit_deadline!==reference_dut.permit_deadline)$fatal(1,"cycle state mismatch permit_deadline at %t",$time);
   if(dut.auth_deadline!==reference_dut.auth_deadline)$fatal(1,"cycle state mismatch auth_deadline at %t",$time);
   if(dut.pose_qualified!==reference_dut.pose_qualified)$fatal(1,"cycle state mismatch pose_qualified at %t",$time);
   if(dut.permit_qualified!==reference_dut.permit_qualified)$fatal(1,"cycle state mismatch permit_qualified at %t",$time);
   if(dut.auth_qualified!==reference_dut.auth_qualified)$fatal(1,"cycle state mismatch auth_qualified at %t",$time);
   if(dut.cancel_floor!==reference_dut.cancel_floor)$fatal(1,"cycle state mismatch cancel_floor at %t",$time);
   if(dut.nva_floor!==reference_dut.nva_floor)$fatal(1,"cycle state mismatch nva_floor at %t",$time);
   if(dut.last_ticket!==reference_dut.last_ticket)$fatal(1,"cycle state mismatch last_ticket at %t",$time);
   if(dut.auth_result!==reference_dut.auth_result)$fatal(1,"cycle state mismatch auth_result at %t",$time);
   if(dut.claim_result!==reference_dut.claim_result)$fatal(1,"cycle state mismatch claim_result at %t",$time);
   if(dut.pwm_result!==reference_dut.pwm_result)$fatal(1,"cycle state mismatch pwm_result at %t",$time);
   if(dut.revoke_result!==reference_dut.revoke_result)$fatal(1,"cycle state mismatch revoke_result at %t",$time);
   if(dut.hard_event!==reference_dut.hard_event)$fatal(1,"cycle state mismatch hard_event at %t",$time);
   if(dut.pose_fresh!==reference_dut.pose_fresh)$fatal(1,"cycle state mismatch pose_fresh at %t",$time);
   if(dut.permit_fresh!==reference_dut.permit_fresh)$fatal(1,"cycle state mismatch permit_fresh at %t",$time);
   if(dut.auth_fresh!==reference_dut.auth_fresh)$fatal(1,"cycle state mismatch auth_fresh at %t",$time);
 end end

 // This models only ingress's immutable read transport, never authorization.
 reg[31:0]body[0:255];reg pending=0,allow_read=1,hold_response=0;
 assign source_grant=source_req&&!pending&&allow_read;
 always @(posedge clk)begin
  source_valid<=pending&&!hold_response;if(pending&&!hold_response)pending<=0;
  if(source_grant)begin pending<=1;source_data<=body[source_lane*64+source_word];end
  if(!reset_n)begin pending<=0;source_valid<=0;end
 end
 reg[31:0]a[0:48],p[0:36];reg[31:0]ticket=0,token=0;reg[63:0]wire_seq=0;
 reg[31:0]boot_ms=100;reg[63:0]test_phone=1,test_auth=1;reg[63:0]test_session=17,test_run=2,test_map=1,test_rev=1,test_loc=1,test_plan=1;
 reg[127:0]test_goal=128'h0102030405060708090a0b0c0d0e0f10;
 integer fd,cases=0,failed=0,j,k,loop_time,field;reg[511:0]case_name;reg[63:0]saved_epoch,saved_event,saved_floor;reg[31:0]saved_ctx,saved_commit;reg[1023:0]outname;reg[31:0]r,result;
 task ck(input[511:0]name,input integer ok,input[63:0]actual);begin
  cases=cases+1;if(!ok)failed=failed+1;
  $fdisplay(fd,"{\"case\":\"%0s\",\"pass\":%s,\"actual\":%0d,\"device_ms\":%0d,\"epoch\":%0d,\"ctx\":%0d,\"owner\":%0d,\"started\":%0d,\"event_id\":%0d,\"event_mask\":%0d,\"nav_live\":%0d,\"pwm\":\"%0h\"}",name,ok?"true":"false",actual,device_ms,nav_epoch,context_generation,dut.owner,dut.started,dut.event_id,dut.event_reason,nav_live,nav_pwm);
  if(!ok)begin $display("OWNER_FAIL %0s actual=%0d epoch=%0d ctx=%0d result=%0d",name,actual,nav_epoch,context_generation,result);end
 end endtask
 task wr(input[11:0]offset,input[31:0]value,input[3:0]strobes);integer waits;begin
  @(negedge clk);bus_req=1;bus_offset=offset;bus_wdata=value;bus_wstrb=strobes;@(posedge clk);#1;
  if(!bus_rsp_valid)begin @(negedge clk);bus_req=0;bus_wstrb=0;end
  waits=0;while(!bus_rsp_valid&&waits<180)begin @(posedge clk);#1;waits=waits+1;end
  if(!bus_rsp_valid)$fatal(1,"no owner CSR write response");
  @(negedge clk);bus_req=0;bus_wstrb=0;
 end endtask
 task rd(input[11:0]offset,output[31:0]value);integer waits;begin
  @(negedge clk);bus_req=1;bus_offset=offset;bus_wstrb=0;@(posedge clk);#1;
  if(!bus_rsp_valid)begin @(negedge clk);bus_req=0;end
  waits=0;while(!bus_rsp_valid&&waits<180)begin @(posedge clk);#1;waits=waits+1;end
  if(!bus_rsp_valid)$fatal(1,"no owner CSR read response");value=bus_rsp_data;
  @(negedge clk);bus_req=0;
 end endtask
 task await_result(input[11:0]offset);integer count;begin
  result=2;count=0;while(result==2&&count<600)begin rd(offset,result);count=count+1;end
  if(result==2)$fatal(1,"owner operation did not finish");
 end endtask
 task put64(input integer n,input[63:0]v);begin a[n]=v[31:0];a[n+1]=v[63:32];end endtask
 task putp64(input integer n,input[63:0]v);begin p[n]=v[31:0];p[n+1]=v[63:32];end endtask
 task descriptor(input integer action);begin
  for(j=0;j<49;j=j+1)a[j]=0;ticket=ticket+1;a[0]=action;a[1]=ticket;put64(2,nav_epoch);put64(4,test_session);put64(6,test_run);
  a[8]=test_goal[127:96];a[9]=test_goal[95:64];a[10]=test_goal[63:32];a[11]=test_goal[31:0];put64(12,nav_epoch);put64(14,test_map);put64(16,test_rev);put64(18,test_loc);put64(20,test_plan);a[22]=1;a[43]=context_generation;
  a[23]=dut.pose_token;put64(24,dut.pose_seq);a[26]=dut.pose_deadline;a[27]=3;a[28]=60;a[29]=50;
  a[32]=dut.permit_token;put64(33,dut.permit_seq);a[35]=dut.permit_deadline;put64(36,dut.auth_epoch);put64(38,dut.phone_seq);a[40]=dut.auth_deadline;a[48]=32'h10203040;
 end endtask
 task write_a;begin for(j=0;j<49;j=j+1)wr(12'h900+j*4,a[j],15);end endtask
 task auth;begin write_a;wr(12'h4b0,a[1],15);await_result(12'h4b4);end endtask
 task body64(input integer lane,input integer n,input[63:0]v);begin body[lane*64+n]=v[63:32];body[lane*64+n+1]=v[31:0];end endtask
 task header(input integer lane,input[31:0]magic,input[7:0]op,input integer bytes);begin
  for(k=0;k<64;k=k+1)body[lane*64+k]=0;
  body[lane*64]=magic;body[lane*64+1]={8'd1,op,16'b0};wire_seq=wire_seq+1;body64(lane,2,wire_seq);body64(lane,4,test_session);body64(lane,6,test_run);
  body[lane*64+8]=test_goal[127:96];body[lane*64+9]=test_goal[95:64];body[lane*64+10]=test_goal[63:32];body[lane*64+11]=test_goal[31:0];
  body64(lane,12,test_map);body64(lane,14,test_rev);body64(lane,16,test_loc);body64(lane,18,test_plan);body64(lane,20,5000);body[lane*64+22]=bytes<<16;
  token=token+1;pin_tokens[lane*32+:32]=token;pin_rx_ms[lane*32+:32]=device_ms;pin_flags[lane*4+:4]=1;pin_bytes[lane*9+:9]=lane==0?256:160;
 end endtask
 task install_session;begin
  header(0,32'h4e415633,8'h03,16);body64(0,4,0);body64(0,24,test_session);body64(0,26,123);
  descriptor(1);put64(12,1);a[31]=pin_tokens[31:0];auth;
 end endtask
 task install_task;begin
  header(0,32'h4e415633,8'h20,24);body64(0,18,0);body[24]=1000;body[25]=50;body[26]=1;body64(0,27,1);
  descriptor(2);put64(12,nav_epoch+1);put64(24,1);a[31]=pin_tokens[31:0];auth;
 end endtask
 task accept_pose(input[63:0]id,input[31:0]deadline);begin
  header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);
  body[88]=60;body[89]=50;body[90]=0;body[91]=1;body64(1,28,id);body64(1,30,1000000000+id*1000000);body[96]=1;body[99]=deadline;body64(1,36,1);body[102]=3;
  descriptor(3);a[23]=pin_tokens[63:32];put64(24,id);a[26]=deadline;auth;
 end endtask
 task accept_permit(input[63:0]id,input[31:0]pd,input[31:0]ad);begin
  header(2,32'h4e564131,8'h02,64);body[152]=1;body64(2,25,nav_epoch);body64(2,27,test_auth);body64(2,29,test_phone);body64(2,31,id);body[161]=pd;body[162]=ad;body64(2,35,1);
  descriptor(4);a[32]=pin_tokens[95:64];put64(33,id);a[35]=pd;put64(36,test_auth);put64(38,test_phone);a[40]=ad;auth;
 end endtask
 task claim_initial;begin
  header(0,32'h4e415633,8'h21,24);body64(0,24,nav_epoch);body64(0,26,dut.permit_seq);body64(0,28,dut.auth_epoch);
  descriptor(4);a[31]=pin_tokens[31:0];a[42]=1;put64(44,5000);put64(46,{body[2],body[3]});write_a;wr(12'h4b8,a[1],15);await_result(12'h4bc);
 end endtask
 task pwm_descriptor;begin
  for(j=0;j<37;j=j+1)p[j]=0;ticket=ticket+1;p[0]=ticket;p[1]=2;putp64(2,test_session);putp64(4,test_run);
  p[6]=test_goal[127:96];p[7]=test_goal[95:64];p[8]=test_goal[63:32];p[9]=test_goal[31:0];
  putp64(10,nav_epoch);putp64(12,dut.pose_seq);putp64(14,dut.permit_seq);putp64(16,dut.auth_epoch);putp64(18,test_plan);putp64(20,test_map);putp64(22,test_rev);putp64(24,test_loc);p[26]=1;p[27]=3;
  for(j=28;j<32;j=j+1)p[j]=100;p[32]=220;p[33]=0;p[34]=dut.pose_token;p[35]=dut.permit_token;p[36]=context_generation;
 end endtask
 task write_p;begin for(j=0;j<37;j=j+1)wr(12'ha00+j*4,p[j],15);end endtask
 task commit_pwm;begin pwm_descriptor;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);end endtask
 task fresh(input integer start_now,input integer pwm_now);begin
  @(negedge clk);reset_n=0;cpu_trap=0;mot_accept=0;cancel_valid=0;bus_req=0;pin_tokens=0;pin_flags=0;pin_bytes=0;allow_read=1;hold_response=0;manual_quiet=1;device_ms=boot_ms;test_phone=1;test_auth=1;ticket=0;token=0;wire_seq=0;test_session=17;test_run=2;test_map=1;test_rev=1;test_loc=1;test_plan=1;raw_sonar_online=7;raw_sonar_mm={16'd2000,16'd2000,16'd2000};
  repeat(3)@(posedge clk);@(negedge clk);reset_n=1;repeat(2)@(posedge clk);
  install_session;if(result!=0)$fatal(1,"setup session %d",result);
  install_task;if(result!=0)$fatal(1,"setup task %d",result);
  accept_pose(2,device_ms+250);if(result!=0)$fatal(1,"setup pose %d",result);
  accept_permit(1,device_ms+500,device_ms+30000);if(result!=0)$fatal(1,"setup permit %d",result);
  if(start_now)begin claim_initial;if(result!=0)$fatal(1,"setup claim %d",result);end
  if(pwm_now)begin commit_pwm;if(result!=0)$fatal(1,"setup PWM %d",result);end
 end endtask

 task cancel_packet(input integer kind,input[63:0]seq);begin
  cancel_desc=0;cancel_desc[1:0]=kind;cancel_desc[65:2]=test_session;cancel_desc[129:66]=test_run;cancel_desc[257:130]=test_goal;cancel_desc[321:258]=seq;if(kind==2)cancel_desc[385:322]=nav_epoch;cancel_desc[417:386]=7;
 end endtask
 task pulse_cancel;begin @(negedge clk);cancel_valid=1;@(posedge clk);#1;@(negedge clk);cancel_valid=0;end endtask
 task pulse_mot;begin @(negedge clk);mot_accept=1;@(posedge clk);#1;@(negedge clk);mot_accept=0;end endtask
 task fence(input integer external_pin,input integer change_epoch);begin
  if(external_pin)begin header(0,32'h4e415633,8'h10,32);body64(0,6,0);for(k=8;k<12;k=k+1)body[k]=0;body64(0,16,0);body64(0,18,0);body[24]=1;body[27]=32'h00644040;body[28]=32'h10203040;if(change_epoch)body64(0,12,test_map+1);end
  descriptor(5);a[23]=0;a[31]=external_pin?pin_tokens[31:0]:0;a[41]=dut.continuation;if(change_epoch)put64(14,test_map+1);auth;
 end endtask
 task internal_task;begin descriptor(2);a[23]=0;a[31]=0;a[41]=dut.continuation;auth;end endtask
 task claim_resume;begin descriptor(4);a[31]=0;a[42]=2;a[41]=1;write_a;wr(12'h4b8,a[1],15);await_result(12'h4bc);end endtask
 initial begin
  if(!$value$plusargs("OUT=%s",outname))outname="owner-cases.jsonl";fd=$fopen(outname,"w");if(!fd)$fatal(1,"cannot open report");
  fresh(0,0);ck("plan_pose_permit_do_not_claim",!nav_owned&&!nav_live&&!dut.started,nav_epoch);
  claim_initial;ck("pinned_start_claims_without_existing_pwm",result==0&&nav_owned&&!nav_live,result);
  commit_pwm;ck("first_pwm_establishes_cpu_freshness",result==0&&nav_live&&nav_linear==220&&nav_pwm==64'h0064006400640064,result);
  accept_pose(3,350);ck("new_pose_preserves_last_valid_pwm",result==0&&nav_live&&nav_pwm!=0,result);
  accept_permit(2,600,30100);ck("new_permit_preserves_last_valid_pwm",result==0&&nav_live,result);
  wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("obsolete_pwm_cannot_replace_or_refresh",result!=0&&nav_live&&dut.commit_ms==100,result);
  device_ms=159;#1;ck("cpu_age59_live",nav_live,nav_live);device_ms=160;#1;ck("cpu_age60_zero",!nav_live&&!nav_pwm,nav_live);
  device_ms=161;commit_pwm;ck("new_pwm_recovers_after_cpu_stale_without_old_freshness",result==0&&nav_live,result);

  fresh(0,0);manual_quiet=0;claim_initial;ck("moving_manual_blocks_start",result==11&&!nav_owned,result);
  fresh(0,0);fence(0,0);ck("unstarted_internal_fence_cannot_create_qualification",result==10&&!dut.started&&!nav_owned,result);
  fresh(1,1);saved_epoch=nav_epoch;saved_event=dut.event_id;fence(1,0);ck("singlemap_external_map_begin_refused_without_side_effect",result!=0&&nav_live&&nav_owned&&dut.started&&nav_epoch==saved_epoch&&dut.event_id==saved_event,result);fence(0,0);ck("same_map_avoid_internal_fence",result==0&&!nav_live,result);
  test_plan=2;internal_task;ck("same_map_internal_new_plan_installs_while_fenced",result==0&&dut.map_revision==1&&dut.plan==2&&!nav_live,result);
  claim_resume;ck("same_run_internal_resume_needs_no_new_start",result==0&&dut.started&&nav_owned&&!nav_live,result);commit_pwm;ck("resume_new_pwm_restores_output",result==0&&nav_live,result);
  fresh(1,1);saved_epoch=nav_epoch;fence(1,1);ck("singlemap_external_cross_reference_map_refused",result!=0&&nav_owned&&dut.started&&nav_live&&nav_epoch==saved_epoch&&dut.event_reason==0,result);
  fresh(1,1);saved_epoch=nav_epoch;saved_floor=dut.cancel_floor;descriptor(5);a[23]=0;a[31]=0;a[41]=1;auth;ck("started_internal_fence_preserves_epoch_and_leases",result==0&&nav_epoch==saved_epoch&&dut.permit_qualified&&dut.auth_qualified&&dut.started&&dut.continuation,result);
  fresh(1,1);saved_epoch=nav_epoch;descriptor(0);a[23]=0;a[31]=0;write_a;wr(12'h4c8,a[1],15);await_result(12'h4cc);saved_event=dut.event_id;ck("software_revoke_emits_one_hardware_event",result==0&&dut.event_reason==4&&dut.event_id==1&&nav_epoch==saved_epoch+1&&!nav_live&&dut.cancel_floor==0,result);
  wr(12'h4c8,a[1],15);await_result(12'h4cc);ck("software_revoke_same_ticket_does_not_repeat_event",result!=0&&dut.event_id==saved_event,result);
  fresh(1,1);cancel_packet(1,1000);saved_epoch=nav_epoch;pulse_cancel;ck("network_cancel_immediate_owner_release",!nav_owned&&!nav_live&&nav_epoch==saved_epoch+1&&dut.event_reason==2&&dut.cancel_floor==1000,nav_epoch);
  saved_event=dut.event_id;pulse_cancel;ck("network_cancel_duplicate_idempotent",dut.event_id==saved_event&&dut.cancel_count==1,dut.event_id);
  commit_pwm;ck("cancelled_run_old_pwm_cannot_resume",result!=0&&!nav_live&&!dut.started,result);
  fresh(1,1);cancel_packet(2,1);cancel_desc[385:322]=nav_epoch+1;pulse_cancel;ck("wrong_epoch_revoke_does_not_retire_run",nav_live&&dut.event_id==0&&dut.cancel_floor==0,dut.cancel_rejected);
  cancel_packet(1,1);cancel_desc[65:2]=test_session+1;pulse_cancel;ck("wrong_session_cancel_does_not_consume_floor",nav_live&&dut.event_id==0&&dut.cancel_floor==0,dut.cancel_rejected);
  fresh(1,1);pwm_descriptor;write_p;saved_epoch=nav_epoch;@(negedge clk);mot_accept=1;bus_req=1;bus_offset=12'h4c0;bus_wdata=p[0];bus_wstrb=15;@(posedge clk);#1;@(negedge clk);mot_accept=0;bus_req=0;bus_wstrb=0;
  ck("actual_mot_accept_beats_same_cycle_pwm",dut.owner==1&&!nav_live&&nav_epoch==saved_epoch+1&&dut.event_reason==1&&dut.pwm_result==7,dut.pwm_result);
  fresh(0,0);header(0,32'h4e415633,8'h21,24);body64(0,24,nav_epoch);body64(0,26,dut.permit_seq);body64(0,28,dut.auth_epoch);descriptor(4);a[31]=pin_tokens[31:0];a[42]=1;put64(44,5000);put64(46,{body[2],body[3]});write_a;allow_read=0;wr(12'h4b8,a[1],15);repeat(5)@(posedge clk);cancel_packet(1,1);pulse_cancel;ck("cancel_ready_and_event_not_blocked_by_stalled_source",cancel_ready&&!dut.check_busy&&!nav_owned&&dut.event_reason==2,cancel_ready);
  allow_read=1;repeat(5)@(posedge clk);
  fresh(1,1);descriptor(3);a[23]=pin_tokens[63:32]+1;put64(24,3);auth;ck("wrong_pin_rejected_no_pose_or_epoch_change",result==4&&dut.pose_seq==2&&nav_live,result);
  fresh(1,1);header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);body[88]=61;body[89]=50;body[91]=1;body64(1,28,3);body[99]=350;body[102]=3;descriptor(3);a[23]=pin_tokens[63:32];put64(24,3);a[26]=350;auth;ck("cpu_pose_must_equal_original_pinned_pixel_position",result==6&&dut.pose_seq==2&&nav_live,result);
  fresh(1,1);descriptor(3);a[23]=pin_tokens[63:32];write_a;wr(12'h4b0,a[1],1);rd(12'h4b4,result);ck("partial_authority_strobe_rejected",result==3&&nav_live,result);
  fresh(1,1);pwm_descriptor;p[32]=301;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("physical_linear_above300_rejected",result==12&&nav_linear==220,result);
  pwm_descriptor;p[33]=1001;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("physical_angular_above1000_rejected",result==12&&nav_angular==0,result);
  pwm_descriptor;p[28]=-601;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("physical_pwm_below_minus600_rejected",result==12&&nav_pwm!=0,result);
  fresh(1,1);raw_sonar_mm[15:0]=200;#1;ck("raw_front_200mm_instant_output_floor",!nav_live&&nav_pwm==0,nav_live);raw_sonar_mm[15:0]=201;#1;ck("raw_front201mm_does_not_retire_or_block",nav_live&&dut.event_id==0,nav_live);
  fresh(1,1);device_ms=349;accept_pose(3,599);commit_pwm;device_ms=599;accept_pose(4,849);commit_pwm;ck("motion499ms_still_valid",nav_live&&dut.started,nav_live);device_ms=600;commit_pwm;ck("motion500ms_deadline_inclusive",result==0&&nav_live&&dut.started,result);device_ms=601;repeat(2)@(posedge clk);#1;ck("motion501ms_hard_retires",!nav_live&&!dut.started&&(dut.event_reason&8)!=0,dut.event_reason);
  fresh(1,1);device_ms=349;commit_pwm;ck("pose249ms_still_valid",result==0&&nav_live,result);device_ms=350;commit_pwm;ck("pose250ms_deadline_inclusive",result==0&&nav_live,result);device_ms=351;repeat(2)@(posedge clk);#1;ck("pose251ms_hard_retires",!nav_live&&!dut.started&&(dut.event_reason&32)!=0,dut.event_reason);

  fresh(1,1);saved_ctx=context_generation;saved_epoch=nav_epoch;accept_pose(2,350);ck("duplicate_pose_cannot_extend_deadline",result==8&&dut.pose_deadline==350&&context_generation==saved_ctx&&nav_epoch==saved_epoch,result);
  fresh(1,1);accept_permit(2,600,30101);ck("same_phone_sequence_cannot_extend_authorization",result==8&&dut.auth_deadline==30100&&dut.permit_seq==1,result);
  test_phone=2;device_ms=101;accept_permit(2,600,30101);ck("new_phone_source_can_extend_authorization",result==0&&dut.auth_deadline==30101&&dut.phone_seq==2,result);
  fresh(1,1);descriptor(3);a[23]=pin_tokens[63:32];put64(24,3);write_a;allow_read=0;wr(12'h4b0,a[1],15);saved_ctx=context_generation;wr(12'h970,32'hdeadbeef,15);rd(12'h970,r);ck("busy_source_check_freezes_authority_shadow",r==60&&context_generation==saved_ctx,r);
  saved_epoch=nav_epoch;cpu_trap=1;repeat(2)@(posedge clk);#1;ck("trap_preempts_busy_checker_without_grant",!nav_live&&!dut.check_busy&&nav_epoch==saved_epoch+1&&dut.event_reason==64&&cancel_ready,dut.event_reason);cpu_trap=0;allow_read=1;repeat(5)@(posedge clk);
  fresh(1,1);header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);body[88]=60;body[89]=50;body[91]=1;body64(1,28,3);body[99]=350;body[102]=3;descriptor(3);a[23]=pin_tokens[63:32];put64(24,3);a[26]=350;write_a;wr(12'h4b0,a[1],15);repeat(25)@(posedge clk);pin_tokens[63:32]=pin_tokens[63:32]+1;await_result(12'h4b4);ck("pin_rechecked_after_complete_original_stream",result==4&&dut.pose_seq==2&&nav_live,result);
  fresh(1,1);pwm_descriptor;write_p;wr(12'h4c0,p[0],3);rd(12'h4c4,result);ck("partial_pwm_commit_rejected",result==3&&dut.commit_ms==100,result);
  fresh(1,1);pwm_descriptor;p[36]=p[36]+1;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("pwm_final_context_generation_checked",result==7&&nav_live&&dut.commit_ms==100,result);
  fresh(1,1);pwm_descriptor;p[34]=p[34]+1;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("pwm_pose_pin_identity_rechecked",result==8&&nav_live,result);
  fresh(1,1);pwm_descriptor;p[35]=p[35]+1;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("pwm_permit_pin_identity_rechecked",result==8&&nav_live,result);
  fresh(1,1);pwm_descriptor;p[32]=-300;p[33]=-1000;p[28]=-600;p[29]=600;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("physical_ranges_include_both_endpoints",result==0&&nav_live&&$signed(nav_linear)==-300&&$signed(nav_angular)==-1000,result);
  raw_sonar_mm[47:32]=200;#1;ck("raw_right_floor_directional_during_clockwise",!nav_live&&!nav_pwm,nav_live);raw_sonar_mm[47:32]=201;#1;ck("raw_right_floor_release_keeps_run",nav_live&&dut.started,nav_live);
  fresh(1,1);pwm_descriptor;p[32]=0;p[33]=500;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);raw_sonar_mm[31:16]=200;#1;ck("raw_left_floor_directional_during_counterclockwise",!nav_live,nav_live);raw_sonar_mm[31:16]=201;raw_sonar_online=6;#1;ck("offline_sonar_blocks_hardware_output",!nav_live&&dut.started,nav_live);
  fresh(1,1);saved_epoch=nav_epoch;device_ms=100;cpu_trap=1;mot_accept=1;cancel_packet(1,1);cancel_valid=1;@(posedge clk);#1;ck("same_cycle_hard_causes_merge_single_epoch_event",dut.event_id==1&&nav_epoch==saved_epoch+1&&dut.event_reason==67&&dut.owner==1&&!nav_live,dut.event_reason);@(negedge clk);mot_accept=0;cancel_valid=0;repeat(3)@(posedge clk);#1;ck("held_trap_does_not_duplicate_event",dut.event_id==1,dut.event_id);
  fresh(1,1);for(loop_time=300;loop_time<30099;loop_time=loop_time+200)begin device_ms=loop_time;accept_pose(dut.pose_seq+1,loop_time+250);accept_permit(dut.permit_seq+1,loop_time+500,30100);commit_pwm;if(result!=0)$fatal(1,"auth boundary setup %d",result);end
  device_ms=30099;accept_pose(dut.pose_seq+1,30349);accept_permit(dut.permit_seq+1,30599,30100);commit_pwm;ck("authorization29999ms_live",result==0&&nav_live,result);device_ms=30100;repeat(2)@(posedge clk);#1;ck("authorization30000ms_hard_expires",!nav_live&&!dut.started&&(dut.event_reason&16)!=0,dut.event_reason);saved_event=dut.event_id;device_ms=30101;repeat(2)@(posedge clk);#1;ck("authorization30001ms_no_repeat_or_resume",!nav_live&&dut.event_id==saved_event,dut.event_id);
  boot_ms=32'hffffff9b;fresh(1,1);device_ms=32'h00000094;commit_pwm;ck("pose_tickwrap_age249_still_valid",result==0&&nav_live,result);device_ms=32'h00000095;commit_pwm;ck("pose_tickwrap_age250_still_valid",result==0&&nav_live,result);device_ms=32'h00000096;repeat(2)@(posedge clk);#1;ck("pose_tickwrap_age251_retires",!nav_live&&(dut.event_reason&32)!=0,dut.event_reason);boot_ms=100;
  // Deliberate test-only counter fault injection, not a CPU write interface.
  fresh(1,1);@(negedge clk);dut.epoch=64'hfffffffffffffffe;reference_dut.epoch=64'hfffffffffffffffe;cancel_packet(1,1);pulse_cancel;ck("epoch_saturates_and_exhaustion_blocks_run",nav_epoch==64'hffffffffffffffff&&dut.exhausted&&!nav_live&&(dut.event_reason&256)!=0,nav_epoch);test_session=18;install_session;ck("genuine_new_session_resets_exhausted_epoch_to_one",result==0&&nav_epoch==1&&!dut.exhausted&&dut.event_id==0&&dut.session==18,result);
  // Known contract ambiguity retained as a real failing two-family sample.
  fresh(1,1);cancel_packet(1,1000);pulse_cancel;test_run=3;install_task;accept_pose(3,350);accept_permit(2,600,30100);claim_initial;commit_pwm;cancel_packet(2,wire_seq+1);pulse_cancel;
  ck("independent_nva1_revoke_after_large_nav3_cancel",!nav_live&&!dut.started&&dut.event_reason==4,dut.cancel_floor);

  ck("nav3_cancel_floor_and_nva_floor_are_distinct",dut.cancel_floor==1000&&dut.nva_floor==wire_seq+1,dut.nva_floor);
  fresh(1,1);wire_seq=9000;accept_permit(2,600,30100);commit_pwm;ck("high_nva_permit_transport_is_accepted",result==0&&dut.nva_floor==9001&&nav_live,dut.nva_floor);cancel_packet(1,1);pulse_cancel;ck("high_nva_permit_does_not_block_low_nav3_cancel",!nav_live&&!dut.started&&dut.event_reason==2&&dut.cancel_floor==1&&dut.nva_floor==9001,dut.cancel_floor);
  fresh(1,1);saved_floor=dut.nva_floor;cancel_packet(2,90000);cancel_desc[65:2]=test_session+1;pulse_cancel;ck("wrong_session_revoke_cannot_poison_nva_floor",nav_live&&dut.nva_floor==saved_floor,dut.nva_floor);
  cancel_packet(2,90000);cancel_desc[129:66]=test_run+1;pulse_cancel;ck("wrong_run_revoke_cannot_poison_nva_floor",nav_live&&dut.nva_floor==saved_floor,dut.nva_floor);
  cancel_packet(1,90000);cancel_desc[129:66]=test_run+1;pulse_cancel;ck("wrong_run_cancel_cannot_poison_nav3_floor",nav_live&&dut.cancel_floor==0,dut.cancel_floor);
  cancel_packet(2,saved_floor);pulse_cancel;ck("revoke_same_transport_as_accepted_permit_is_duplicate",nav_live&&dut.event_id==0&&dut.nva_floor==saved_floor,dut.nva_floor);
  cancel_packet(2,saved_floor+1);pulse_cancel;ck("new_revoke_advances_only_nva_floor",!nav_live&&dut.event_reason==4&&dut.nva_floor==saved_floor+1&&dut.cancel_floor==0,dut.nva_floor);
  test_session=18;install_session;ck("new_session_resets_both_family_floors",result==0&&dut.nva_floor==0&&dut.cancel_floor==0&&nav_epoch==1,result);install_task;accept_pose(2,350);accept_permit(1,600,30100);claim_initial;commit_pwm;saved_floor=dut.nva_floor;
  cancel_packet(2,64'h7fffffffffffffff);cancel_desc[65:2]=17;pulse_cancel;ck("old_session_huge_revoke_still_rejected_after_reset",nav_live&&dut.nva_floor==saved_floor&&dut.event_id==0,dut.nva_floor);
  cancel_packet(1,64'h7fffffffffffffff);cancel_desc[65:2]=17;pulse_cancel;ck("old_session_huge_cancel_still_rejected_after_reset",nav_live&&dut.cancel_floor==0&&dut.event_id==0,dut.cancel_floor);
  // Raw packet retry with a forged new permit sequence cannot reuse NVA transport.
  fresh(1,1);saved_floor=dut.nva_floor;wire_seq=saved_floor-1;accept_permit(2,600,30100);ck("new_permit_id_cannot_reuse_old_transport",result==8&&dut.permit_seq==1&&dut.nva_floor==saved_floor,result);
  // Accepted request/pin stays fixed across a stalled read, then old response drains.
  fresh(1,1);descriptor(3);a[23]=pin_tokens[63:32];put64(24,3);write_a;wr(12'h4b0,a[1],15);wait(dut.read_wait);cancel_packet(1,1);pulse_cancel;repeat(4)@(posedge clk);#1;ck("cancel_drains_inflight_read_and_retires_once",!dut.check_busy&&!dut.draining&&!nav_live&&dut.event_id==1,dut.event_id);
  fresh(1,1);descriptor(3);write_a;allow_read=0;wr(12'h4b0,a[1],15);repeat(1050)@(posedge clk);rd(12'h4b4,result);ck("source_arbitration_timeout_is_bounded_and_fail_closed",result==2&&!dut.check_busy&&!source_req&&nav_live,result);allow_read=1;
  fresh(1,1);descriptor(3);write_a;wr(12'h9c4,1,15);wr(12'h4b0,a[1],15);await_result(12'h4b4);ck("nonzero_reserved_shadow_rejects_authority",result==13&&nav_live,result);
  fresh(1,1);pwm_descriptor;write_p;wr(12'ha94,1,15);wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("nonzero_reserved_shadow_rejects_pwm",result==13&&nav_live,result);
  // Rebinding a different session cannot claim while manual motion is active.
  fresh(0,0);test_session=18;manual_quiet=0;install_session;ck("session_bind_requires_actual_external_quiescence",result==11&&dut.session==17,result);

  fresh(1,1);descriptor(3);put64(24,3);write_a;hold_response=1;wr(12'h4b0,a[1],15);repeat(1050)@(posedge clk);rd(12'h4b4,result);ck("read_response_timeout_keeps_old_response_in_drain",result==2&&!dut.check_busy&&dut.draining&&dut.pose_seq==2&&cancel_ready,result);
  wr(12'h4b0,a[1],15);rd(12'h4b4,result);ck("draining_rejects_new_operation_without_reusing_read",result==2&&!dut.check_busy&&dut.draining,dut.draining);hold_response=0;repeat(5)@(posedge clk);#1;ck("delayed_response_consumed_only_by_drain",!dut.draining&&dut.pose_seq==2&&!dut.check_busy,dut.draining);accept_pose(3,350);ck("new_valid_operation_after_drain_has_own_stream",result==0&&dut.pose_seq==3,result);
  // The original START body is independently checked word-by-word, including IDs.
  for(field=2;field<=29;field=field+1)begin
   if(field!=22&&field!=23)begin
    fresh(0,0);header(0,32'h4e415633,8'h21,24);body64(0,24,nav_epoch);body64(0,26,dut.permit_seq);body64(0,28,dut.auth_epoch);
    descriptor(0);a[31]=pin_tokens[31:0];a[42]=1;put64(44,5000);put64(46,{body[2],body[3]});body[field]=body[field]^1;write_a;wr(12'h4b8,a[1],15);await_result(12'h4bc);
    $sformat(case_name,"original_start_word_%0d_mismatch_no_claim",field);ck(case_name,result==6&&!nav_owned&&!dut.started,result);
   end
  end
  fresh(0,0);header(0,32'h4e415633,8'h21,24);body64(0,24,nav_epoch);body64(0,26,dut.permit_seq);body64(0,28,dut.auth_epoch);descriptor(0);a[31]=pin_tokens[31:0];a[42]=1;put64(44,5000);put64(46,{body[2],body[3]});write_a;wr(12'h4b8,a[1],15);wait(dut.read_wait&&source_valid&&source_word==29);@(negedge clk);mot_accept=1;@(posedge clk);#1;ck("mot_accept_beats_final_start_source_response",dut.owner==1&&!dut.started&&dut.claim_result==7&&dut.event_reason==1,dut.claim_result);@(negedge clk);mot_accept=0;
  fresh(0,0);header(0,32'h4e415633,8'h21,24);body64(0,24,nav_epoch);body64(0,26,dut.permit_seq);body64(0,28,dut.auth_epoch);descriptor(0);a[31]=pin_tokens[31:0];a[42]=1;put64(44,5000);put64(46,{body[2],body[3]});write_a;wr(12'h4b8,a[1],15);wait(dut.read_wait&&source_valid&&source_word==29);@(negedge clk);cancel_packet(1,1);cancel_valid=1;@(posedge clk);#1;ck("cancel_beats_final_start_source_response",!nav_owned&&!dut.started&&dut.claim_result==7&&dut.event_reason==2,dut.claim_result);@(negedge clk);cancel_valid=0;
  fresh(1,1);@(negedge clk);dut.event_id=64'hfffffffffffffffe;reference_dut.event_id=64'hfffffffffffffffe;pulse_mot;ck("event_id_saturates_without_wrap",dut.event_id==64'hffffffffffffffff&&dut.exhausted&&!nav_live&&(dut.event_reason&256)!=0,dut.event_id);pulse_mot;ck("exhausted_event_id_stays_maximum",dut.event_id==64'hffffffffffffffff&&dut.exhausted,dut.event_id);test_session=18;install_session;ck("new_session_resets_exhausted_event_counter",result==0&&dut.event_id==0&&nav_epoch==1&&!dut.exhausted,result);

  for(field=0;field<4;field=field+1)begin
   fresh(1,0);pwm_descriptor;p[27]=field;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);$sformat(case_name,"nav_atomic_ready_flags_%0d_physical_gate",field);ck(case_name,result==0&&(field==3?nav_live&&nav_pwm!=0:!nav_live&&nav_pwm==0),nav_pwm);
  end
  fresh(1,1);pwm_descriptor;p[27]=0;for(j=28;j<34;j=j+1)p[j]=0;write_p;wr(12'h4c0,p[0],15);await_result(12'h4c4);ck("notready_zero_commit_establishes_timestamp_without_motion",result==0&&dut.pwm_valid&&!nav_live&&dut.commit_ms==device_ms,result);commit_pwm;ck("own_nav_ready_commit_can_enable_output",result==0&&nav_live,result);pulse_mot;ck("mot_still_preempts_ready_nav_bank",!nav_live&&dut.owner==1,dut.owner);

  fresh(1,1);test_run=3;test_rev=2;install_task;ck("singlemap_running_new_run_cannot_change_map",result==11&&dut.map_revision==1&&dut.run==2&&nav_live,result);
  fresh(1,1);fence(0,0);test_rev=2;test_plan=2;descriptor(2);a[31]=0;a[41]=1;auth;ck("singlemap_internal_replan_cannot_change_map_generation",result==6&&dut.plan==1&&dut.map_revision==1&&!nav_live,result);test_rev=1;internal_task;ck("runtime_same_map_task_keeps_accepted_pose_token",result==0&&dut.plan==2&&dut.map_revision==1,result);
  // User-approved C guard NE_REBIND reference fence, same map epochs.
  fresh(1,1);header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);body[88]=61;body[89]=50;body[91]=1;body64(1,28,3);body64(1,30,900000000);body[96]=2;body[99]=350;body[102]=3;descriptor(5);a[23]=pin_tokens[63:32];a[31]=0;put64(24,3);a[28]=61;a[26]=350;saved_epoch=nav_epoch;auth;ck("pose_source_reference_reset_same_epochs_must_retire",result==0&&!dut.started&&!nav_owned&&nav_epoch==saved_epoch+1&&dut.event_reason==128,nav_epoch);

  fresh(1,1);header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);body[88]=60;body[89]=50;body[91]=1;body64(1,28,3);body64(1,30,900000000);body[96]=1;body[99]=350;body[102]=3;descriptor(5);a[23]=pin_tokens[63:32];a[31]=0;put64(24,3);a[26]=350;saved_epoch=nav_epoch;saved_commit=dut.pose_deadline;auth;ck("regressed_source_stamp_reference_fence_retires_without_refresh",result==0&&!dut.started&&nav_epoch==saved_epoch+1&&!dut.pose_qualified&&!dut.permit_qualified&&!dut.auth_qualified&&dut.pose_deadline==saved_commit&&dut.pose_seq==2,result);
  fresh(1,1);descriptor(5);a[31]=0;a[23]=dut.pose_token;a[41]=1;saved_epoch=nav_epoch;auth;ck("pose_fence_without_semantic_reset_marker_rejected",result==10&&nav_live&&nav_epoch==saved_epoch&&dut.started,result);
  fresh(0,0);test_session=18;header(0,32'h4e415633,8'h03,16);body64(0,4,0);body64(0,24,test_session);body64(0,26,123);descriptor(1);a[0]=257;put64(12,1);a[31]=pin_tokens[31:0];auth;ck("authority_action_upper_bits_cannot_alias_install_session",result==13&&dut.session==17,result);

  fresh(1,1);header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);body[88]=60;body[89]=50;body[91]=2;body64(1,28,3);body[99]=350;body[102]=3;descriptor(5);a[23]=pin_tokens[63:32];a[31]=0;put64(24,3);a[26]=350;saved_epoch=nav_epoch;saved_commit=dut.pose_deadline;auth;ck("rejected_raw_frame_reference_fence_stops_without_accepting_frame",result==0&&!nav_owned&&!dut.started&&nav_epoch==saved_epoch+1&&dut.event_reason==128&&dut.frame_token==1&&dut.pose_deadline==saved_commit&&dut.pose_seq==2,result);
  fresh(1,1);header(1,32'h4e415633,8'h30,60);body64(1,6,0);for(k=8;k<12;k=k+1)body[64+k]=0;body64(1,18,0);body[88]=60;body[89]=50;body[91]=2;body64(1,28,3);body[99]=350;body[102]=3;descriptor(3);a[23]=pin_tokens[63:32];put64(24,3);a[26]=350;saved_epoch=nav_epoch;auth;ck("ordinary_accept_pose_still_strictly_checks_original_frame",result==6&&nav_live&&dut.frame_token==1&&dut.pose_seq==2&&nav_epoch==saved_epoch,result);
  $fdisplay(fd,"{\"summary\":true,\"cases\":%0d,\"failed\":%0d}",cases,failed);$display("OWNER_CYCLE_EQ_PASS cycles=%0d",compared_cycles);$fclose(fd);$display("OWNER_RESULT cases=%0d failed=%0d",cases,failed);if(failed)$fatal(1,"owner tests");$finish;
 end
 initial begin #100000000;$fatal(1,"owner TB timeout");end
endmodule
