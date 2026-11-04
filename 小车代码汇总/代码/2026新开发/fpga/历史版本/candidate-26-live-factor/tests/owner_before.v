// Atomic NAV3 owner gate. Authority: docs/NAV3_MMIO_contract.txt.
// Original packets remain immutable in ingress. CPU shadow words live in a
// synchronous BSRAM; a bounded word consumer stages only commit data.
module nav_owner_gate_before #(
    parameter integer CHECK_TIMEOUT_CYCLES=1024
)(
    input wire clk,reset_n,input wire [31:0] device_ms,
    input wire cpu_trap,mot_accept,manual_quiet,
    input wire [47:0] raw_sonar_mm,input wire [2:0] raw_sonar_online,
    input wire bus_req,input wire [11:0] bus_offset,
    input wire [31:0] bus_wdata,input wire [3:0] bus_wstrb,
    output reg bus_rsp_valid,output reg [31:0] bus_rsp_data,
    input wire [127:0] pin_tokens,pin_rx_ms,input wire [15:0] pin_flags,
    input wire [35:0] pin_bytes,
    output reg source_req,output reg [1:0] source_lane,output reg [5:0] source_word,
    input wire source_grant,source_valid,input wire [31:0] source_data,
    input wire cancel_valid,input wire [417:0] cancel_desc,output wire cancel_ready,
    input wire [31:0] last_tx_ticket,
    output wire nav_owned,nav_live,output wire [63:0] nav_pwm,
    output wire [15:0] nav_linear,nav_angular,output wire [63:0] nav_epoch,
    output wire [31:0] context_generation
);
 localparam OK=0,EMPTY=1,BUSY=2,BAD_STROBE=3,BAD_TOKEN=4,BAD_TICKET=5,
            IDENTITY=6,OLD_EPOCH=7,OLD_SOURCE=8,EXPIRED=9,NO_START=10,
            ACTIVE_OWNER=11,BAD_RANGE=12,RESERVED=13;
 localparam INSTALL_SESSION=1,INSTALL_TASK=2,ACCEPT_POSE=3,ACCEPT_PERMIT=4,MAP_FENCE=5;
 localparam OP_AUTH=1,OP_CLAIM=2,OP_REVOKE=3,OP_PWM=4;
 localparam EV_MOT=32'h1,EV_CANCEL=32'h2,EV_REVOKE=32'h4,EV_MOTION=32'h8,
            EV_AUTH=32'h10,EV_POSE=32'h20,EV_TRAP=32'h40,EV_REFERENCE=32'h80,EV_EXHAUST=32'h100;
 localparam [63:0] U64MAX=64'hffffffffffffffff;

 reg auth_reserved,pwm_reserved;
 reg [31:0] auth_result,claim_result,pwm_result,revoke_result,last_ticket;
 reg [63:0] session,run;
 // Keep the safety epoch as one logical state value while allowing synthesis
 // to replicate its read fanout for the independent validation cones.
 (* syn_maxfan=64 *) reg [63:0] epoch;
 reg [63:0] run_floor,plan;
 reg [31:0] map_generation;
 localparam [63:0] map_epoch=1,localization=1;
 wire [63:0] map_revision={32'b0,map_generation};
 reg [127:0] goal;
 reg [31:0] frame_token,active_crc,ctx,hw_generation;
 reg [63:0] status_generation;
 reg bound,task_valid,plan_ready,started,continuation,revoked,map_fenced,exhausted;
 reg [1:0] owner;
 reg pose_qualified,permit_qualified,auth_qualified,pwm_valid;
 reg [63:0] pose_seq,permit_seq,auth_epoch,phone_seq;
 reg [31:0] pose_deadline,permit_deadline,auth_deadline,pose_rx_ms;
 reg [31:0] pose_token,permit_token,commit_ms,last_pwm_ticket;
 reg [31:0] pose_x,pose_y,pose_yaw,pwm_flags;
 reg [63:0] pwm;
 reg [15:0] linear,angular;
 reg [63:0] cancel_floor,nva_floor,source_transport,event_id,event_old_epoch,event_new_epoch;
 wire [63:0] event_session=session;
 reg [31:0] event_reason,event_context,event_generation,last_revoke_reason;
 reg event_valid,trap_previous;
 reg [31:0] cancel_count,cancel_rejected;
 reg context_race;
 reg check_busy,read_wait,draining,source_bad;
 reg [2:0] operation;
 reg [2:0] action;
 reg [31:0] operation_ticket,operation_ctx,operation_token;
 reg [5:0] check_words;
 localparam integer CHECK_CYCLES_BITS=(CHECK_TIMEOUT_CYCLES>0)?$clog2(CHECK_TIMEOUT_CYCLES+1):1;
 reg [CHECK_CYCLES_BITS-1:0] check_cycles;
 integer i;
 // Keep saturation reductions balanced and independent of increment carry.
 wire epoch_full,epoch_near_full,event_full,event_near_full;
 nav_eq_reduce_lut4 #(.N(64)) epoch_all(.v(epoch),.q(epoch_full));
 nav_eq_reduce_lut4 #(.N(63)) epoch_last_two(.v(epoch[63:1]),.q(epoch_near_full));
 nav_eq_reduce_lut4 #(.N(64)) event_all(.v(event_id),.q(event_full));
 nav_eq_reduce_lut4 #(.N(63)) event_last_two(.v(event_id[63:1]),.q(event_near_full));
 wire eq_cancel_session,ne_cancel_session;
 wire eq_cancel_run,ne_cancel_run;
 wire eq_cancel_goal,ne_cancel_goal;
 wire eq_cancel_epoch,ne_cancel_epoch;
 wire eq_cancel_duplicate,ne_cancel_duplicate;
 wire eq_source_word,ne_source_word;
 reg eq_a_ticket; wire ne_a_ticket=~eq_a_ticket;
 reg eq_a_old_epoch; wire ne_a_old_epoch=~eq_a_old_epoch;
 reg eq_a_context; wire ne_a_context=~eq_a_context;
 wire eq_op_context,ne_op_context;
 reg eq_a_session; wire ne_a_session=~eq_a_session;
 reg eq_a_run; wire ne_a_run=~eq_a_run;
 reg eq_a_goal; wire ne_a_goal=~eq_a_goal;
 reg eq_a_epoch; wire ne_a_epoch=~eq_a_epoch;
 reg eq_a_map_revision; wire ne_a_map_revision=~eq_a_map_revision;
 reg eq_a_plan; wire ne_a_plan=~eq_a_plan;
 reg eq_a_frame; wire ne_a_frame=~eq_a_frame;
 reg eq_a_pose_seq; wire ne_a_pose_seq=~eq_a_pose_seq;
 reg eq_a_pose_token; wire ne_a_pose_token=~eq_a_pose_token;
 reg eq_a_permit_seq; wire ne_a_permit_seq=~eq_a_permit_seq;
 reg eq_a_permit_token; wire ne_a_permit_token=~eq_a_permit_token;
 reg eq_a_auth_epoch; wire ne_a_auth_epoch=~eq_a_auth_epoch;
 reg eq_a_phone_seq; wire ne_a_phone_seq=~eq_a_phone_seq;
 reg eq_p_epoch; wire ne_p_epoch=~eq_p_epoch;
 reg eq_p_context; wire ne_p_context=~eq_p_context;
 reg eq_p_session; wire ne_p_session=~eq_p_session;
 reg eq_p_run; wire ne_p_run=~eq_p_run;
 reg eq_p_goal; wire ne_p_goal=~eq_p_goal;
 reg eq_p_plan; wire ne_p_plan=~eq_p_plan;
 reg eq_p_map_revision; wire ne_p_map_revision=~eq_p_map_revision;
 reg eq_p_frame; wire ne_p_frame=~eq_p_frame;
 reg eq_p_pose_seq; wire ne_p_pose_seq=~eq_p_pose_seq;
 reg eq_p_permit_seq; wire ne_p_permit_seq=~eq_p_permit_seq;
 reg eq_p_auth_epoch; wire ne_p_auth_epoch=~eq_p_auth_epoch;
 reg eq_p_pose_token; wire ne_p_pose_token=~eq_p_pose_token;
 reg eq_p_permit_token; wire ne_p_permit_token=~eq_p_permit_token;

 function [31:0] merged;input [31:0] oldv,newv;input [3:0] lanes;integer k;begin
  merged=oldv;for(k=0;k<4;k=k+1)if(lanes[k])merged[k*8+:8]=newv[k*8+:8];end endfunction
 function past;
  input [31:0] deadline,now;
  reg [7:0] less4,equal4;
  reg [3:0] less8,equal8;
  reg [1:0] less16,equal16;
  reg borrow31;
  integer group_index;
  begin
   // Modulo-2^32 subtract sign = operand sign XORs XOR low-31 borrow.
   // Balanced lexicographic tree; no 31/32-bit subtract or compare chain.
   for(group_index=0;group_index<7;group_index=group_index+1)begin
    less4[group_index]=deadline[group_index*4+:4]<now[group_index*4+:4];
    equal4[group_index]=deadline[group_index*4+:4]==now[group_index*4+:4];
   end
   less4[7]=deadline[30:28]<now[30:28];
   equal4[7]=deadline[30:28]==now[30:28];
   for(group_index=0;group_index<4;group_index=group_index+1)begin
    less8[group_index]=less4[group_index*2+1]|(equal4[group_index*2+1]&less4[group_index*2]);
    equal8[group_index]=equal4[group_index*2+1]&equal4[group_index*2];
   end
   for(group_index=0;group_index<2;group_index=group_index+1)begin
    less16[group_index]=less8[group_index*2+1]|(equal8[group_index*2+1]&less8[group_index*2]);
    equal16[group_index]=equal8[group_index*2+1]&equal8[group_index*2];
   end
   borrow31=less16[1]|(equal16[1]&less16[0]);
   past=deadline[31]^now[31]^borrow31;
  end
 endfunction
 function future_budget;input [31:0] deadline,now,limit;input strict;reg signed [31:0] delta;begin
  delta=$signed(deadline-now);future_budget=(strict?delta>0:delta>=0)&&$unsigned(delta)<=limit;end endfunction
 function raw_stop;input [15:0] v,w;input [47:0] ranges;input [2:0] online;begin
  raw_stop=(online!=3'b111)||($signed(v)>0&&ranges[15:0]<=200)||
   ($signed(w)>0&&ranges[31:16]<=200)||($signed(w)<0&&ranges[47:32]<=200);
 end endfunction
 function task_match;input pose_revision;begin
  task_match=bound&&task_valid&&eq_a_session&&eq_a_run&&eq_a_goal&&
   eq_a_epoch&&1'b1&&eq_a_map_revision&&
   1'b1&&eq_a_plan&&eq_a_frame;
 end endfunction
 function pin_ok;input [1:0] lane;input [31:0] token;input [5:0] words;begin
  pin_ok=token!=0&&pin_flags[lane*4]&&pin_tokens[lane*32+:32]==token&&
   pin_bytes[lane*9+:9]>={words,2'b00};
 end endfunction
 wire pose_fresh=pose_qualified&&!past(pose_deadline,device_ms);
 wire permit_fresh=permit_qualified&&!past(permit_deadline,device_ms);
 wire auth_fresh=auth_qualified&&!past(auth_deadline,device_ms)&&(auth_deadline!=device_ms);
 wire cpu_fresh=pwm_valid&&(device_ms-commit_ms)<60;
 wire [7:0] live_bits={manual_quiet,continuation,started,auth_fresh,permit_fresh,pose_fresh,task_valid,bound};
 wire cancel_identity=bound&&task_valid&&eq_cancel_session&&eq_cancel_run&&eq_cancel_goal;
 wire cancel_kind_ok=cancel_desc[1:0]==1||(cancel_desc[1:0]==2&&eq_cancel_epoch);
 wire cancel_shape_ok=cancel_identity&&cancel_kind_ok&&cancel_desc[321:258]!=0;
 wire [63:0] family_floor=cancel_desc[1:0]==1?cancel_floor:nva_floor;
 // Balanced two-bit comparison tree replaces a mapped serial 64-bit chain.
 // This is combinational and still uses the live replay floor in this cycle.
 wire cancel_sequence_gt;
 wire cancel_new=cancel_valid&&cancel_shape_ok&&cancel_sequence_gt;
 wire cancel_duplicate=cancel_valid&&cancel_shape_ok&&eq_cancel_duplicate;
 wire [31:0] asynchronous_reasons=(mot_accept?EV_MOT:0)|
  (cancel_new&&!revoked?(cancel_desc[1:0]==1?EV_CANCEL:EV_REVOKE):0)|
  (started&&permit_qualified&&!permit_fresh?EV_MOTION:0)|
  (started&&auth_qualified&&!auth_fresh?EV_AUTH:0)|
  (started&&pose_qualified&&!pose_fresh?EV_POSE:0)|
  (cpu_trap&&!trap_previous?EV_TRAP:0);
 wire hard_event=asynchronous_reasons!=0;
 wire [31:0] gates=(!bound?1:0)|(!task_valid?2:0)|(!started?4:0)|(!permit_fresh?8:0)|
  (!auth_fresh?16:0)|(!pose_fresh?32:0)|(!cpu_fresh?64:0)|(cpu_trap?128:0)|
  (raw_stop(linear,angular,raw_sonar_mm,raw_sonar_online)?256:0)|(owner==1?512:0)|(revoked?1024:0)|
  (map_fenced?2048:0)|(context_race?4096:0);
 assign nav_epoch=epoch;
 assign context_generation=ctx;
 assign nav_owned=owner==2;
 assign nav_live=owner==2&&bound&&task_valid&&started&&continuation&&plan_ready&&!map_fenced&&!revoked&&!exhausted&&
  pose_fresh&&permit_fresh&&auth_fresh&&cpu_fresh&&pwm_flags[1:0]==2'b11&&!cpu_trap&&!raw_stop(linear,angular,raw_sonar_mm,raw_sonar_online)&&!hard_event;
 assign nav_pwm=nav_live?pwm:64'b0;
 assign nav_linear=nav_live?linear:16'b0;
 assign nav_angular=nav_live?angular:16'b0;
 assign cancel_ready=reset_n;

 reg [31:0] source_expected;

 nav_eq_lut #(.WIDTH(64)) cmp_cancel_session(.a(cancel_desc[65:2]),.b(session),.eq(eq_cancel_session),.ne(ne_cancel_session));
 nav_eq_lut #(.WIDTH(64)) cmp_cancel_run(.a(cancel_desc[129:66]),.b(run),.eq(eq_cancel_run),.ne(ne_cancel_run));
 nav_eq_lut #(.WIDTH(128)) cmp_cancel_goal(.a(cancel_desc[257:130]),.b(goal),.eq(eq_cancel_goal),.ne(ne_cancel_goal));
 nav_eq_lut #(.WIDTH(64)) cmp_cancel_epoch(.a(cancel_desc[385:322]),.b(epoch),.eq(eq_cancel_epoch),.ne(ne_cancel_epoch));
 nav_cancel_order64_before cmp_cancel_order(.a(cancel_desc[321:258]),.b(family_floor),
  .gt(cancel_sequence_gt),.eq(eq_cancel_duplicate));
 assign ne_cancel_duplicate=~eq_cancel_duplicate;
 nav_eq_lut #(.WIDTH(32)) cmp_source_word(.a(source_data),.b(source_expected),.eq(eq_source_word),.ne(ne_source_word));
 nav_eq_lut #(.WIDTH(32)) cmp_op_context(.a(operation_ctx),.b(ctx),.eq(eq_op_context),.ne(ne_op_context));

 localparam S_TICKET=0,S_TICKET_WAIT=1,S_ACTION=2,S_ACTION_WAIT=3,
  S_TOKEN=4,S_TOKEN_WAIT=5,S_SCAN=6,S_SCAN_WAIT=7,S_PREPARE=8,
  S_SOURCE=9,S_PUBLISH=10,S_EXPECTED=11;
 reg [3:0] phase;
 reg [5:0] scan_word;
 reg [31:0] action_word,request_token,pose_pin_word,permit_pin_word;
 reg [31:0] resume_word,claim_word,pose_flags_word;
 reg [31:0] work[0:10];
 reg nz_session,nz_run,nz_goal,nz_revision,nz_plan,nz_frame,nz_pose,nz_permit,nz_auth,nz_phone;
 reg order_run_gt,order_plan_gt,order_pose_gt,order_permit_gt,order_auth_lt,order_phone_lt;
 reg low_lt,low_gt;
 reg pwm_owner_ok,pwm_range_bad,pwm_wheel_bad;
 reg source_literal;
 reg [6:0] source_shadow_address;
 reg [31:0] source_literal_value;
 reg [6:0] checker_address;
 wire [31:0] checker_data;
 wire checker_en=check_busy&&(phase==S_TICKET||phase==S_ACTION||phase==S_TOKEN||phase==S_SCAN||phase==S_PREPARE);
 wire [63:0] expected_epoch=operation==OP_AUTH&&action==INSTALL_SESSION?64'd1:
   operation==OP_AUTH&&action==INSTALL_TASK&&request_token!=0?epoch+64'd1:epoch;
 wire shadow_address=(bus_offset>=12'h900&&bus_offset<=12'h9c0&&bus_offset[1:0]==0)||
   (bus_offset>=12'ha00&&bus_offset<=12'ha90&&bus_offset[1:0]==0);
 wire [6:0] cpu_shadow_address=bus_offset[8]?{1'b0,bus_offset[7:2]}:{1'b1,bus_offset[7:2]};
 wire shadow_rsp,shadow_ready;wire [31:0] shadow_rdata;
 wire shadow_writable=!check_busy&&!draining&&!hard_event;
 nav_owner_shadow_before shadow(.clk(clk),.reset_n(reset_n),
   .cpu_req(bus_req&&shadow_address),.cpu_addr(cpu_shadow_address),
   .cpu_data(bus_wdata),.cpu_strobe(shadow_writable?bus_wstrb:4'b0),
   .cpu_rsp(shadow_rsp),.cpu_rdata(shadow_rdata),.ready(shadow_ready),
   .check_en(checker_en),.check_addr(checker_address),.check_data(checker_data));
 function [63:0] W64;input integer n;begin W64={work[n+1],work[n]};end endfunction
 always @*begin
  checker_address=0;
  case(phase)
   S_TICKET:checker_address=operation==OP_PWM?7'd64:7'd1;
   S_ACTION:checker_address=0;
   S_TOKEN:checker_address=31;
   S_SCAN:checker_address={1'b0,scan_word}+(operation==OP_PWM?7'd64:7'd0);
   S_PREPARE:checker_address=source_shadow_address;
  endcase
 end

 // The immutable ingress-source comparison reads exactly the same shadow word
 // as before. Literal header words never consume a spare memory read port.
 always @*begin
  source_literal=1;source_shadow_address=0;source_literal_value=0;
  case(source_word)
   0:source_literal_value=operation!=OP_CLAIM&&action==ACCEPT_PERMIT?32'h4e564131:32'h4e415633;
   1:begin
    if(operation==OP_CLAIM)source_literal_value=32'h01210000;
    else case(action)
     INSTALL_SESSION:source_literal_value=32'h01030000;
     INSTALL_TASK:source_literal_value=32'h01200000;
     ACCEPT_POSE:source_literal_value=32'h01300000;
     ACCEPT_PERMIT:source_literal_value=32'h01020000;
     MAP_FENCE:source_literal_value=source_lane==1?32'h01300000:32'h01100000;
    endcase
   end
   2:begin source_literal=0;source_shadow_address=47;end
   3:begin source_literal=0;source_shadow_address=46;end
   4:if(action!=INSTALL_SESSION)begin source_literal=0;source_shadow_address=5;end
   5:if(action!=INSTALL_SESSION)begin source_literal=0;source_shadow_address=4;end
   6:begin source_literal=0;source_shadow_address=7;end
   7:begin source_literal=0;source_shadow_address=6;end
   8,9,10,11:begin source_literal=0;source_shadow_address={1'b0,source_word};end
   12:source_literal_value=0;
   13:source_literal_value=1;
   14:source_literal_value=0;
   15:begin source_literal=0;source_shadow_address=16;end
   16:source_literal_value=0;
   17:source_literal_value=1;
   18:begin source_literal=0;source_shadow_address=21;end
   19:begin source_literal=0;source_shadow_address=20;end
   20:begin source_literal=0;source_shadow_address=45;end
   21:begin source_literal=0;source_shadow_address=44;end
   22:begin
    if(operation==OP_CLAIM)source_literal_value=32'h00180000;
    else case(action)
     INSTALL_SESSION:source_literal_value=32'h00100000;
     INSTALL_TASK:source_literal_value=32'h00180000;
     ACCEPT_POSE:source_literal_value=32'h003c0000;
     ACCEPT_PERMIT:source_literal_value=32'h00400000;
     MAP_FENCE:source_literal_value=source_lane==1?32'h003c0000:32'h00200000;
    endcase
   end
   default:begin
    if(operation==OP_CLAIM)case(source_word)
     24:begin source_literal=0;source_shadow_address=13;end
     25:begin source_literal=0;source_shadow_address=12;end
     26:begin source_literal=0;source_shadow_address=34;end
     27:begin source_literal=0;source_shadow_address=33;end
     28:begin source_literal=0;source_shadow_address=37;end
     29:begin source_literal=0;source_shadow_address=36;end
    endcase
    else if(action==INSTALL_SESSION)case(source_word)
     24:begin source_literal=0;source_shadow_address=5;end
     25:begin source_literal=0;source_shadow_address=4;end
    endcase
    else if(action==INSTALL_TASK)case(source_word)
     26:begin source_literal=0;source_shadow_address=22;end
     27:begin source_literal=0;source_shadow_address=25;end
     28:begin source_literal=0;source_shadow_address=24;end
    endcase
    else if(action==ACCEPT_PERMIT)case(source_word)
     24:source_literal_value=1;
     25:begin source_literal=0;source_shadow_address=13;end
     26:begin source_literal=0;source_shadow_address=12;end
     27:begin source_literal=0;source_shadow_address=37;end
     28:begin source_literal=0;source_shadow_address=36;end
     29:begin source_literal=0;source_shadow_address=39;end
     30:begin source_literal=0;source_shadow_address=38;end
     31:begin source_literal=0;source_shadow_address=34;end
     32:begin source_literal=0;source_shadow_address=33;end
     33:begin source_literal=0;source_shadow_address=35;end
     34:begin source_literal=0;source_shadow_address=40;end
    endcase
    else if(action==ACCEPT_POSE||(action==MAP_FENCE&&source_lane==1))case(source_word)
     24:begin source_literal=0;source_shadow_address=28;end
     25:begin source_literal=0;source_shadow_address=29;end
     26:begin source_literal=0;source_shadow_address=30;end
     27:begin source_literal=0;source_shadow_address=22;end
     28:begin source_literal=0;source_shadow_address=25;end
     29:begin source_literal=0;source_shadow_address=24;end
     35:begin source_literal=0;source_shadow_address=26;end
     38:begin source_literal=0;source_shadow_address=27;end
    endcase
    else if(action==MAP_FENCE)case(source_word)
     24:begin source_literal=0;source_shadow_address=22;end
     27:source_literal_value=32'h00644040;
     28:begin source_literal=0;source_shadow_address=48;end
    endcase
   end
  endcase
 end
 function compared;input [5:0] n;begin
  compared=0;
  if(n==0||n==1||n==22)compared=1;
  if(operation==OP_CLAIM)begin
   if((n>=2&&n<=21)||(n>=24&&n<=29))compared=1;
  end else if(action==INSTALL_SESSION&&operation==OP_AUTH)begin
   if(n==4||n==5||n==24||n==25)compared=1;
  end else if(action==ACCEPT_POSE||(action==MAP_FENCE&&source_lane==1))begin
   if((n>=4&&n<=5)||(n>=12&&n<=17)||(n>=24&&n<=29)||n==35||n==38)compared=1;
   // A stop-only C-validated NE_REBIND may reject precisely this raw frame.
   // Keep old accepted a[22]; never install the rejected frame as authority.
   if(action==MAP_FENCE&&n==27)compared=0;
  end else if(action==MAP_FENCE)begin
   // Map publication is asynchronous: it need not carry a goal/run/plan.
   if(n==4||n==5||(n>=12&&n<=15)||n==24||n==27||n==28)compared=1;
  end else begin
   if((n>=4&&n<=17))compared=1;
   if(operation==OP_CLAIM&&(n==2||n==3||(n>=18&&n<=21)||(n>=24&&n<=29)))compared=1;
   if(action==INSTALL_TASK&&(n>=26&&n<=28))compared=1;
   if(action==ACCEPT_PERMIT&&((n>=18&&n<=19)||(n>=24&&n<=34)))compared=1;
   if(action==MAP_FENCE&&(n==24||n==27||n==28))compared=1;
  end
 end endfunction
 reg eq_p_ticket;wire ne_p_ticket=~eq_p_ticket;
 function [31:0] validate_authority;input [2:0] op;input [2:0] act;begin
  validate_authority=OK;
  if(auth_reserved||(op==OP_AUTH&&(action_word<1||action_word>5)))validate_authority=RESERVED;
  else if(!operation_ticket||ne_a_ticket||(!(op==OP_AUTH&&act==INSTALL_SESSION)&&operation_ticket<=last_ticket))validate_authority=BAD_TICKET;
  else if(ne_a_old_epoch||ne_a_context||ne_op_context)validate_authority=OLD_EPOCH;
  else if(cpu_trap||(exhausted&&!(op==OP_AUTH&&act==INSTALL_SESSION)))validate_authority=EXPIRED;
  else if(op==OP_REVOKE)begin
   if(!task_match(0)||revoked)validate_authority=IDENTITY;
  end else if(op==OP_CLAIM)begin
   if(!task_match(0))validate_authority=IDENTITY;
   else if(revoked||!plan_ready||!plan)validate_authority=NO_START;
   else if(!pose_fresh||!permit_fresh||!auth_fresh)validate_authority=EXPIRED;
   else if(ne_a_pose_seq||ne_a_pose_token||ne_a_permit_seq||ne_a_permit_token||ne_a_auth_epoch)validate_authority=OLD_SOURCE;
   else if(claim_word==1)begin if(started||continuation||!manual_quiet||resume_word)validate_authority=ACTIVE_OWNER;end
   else if(claim_word==2)begin if(!started||!continuation||resume_word!=1)validate_authority=NO_START;end
   else validate_authority=BAD_RANGE;
  end else case(act)
   INSTALL_SESSION:begin
    if(!nz_session||eq_a_session)validate_authority=IDENTITY;
    else if(owner==2||started||continuation||!manual_quiet)validate_authority=ACTIVE_OWNER;
    else if(ne_a_epoch)validate_authority=OLD_EPOCH;
   end
   INSTALL_TASK:begin
    if(!bound||ne_a_session||!nz_run||!nz_goal||1'b0||!nz_revision||1'b0||!nz_frame)validate_authority=IDENTITY;
    else if(request_token)begin
     if(started&&ne_a_map_revision)validate_authority=ACTIVE_OWNER;
     else if(!order_run_gt)validate_authority=OLD_SOURCE;
     else if(epoch_full||ne_a_epoch)validate_authority=OLD_EPOCH;
     else if(resume_word)validate_authority=NO_START;
     else if(frame_token&&ne_a_frame)validate_authority=IDENTITY;
    end else begin
     if(!task_valid||revoked||ne_a_run||ne_a_goal||ne_a_epoch||ne_a_map_revision||ne_a_frame)validate_authority=IDENTITY;
     else if(!nz_plan||!order_plan_gt||resume_word!={31'b0,continuation})validate_authority=OLD_SOURCE;
    end
   end
   ACCEPT_POSE:begin
    if(!task_match(1))validate_authority=IDENTITY;
    else if(!nz_pose||(1'b1&&!order_pose_gt))validate_authority=OLD_SOURCE;
    else if(!future_budget(work[2],device_ms,250,0)||pose_flags_word!=3)validate_authority=EXPIRED;
    else if($signed(work[5])< -3141593||$signed(work[5])>=3141593)validate_authority=BAD_RANGE;
   end
   ACCEPT_PERMIT:begin
    if(!task_match(0)||revoked)validate_authority=IDENTITY;
    else if(!source_transport||source_transport<=nva_floor)validate_authority=OLD_SOURCE;
    else if(!nz_permit||!order_permit_gt||!nz_auth||!nz_phone||order_auth_lt)validate_authority=OLD_SOURCE;
    else if(eq_a_auth_epoch&&(order_phone_lt||(eq_a_phone_seq&&$signed(work[7]-auth_deadline)>0)))validate_authority=OLD_SOURCE;
    else if(!future_budget(work[2],device_ms,500,0)||!future_budget(work[7],device_ms,30000,1))validate_authority=EXPIRED;
   end
   MAP_FENCE:begin
    if(!bound||!task_valid||ne_a_session||ne_a_run||ne_a_goal||ne_a_epoch||ne_a_frame||revoked)validate_authority=IDENTITY;
    else if(request_token!=0)validate_authority=ACTIVE_OWNER;
    else if(!request_token&&!pose_pin_word&&(!started||!continuation||resume_word!=1||1'b0||1'b0))validate_authority=NO_START;
    else if(request_token&&pose_pin_word)validate_authority=BAD_TOKEN;
    else if(pose_pin_word!=0&&resume_word!=0)validate_authority=NO_START;
   end
   default:validate_authority=RESERVED;
  endcase
 end endfunction
 function [31:0] validate_pwm;input [31:0] ticket;integer k;begin
  validate_pwm=OK;
  if(pwm_reserved)validate_pwm=RESERVED;
  else if(!ticket||ne_p_ticket||ticket<=last_ticket)validate_pwm=BAD_TICKET;
  else if(ne_p_epoch||ne_p_context||ne_op_context)validate_pwm=OLD_EPOCH;
  else if(owner!=2||!started||!continuation||!plan_ready||map_fenced||revoked||exhausted)validate_pwm=NO_START;
  else if(ne_p_session||ne_p_run||ne_p_goal||ne_p_plan||1'b0||ne_p_map_revision||1'b0||ne_p_frame||!pwm_owner_ok)validate_pwm=IDENTITY;
  else if(ne_p_pose_seq||ne_p_permit_seq||ne_p_auth_epoch||ne_p_pose_token||ne_p_permit_token)validate_pwm=OLD_SOURCE;
  else if(!pose_fresh||!permit_fresh||!auth_fresh||cpu_trap||raw_stop(work[5][15:0],work[6][15:0],raw_sonar_mm,raw_sonar_online))validate_pwm=EXPIRED;
  else if(pwm_range_bad)validate_pwm=BAD_RANGE;
  if(pwm_wheel_bad)validate_pwm=BAD_RANGE;
 end endfunction

 task changed;begin hw_generation<=hw_generation+2;status_generation<=status_generation+1;end endtask
 task hard_revoke;input [31:0] reason;begin
  event_valid<=1;event_id<=event_full?U64MAX:event_id+1;
  event_old_epoch<=epoch;event_new_epoch<=epoch_full?U64MAX:epoch+1;
  epoch<=epoch_full?U64MAX:epoch+1;
  event_reason<=reason|((epoch_near_full||event_near_full)?EV_EXHAUST:0);
  event_context<=ctx+1;ctx<=ctx+1;event_generation<=event_generation+2;
  exhausted<=exhausted||epoch_near_full||event_near_full;
  started<=0;continuation<=0;revoked<=1;map_fenced<=1;
  pose_qualified<=0;permit_qualified<=0;auth_qualified<=0;pwm_valid<=0;
  if(reason&EV_MOT)owner<=1;else if(owner==2)owner<=0;
  pwm<=0;linear<=0;angular<=0;last_revoke_reason<=reason;changed;
 end endtask
 task publish;input [31:0] result;begin
  check_busy<=0;source_req<=0;read_wait<=0;
  case(operation)OP_PWM:pwm_result<=result;OP_AUTH:auth_result<=result;OP_CLAIM:claim_result<=result;OP_REVOKE:revoke_result<=result;endcase
  changed;
  if(result==OK)begin
   last_ticket<=operation_ticket;context_race<=0;
   if(operation==OP_PWM)begin
    pwm<={work[4][15:0],work[3][15:0],work[2][15:0],work[1][15:0]};
    linear<=work[5][15:0];angular<=work[6][15:0];pwm_flags<=work[0];
    commit_ms<=device_ms;pwm_valid<=1;last_pwm_ticket<=operation_ticket;
   end else if(operation==OP_REVOKE)hard_revoke(EV_REVOKE);
   else if(operation==OP_CLAIM)begin owner<=2;started<=1;continuation<=1;revoked<=0;map_fenced<=0;pwm_valid<=0;ctx<=ctx+1;end
   else case(action)
    INSTALL_SESSION:begin
     session<=W64(0);bound<=1;task_valid<=0;run<=0;run_floor<=0;goal<=0;plan<=0;frame_token<=0;
     map_generation<=0;active_crc<=0;
     epoch<=1;event_id<=0;event_valid<=0;event_reason<=0;event_old_epoch<=0;event_new_epoch<=0;event_context<=0;event_generation<=event_generation+2;
     owner<=0;started<=0;continuation<=0;revoked<=0;map_fenced<=0;exhausted<=0;plan_ready<=0;
     pose_qualified<=0;permit_qualified<=0;auth_qualified<=0;pwm_valid<=0;pose_seq<=0;permit_seq<=0;auth_epoch<=0;phone_seq<=0;
     pose_deadline<=0;permit_deadline<=0;auth_deadline<=0;pose_token<=0;permit_token<=0;cancel_floor<=0;nva_floor<=0;
     pwm<=0;linear<=0;angular<=0;ctx<=ctx+1;
    end
    INSTALL_TASK:begin
     run<=W64(0);goal<={work[2],work[3],work[4],work[5]};plan<=W64(6);map_generation<=work[8];frame_token<=work[9];active_crc<=work[10];
     task_valid<=1;plan_ready<=W64(6)!=0;revoked<=0;map_fenced<=1;pwm_valid<=0;ctx<=ctx+1;
     if(request_token!=0)begin
      run_floor<=W64(0);epoch<=expected_epoch;started<=0;continuation<=0;pose_qualified<=0;permit_qualified<=0;auth_qualified<=0;if(owner==2)owner<=0;
      if(epoch_near_full)hard_revoke(EV_EXHAUST);
     end
    end
    ACCEPT_POSE:begin
     pose_seq<=W64(0);pose_deadline<=work[2];pose_x<=work[3];pose_y<=work[4];pose_yaw<=work[5];
     pose_token<=pose_pin_word;pose_rx_ms<=pin_rx_ms[63:32];pose_qualified<=1;ctx<=ctx+1;
    end
    ACCEPT_PERMIT:begin
     nva_floor<=source_transport;permit_seq<=W64(0);permit_deadline<=work[2];auth_epoch<=W64(3);phone_seq<=W64(5);auth_deadline<=work[7];permit_token<=permit_pin_word;permit_qualified<=1;auth_qualified<=1;ctx<=ctx+1;
    end
    MAP_FENCE:begin
     if(pose_pin_word!=0)hard_revoke(EV_REFERENCE);
     else begin map_fenced<=1;plan_ready<=0;pwm_valid<=0;ctx<=ctx+1;end
    end
   endcase
  end else if(result==OLD_EPOCH)context_race<=1;
 end endtask
 reg [31:0] read_value;
 always @* begin
  read_value=0;
  case(bus_offset)
   12'h400:read_value=32'h4e584d31;12'h404:read_value=32'h00010000;12'h408:read_value=63;
   12'h40c:read_value=gates;12'h410:read_value={30'b0,owner};12'h414:read_value=epoch[31:0];12'h418:read_value=epoch[63:32];12'h41c:read_value={24'b0,live_bits};12'h420:read_value=device_ms;
   12'h48c:read_value=cancel_count;12'h4a4:read_value=cancel_rejected;12'h4ac:read_value=hw_generation;
   12'h4b4:read_value=auth_result;12'h4bc:read_value=claim_result;12'h4c4:read_value=pwm_result;12'h4cc:read_value=revoke_result;
   12'h4d0:read_value=event_generation;12'h4d4:read_value={30'b0,exhausted,event_valid};12'h4d8:read_value=event_session[31:0];12'h4dc:read_value=event_session[63:32];
   12'h4e0:read_value=event_id[31:0];12'h4e4:read_value=event_id[63:32];12'h4e8:read_value=event_old_epoch[31:0];12'h4ec:read_value=event_old_epoch[63:32];
   12'h4f0:read_value=event_new_epoch[31:0];12'h4f4:read_value=event_new_epoch[63:32];12'h4f8:read_value=event_reason;12'h4fc:read_value=event_context;
   12'hb00:read_value={30'b0,owner};12'hb04:read_value={24'b0,live_bits};12'hb08:read_value=epoch[31:0];12'hb0c:read_value=epoch[63:32];12'hb10:read_value=session[31:0];12'hb14:read_value=session[63:32];
   12'hb18:read_value=run[31:0];12'hb1c:read_value=run[63:32];12'hb20:read_value=goal[127:96];12'hb24:read_value=goal[95:64];12'hb28:read_value=goal[63:32];12'hb2c:read_value=goal[31:0];
   12'hb30:read_value=ctx;12'hb34:read_value=last_pwm_ticket;12'hb38:read_value=commit_ms;12'hb3c:read_value=gates;12'hb40:read_value=pose_seq[31:0];12'hb44:read_value=pose_seq[63:32];
   12'hb48:read_value=pose_deadline;12'hb4c:read_value=pose_rx_ms;12'hb50:read_value=permit_seq[31:0];12'hb54:read_value=permit_seq[63:32];12'hb58:read_value=permit_deadline;12'hb5c:read_value=auth_deadline;
   12'hb60:read_value=auth_epoch[31:0];12'hb64:read_value=auth_epoch[63:32];12'hb68:read_value=phone_seq[31:0];12'hb6c:read_value=phone_seq[63:32];12'hb70:read_value=pose_x;12'hb74:read_value=pose_y;12'hb78:read_value=pose_yaw;
   12'hb7c:read_value=auth_result;12'hb80:read_value=claim_result;12'hb84:read_value=pwm_result;12'hb88:read_value=last_revoke_reason;12'hb8c:read_value=revoke_result;
   12'hb90:read_value=plan[31:0];12'hb94:read_value=plan[63:32];12'hb98:read_value=map_epoch[31:0];12'hb9c:read_value=map_epoch[63:32];12'hba0:read_value=map_revision[31:0];12'hba4:read_value=map_revision[63:32];
   12'hba8:read_value=localization[31:0];12'hbac:read_value=localization[63:32];12'hbb0:read_value=frame_token;12'hbb4:read_value=last_tx_ticket;12'hbb8:read_value=permit_token;12'hbbc:read_value=pose_token;
   12'hbc0:read_value=active_crc;12'hbd0:read_value=cancel_floor[31:0];12'hbd4:read_value=cancel_floor[63:32];12'hbd8:read_value=status_generation[31:0];12'hbdc:read_value=status_generation[63:32];
   default:read_value=0;
  endcase
 end

 // The current word is read directly from BSRAM, not a wide input mux.
 reg [31:0] identity_expected;
 wire identity_equal=checker_data==identity_expected;
 always @*begin
  identity_expected=0;
  if(operation==OP_PWM)case(scan_word)
   0:identity_expected=operation_ticket;
   10:identity_expected=epoch[31:0];
   11:identity_expected=epoch[63:32];
   36:identity_expected=ctx;
   2:identity_expected=session[31:0];
   3:identity_expected=session[63:32];
   4:identity_expected=run[31:0];
   5:identity_expected=run[63:32];
   6:identity_expected=goal[127:96];
   7:identity_expected=goal[95:64];
   8:identity_expected=goal[63:32];
   9:identity_expected=goal[31:0];
   18:identity_expected=plan[31:0];
   19:identity_expected=plan[63:32];
   22:identity_expected=map_generation;
   26:identity_expected=frame_token;
   12:identity_expected=pose_seq[31:0];
   13:identity_expected=pose_seq[63:32];
   14:identity_expected=permit_seq[31:0];
   15:identity_expected=permit_seq[63:32];
   16:identity_expected=auth_epoch[31:0];
   17:identity_expected=auth_epoch[63:32];
   34:identity_expected=pose_token;
   35:identity_expected=permit_token;
  endcase
  else case(scan_word)
   1:identity_expected=operation_ticket;
   2:identity_expected=epoch[31:0];
   3:identity_expected=epoch[63:32];
   43:identity_expected=ctx;
   4:identity_expected=session[31:0];
   5:identity_expected=session[63:32];
   6:identity_expected=run[31:0];
   7:identity_expected=run[63:32];
   8:identity_expected=goal[127:96];
   9:identity_expected=goal[95:64];
   10:identity_expected=goal[63:32];
   11:identity_expected=goal[31:0];
   12:identity_expected=expected_epoch[31:0];
   13:identity_expected=expected_epoch[63:32];
   16:identity_expected=map_generation;
   20:identity_expected=plan[31:0];
   21:identity_expected=plan[63:32];
   22:identity_expected=frame_token;
   24:identity_expected=pose_seq[31:0];
   25:identity_expected=pose_seq[63:32];
   23:identity_expected=pose_token;
   33:identity_expected=permit_seq[31:0];
   34:identity_expected=permit_seq[63:32];
   32:identity_expected=permit_token;
   36:identity_expected=auth_epoch[31:0];
   37:identity_expected=auth_epoch[63:32];
   38:identity_expected=phone_seq[31:0];
   39:identity_expected=phone_seq[63:32];
  endcase
 end

 task reset_checks;begin
 eq_a_ticket<=1; eq_a_old_epoch<=1; eq_a_context<=1; eq_a_session<=1; eq_a_run<=1; eq_a_goal<=1; eq_a_epoch<=1; eq_a_map_revision<=1; eq_a_plan<=1; eq_a_frame<=1; eq_a_pose_seq<=1; eq_a_pose_token<=1; eq_a_permit_seq<=1; eq_a_permit_token<=1; eq_a_auth_epoch<=1; eq_a_phone_seq<=1; eq_p_ticket<=1; eq_p_epoch<=1; eq_p_context<=1; eq_p_session<=1; eq_p_run<=1; eq_p_goal<=1; eq_p_plan<=1; eq_p_map_revision<=1; eq_p_frame<=1; eq_p_pose_seq<=1; eq_p_permit_seq<=1; eq_p_auth_epoch<=1; eq_p_pose_token<=1; eq_p_permit_token<=1;
  nz_session<=0;nz_run<=0;nz_goal<=0;nz_revision<=0;nz_plan<=0;nz_frame<=0;
  nz_pose<=0;nz_permit<=0;nz_auth<=0;nz_phone<=0;
  order_run_gt<=0;order_plan_gt<=0;order_pose_gt<=0;order_permit_gt<=0;order_auth_lt<=0;order_phone_lt<=0;
  low_lt<=0;low_gt<=0;pwm_range_bad<=0;pwm_wheel_bad<=0;pwm_owner_ok<=0;
 end endtask
 task consume_word;begin
  if(operation==OP_PWM)begin
   case(scan_word)
    0:eq_p_ticket<=eq_p_ticket&&identity_equal;
    10,11:eq_p_epoch<=eq_p_epoch&&identity_equal;
    36:eq_p_context<=eq_p_context&&identity_equal;
    2,3:eq_p_session<=eq_p_session&&identity_equal;
    4,5:eq_p_run<=eq_p_run&&identity_equal;
    6,7,8,9:eq_p_goal<=eq_p_goal&&identity_equal;
    18,19:eq_p_plan<=eq_p_plan&&identity_equal;
    22:eq_p_map_revision<=eq_p_map_revision&&identity_equal;
    26:eq_p_frame<=eq_p_frame&&identity_equal;
    12,13:eq_p_pose_seq<=eq_p_pose_seq&&identity_equal;
    14,15:eq_p_permit_seq<=eq_p_permit_seq&&identity_equal;
    16,17:eq_p_auth_epoch<=eq_p_auth_epoch&&identity_equal;
    34:eq_p_pose_token<=eq_p_pose_token&&identity_equal;
    35:eq_p_permit_token<=eq_p_permit_token&&identity_equal;

   endcase
   if(scan_word==1)pwm_owner_ok<=checker_data==2;
   if(scan_word==27)work[0]<=checker_data;
   if(scan_word>=28&&scan_word<=33)case(scan_word)
    28:work[1]<=checker_data;29:work[2]<=checker_data;30:work[3]<=checker_data;
    31:work[4]<=checker_data;32:work[5]<=checker_data;33:work[6]<=checker_data;
   endcase
   if(scan_word>=28&&scan_word<=31&&($signed(checker_data)< -600||$signed(checker_data)>600))pwm_wheel_bad<=1;
   if(scan_word==32&&($signed(checker_data)< -300||$signed(checker_data)>300)||
      scan_word==33&&($signed(checker_data)< -1000||$signed(checker_data)>1000))pwm_range_bad<=1;
  end else begin
   case(scan_word)
    1:eq_a_ticket<=eq_a_ticket&&identity_equal;
    2,3:eq_a_old_epoch<=eq_a_old_epoch&&identity_equal;
    43:eq_a_context<=eq_a_context&&identity_equal;
    4,5:eq_a_session<=eq_a_session&&identity_equal;
    6,7:eq_a_run<=eq_a_run&&identity_equal;
    8,9,10,11:eq_a_goal<=eq_a_goal&&identity_equal;
    12,13:eq_a_epoch<=eq_a_epoch&&identity_equal;
    16:eq_a_map_revision<=eq_a_map_revision&&identity_equal;
    20,21:eq_a_plan<=eq_a_plan&&identity_equal;
    22:eq_a_frame<=eq_a_frame&&identity_equal;
    24,25:eq_a_pose_seq<=eq_a_pose_seq&&identity_equal;
    23:eq_a_pose_token<=eq_a_pose_token&&identity_equal;
    33,34:eq_a_permit_seq<=eq_a_permit_seq&&identity_equal;
    32:eq_a_permit_token<=eq_a_permit_token&&identity_equal;
    36,37:eq_a_auth_epoch<=eq_a_auth_epoch&&identity_equal;
    38,39:eq_a_phone_seq<=eq_a_phone_seq&&identity_equal;

   endcase
   case(scan_word)
    4,5:nz_session<=nz_session||(checker_data!=0);
    6,7:nz_run<=nz_run||(checker_data!=0);
    8,9,10,11:nz_goal<=nz_goal||(checker_data!=0);
    16:nz_revision<=checker_data!=0;
    20,21:nz_plan<=nz_plan||(checker_data!=0);
    22:nz_frame<=checker_data!=0;
    23:pose_pin_word<=checker_data;
    24,25:nz_pose<=nz_pose||(checker_data!=0);
    27:pose_flags_word<=checker_data;
    32:permit_pin_word<=checker_data;
    33,34:nz_permit<=nz_permit||(checker_data!=0);
    36,37:nz_auth<=nz_auth||(checker_data!=0);
    38,39:nz_phone<=nz_phone||(checker_data!=0);
    41:resume_word<=checker_data;
    42:claim_word<=checker_data;
   endcase
   case(scan_word)
    6:begin low_lt<=checker_data<run_floor[31:0];low_gt<=checker_data>run_floor[31:0];end
    7:order_run_gt<=checker_data>run_floor[63:32]||(checker_data==run_floor[63:32]&&low_gt);
    20:begin low_lt<=checker_data<plan[31:0];low_gt<=checker_data>plan[31:0];end
    21:order_plan_gt<=checker_data>plan[63:32]||(checker_data==plan[63:32]&&low_gt);
    24:begin low_lt<=checker_data<pose_seq[31:0];low_gt<=checker_data>pose_seq[31:0];end
    25:order_pose_gt<=checker_data>pose_seq[63:32]||(checker_data==pose_seq[63:32]&&low_gt);
    33:begin low_lt<=checker_data<permit_seq[31:0];low_gt<=checker_data>permit_seq[31:0];end
    34:order_permit_gt<=checker_data>permit_seq[63:32]||(checker_data==permit_seq[63:32]&&low_gt);
    36:begin low_lt<=checker_data<auth_epoch[31:0];low_gt<=checker_data>auth_epoch[31:0];end
    37:order_auth_lt<=checker_data<auth_epoch[63:32]||(checker_data==auth_epoch[63:32]&&low_lt);
    38:begin low_lt<=checker_data<phone_seq[31:0];low_gt<=checker_data>phone_seq[31:0];end
    39:order_phone_lt<=checker_data<phone_seq[63:32]||(checker_data==phone_seq[63:32]&&low_lt);
   endcase
   if(action==INSTALL_SESSION)case(scan_word)4:work[0]<=checker_data;5:work[1]<=checker_data;endcase
   if(action==INSTALL_TASK)case(scan_word)
    6:work[0]<=checker_data;7:work[1]<=checker_data;
    8:work[2]<=checker_data;9:work[3]<=checker_data;10:work[4]<=checker_data;11:work[5]<=checker_data;
    20:work[6]<=checker_data;21:work[7]<=checker_data;16:work[8]<=checker_data;22:work[9]<=checker_data;48:work[10]<=checker_data;
   endcase
   if(action==ACCEPT_POSE)case(scan_word)
    24:work[0]<=checker_data;25:work[1]<=checker_data;26:work[2]<=checker_data;
    28:work[3]<=checker_data;29:work[4]<=checker_data;30:work[5]<=checker_data;
   endcase
   if(action==ACCEPT_PERMIT)case(scan_word)
    33:work[0]<=checker_data;34:work[1]<=checker_data;35:work[2]<=checker_data;
    36:work[3]<=checker_data;37:work[4]<=checker_data;38:work[5]<=checker_data;39:work[6]<=checker_data;40:work[7]<=checker_data;
   endcase
  end
 end endtask

 reg [31:0] result_code;
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   bus_rsp_valid<=0;bus_rsp_data<=0;source_req<=0;source_lane<=0;source_word<=0;
   auth_reserved<=0;pwm_reserved<=0;auth_result<=EMPTY;claim_result<=EMPTY;pwm_result<=EMPTY;revoke_result<=EMPTY;last_ticket<=0;
   session<=0;run<=0;epoch<=0;run_floor<=0;plan<=0;map_generation<=0;goal<=0;frame_token<=0;active_crc<=0;ctx<=0;hw_generation<=0;status_generation<=0;
   bound<=0;task_valid<=0;plan_ready<=0;started<=0;continuation<=0;revoked<=0;map_fenced<=0;exhausted<=0;owner<=0;
   pose_qualified<=0;permit_qualified<=0;auth_qualified<=0;pwm_valid<=0;pose_seq<=0;permit_seq<=0;auth_epoch<=0;phone_seq<=0;
   pose_deadline<=0;permit_deadline<=0;auth_deadline<=0;pose_rx_ms<=0;pose_token<=0;permit_token<=0;commit_ms<=0;last_pwm_ticket<=0;
   pose_x<=0;pose_y<=0;pose_yaw<=0;pwm_flags<=0;pwm<=0;linear<=0;angular<=0;
   cancel_floor<=0;nva_floor<=0;source_transport<=0;event_id<=0;event_old_epoch<=0;event_new_epoch<=0;event_reason<=0;event_context<=0;event_generation<=0;last_revoke_reason<=0;event_valid<=0;trap_previous<=0;
   cancel_count<=0;cancel_rejected<=0;context_race<=0;
   check_busy<=0;read_wait<=0;draining<=0;source_bad<=0;operation<=0;action<=0;operation_ticket<=0;operation_ctx<=0;operation_token<=0;check_words<=0;check_cycles<=0;

   phase<=S_TICKET;scan_word<=0;action_word<=0;request_token<=0;pose_pin_word<=0;permit_pin_word<=0;
   resume_word<=0;claim_word<=0;pose_flags_word<=0;source_expected<=0;
   for(i=0;i<11;i=i+1)work[i]<=0;
   reset_checks;

  end else begin
   bus_rsp_valid<=bus_req&&!shadow_address;bus_rsp_data<=read_value;trap_previous<=cpu_trap;
   if(shadow_rsp)begin bus_rsp_valid<=1;bus_rsp_data<=shadow_rdata;end
   if(draining&&source_valid)draining<=0;
   if(cancel_valid)begin
    if(cancel_new)begin if(cancel_desc[1:0]==1)cancel_floor<=cancel_desc[321:258];else nva_floor<=cancel_desc[321:258];if(!revoked)cancel_count<=cancel_count+1;end
    else if(!cancel_duplicate)cancel_rejected<=cancel_rejected+1;
   end
   if(hard_event)begin
    hard_revoke(asynchronous_reasons);
    if(check_busy)begin
     case(operation)OP_PWM:pwm_result<=OLD_EPOCH;OP_AUTH:auth_result<=OLD_EPOCH;OP_CLAIM:claim_result<=OLD_EPOCH;OP_REVOKE:revoke_result<=OLD_EPOCH;endcase
     draining<=(read_wait||source_req&&source_grant)&&!source_valid;
     check_busy<=0;read_wait<=0;source_req<=0;
    end
    if(bus_req&&bus_wstrb!=0)case(bus_offset)
     12'h4b0:auth_result<=OLD_EPOCH;12'h4b8:claim_result<=OLD_EPOCH;12'h4c0:pwm_result<=OLD_EPOCH;12'h4c8:revoke_result<=OLD_EPOCH;
    endcase
   end else begin
    if(pose_qualified&&!pose_fresh)begin pose_qualified<=0;ctx<=ctx+1;changed;end
    if(permit_qualified&&!permit_fresh)begin permit_qualified<=0;ctx<=ctx+1;changed;end
    if(auth_qualified&&!auth_fresh)begin auth_qualified<=0;ctx<=ctx+1;changed;end
    if(check_busy)begin
     check_cycles<=check_cycles+1;
     if(check_cycles>=CHECK_TIMEOUT_CYCLES)begin
      case(operation)OP_PWM:pwm_result<=BUSY;OP_AUTH:auth_result<=BUSY;OP_CLAIM:claim_result<=BUSY;OP_REVOKE:revoke_result<=BUSY;endcase
      draining<=(read_wait||source_req&&source_grant)&&!source_valid;
      check_busy<=0;read_wait<=0;source_req<=0;changed;
     end else case(phase)
      S_TICKET:phase<=S_TICKET_WAIT;
      S_TICKET_WAIT:begin
       if(operation!=OP_PWM&&checker_data!=operation_ticket)publish(BAD_TICKET);
       else if(operation==OP_PWM)begin scan_word<=0;phase<=S_SCAN;end
       else phase<=S_ACTION;
      end
      S_ACTION:phase<=S_ACTION_WAIT;
      S_ACTION_WAIT:begin action_word<=checker_data;action<=checker_data[2:0];phase<=S_TOKEN;end
      S_TOKEN:phase<=S_TOKEN_WAIT;
      S_TOKEN_WAIT:begin request_token<=checker_data;scan_word<=0;phase<=S_SCAN;end
      S_SCAN:phase<=S_SCAN_WAIT;
      S_SCAN_WAIT:begin
       consume_word;
       if(scan_word==(operation==OP_PWM?36:48))begin
        if(operation==OP_PWM||operation==OP_REVOKE||operation==OP_AUTH&&
          (action_word==INSTALL_TASK&&request_token==0||action_word==MAP_FENCE&&request_token==0&&pose_pin_word==0)||
          operation==OP_CLAIM&&claim_word==2)begin check_words<=0;phase<=S_PUBLISH;end
        else begin
         if(operation==OP_CLAIM)begin source_lane<=0;operation_token<=request_token;check_words<=30;end
         else if(action_word==ACCEPT_POSE||action_word==MAP_FENCE&&pose_pin_word!=0)begin source_lane<=1;operation_token<=pose_pin_word;check_words<=39;end
         else if(action_word==ACCEPT_PERMIT)begin source_lane<=2;operation_token<=permit_pin_word;check_words<=40;end
         else begin source_lane<=0;operation_token<=request_token;check_words<=action_word==INSTALL_SESSION?28:action_word==MAP_FENCE?32:30;end
         source_word<=0;phase<=S_PREPARE;
        end
       end else begin scan_word<=scan_word+1;phase<=S_SCAN;end
      end
      S_PREPARE:phase<=S_EXPECTED;
      S_EXPECTED:begin source_expected<=source_literal?source_literal_value:checker_data;source_req<=1;phase<=S_SOURCE;end
      S_SOURCE:begin
       if(source_req&&source_grant)begin source_req<=0;read_wait<=1;end
       else if(read_wait&&source_valid)begin
        read_wait<=0;
        if(source_word==2)source_transport[63:32]<=source_data;
        if(source_word==3)source_transport[31:0]<=source_data;
        if(compared(source_word)&&ne_source_word)source_bad<=1;
        if(source_word+1>=check_words)phase<=S_PUBLISH;
        else begin source_word<=source_word+1;phase<=S_PREPARE;end
       end
      end
      S_PUBLISH:begin
       result_code=operation==OP_PWM?validate_pwm(operation_ticket):validate_authority(operation,action);
       if(operation!=OP_PWM&&check_words!=0)begin
        if(!pin_ok(source_lane,operation_token,check_words))result_code=BAD_TOKEN;
        else if(source_bad)result_code=IDENTITY;
       end
       publish(result_code);
      end
     endcase
    end
    if(bus_req&&bus_wstrb!=0)begin
     if(shadow_address)begin
      if(shadow_writable&&bus_offset==12'h900)auth_reserved<=0;
      if(shadow_writable&&bus_offset==12'ha00)pwm_reserved<=0;
     end else if(bus_offset>=12'h9c4&&bus_offset<=12'h9ff)begin if(bus_wdata!=0)auth_reserved<=1;end
     else if(bus_offset>=12'ha94&&bus_offset<=12'haff)begin if(bus_wdata!=0)pwm_reserved<=1;end
     else if(bus_offset==12'h4b0||bus_offset==12'h4b8||bus_offset==12'h4c0||bus_offset==12'h4c8)begin
      result_code=bus_wstrb!=15?BAD_STROBE:(check_busy||draining||!shadow_ready)?BUSY:
         (!bus_wdata&&bus_offset!=12'h4c0)?BAD_TICKET:BUSY;
      case(bus_offset)12'h4b0:auth_result<=result_code;12'h4b8:claim_result<=result_code;12'h4c0:pwm_result<=result_code;12'h4c8:revoke_result<=result_code;endcase
      changed;
      if(bus_wstrb==15&&!check_busy&&!draining&&shadow_ready&&(bus_wdata!=0||bus_offset==12'h4c0))begin
       operation<=bus_offset==12'h4b0 ? OP_AUTH : bus_offset==12'h4b8 ? OP_CLAIM : bus_offset==12'h4c0 ? OP_PWM : OP_REVOKE;
       operation_ticket<=bus_wdata;operation_ctx<=ctx;
       check_busy<=1;check_cycles<=0;source_req<=0;source_word<=0;read_wait<=0;check_words<=0;
       source_bad<=0;source_transport<=0;phase<=S_TICKET;scan_word<=0;reset_checks;
      end
     end
    end
   end
  end
 end
endmodule

// One physical shadow memory, 128x32, with arbitrated CPU/checker reads.
// CPU byte writes use a bounded read/modify/write; the checker never sees a
// partially published command because MMIO responds after the RAM write.
module nav_owner_shadow_before(
 input wire clk,reset_n,cpu_req,input wire [6:0]cpu_addr,
 input wire [31:0]cpu_data,input wire [3:0]cpu_strobe,
 output reg cpu_rsp,output reg [31:0]cpu_rdata,output wire ready,
 input wire check_en,input wire [6:0]check_addr,output wire [31:0]check_data
);
 localparam IDLE=0,WAIT_READ=1,WRITE=2;
 reg [1:0]cpu_state;reg pending,initialized;
 reg [6:0]scrub,address;reg [31:0]data,old_data;reg [3:0]strobe;
 wire fixed_address=address==14||address==15||address==17||address==18||address==19||
    address==84||address==85||address==87||address==88||address==89;
 assign ready=initialized;
 wire [31:0]ram_cpu_data;
 reg ram_en,ram_we;reg [6:0]ram_addr;reg [31:0]ram_wdata;
 integer k;
 always @*begin
  ram_en=0;ram_we=0;ram_addr=address;ram_wdata=data;
  if(!initialized)begin ram_en=reset_n;ram_we=1;ram_addr=scrub;
   ram_wdata=(scrub==14||scrub==18||scrub==84||scrub==88)?1:0;
  end else if(check_en)begin ram_en=1;ram_addr=check_addr;end
  else if(cpu_state==IDLE&&pending)ram_en=1;
  else if(cpu_state==WRITE)begin
   ram_en=1;ram_we=1;
   for(k=0;k<4;k=k+1)ram_wdata[k*8+:8]=strobe[k]?data[k*8+:8]:old_data[k*8+:8];
  end
 end
 assign check_data=ram_cpu_data;
 nav_owner_shadow_ram_before storage(.clk(clk),.cpu_en(ram_en),.cpu_we(ram_we),.cpu_addr(ram_addr),
  .cpu_data(ram_wdata),.cpu_q(ram_cpu_data));
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin cpu_state<=IDLE;pending<=0;initialized<=0;scrub<=0;
   address<=0;data<=0;strobe<=0;old_data<=0;cpu_rsp<=0;cpu_rdata<=0;end
  else begin
   cpu_rsp<=0;
   if(cpu_req&&!pending)begin pending<=1;address<=cpu_addr;data<=cpu_data;strobe<=cpu_strobe;end
   if(!initialized)begin if(scrub==127)initialized<=1;else scrub<=scrub+1;end
   else case(cpu_state)
    IDLE:if(pending&&!check_en)cpu_state<=WAIT_READ;
    WAIT_READ:begin
     old_data<=ram_cpu_data;cpu_rdata<=ram_cpu_data;
     if(strobe!=0&&!fixed_address)cpu_state<=WRITE;
     else begin pending<=0;cpu_rsp<=1;cpu_state<=IDLE;end
    end
    WRITE:begin pending<=0;cpu_rsp<=1;cpu_state<=IDLE;end
   endcase
  end
 end
endmodule

module nav_owner_shadow_ram_before(input wire clk,cpu_en,cpu_we,
 input wire[6:0]cpu_addr,input wire[31:0]cpu_data,output reg[31:0]cpu_q);
 (* syn_ramstyle="block_ram" *)reg[31:0]words[0:127];
 always @(posedge clk)if(cpu_en)begin
  if(cpu_we)words[cpu_addr]<=cpu_data;
  else cpu_q<=words[cpu_addr];
 end
endmodule

// Exact unsigned compare, six LUT levels, no new cycle or narrowed sequence.
// g(hi:lo) = g(hi) | (e(hi) & g(lo)); e(hi:lo) = e(hi) & e(lo).
module nav_cancel_order64_before(input wire [63:0] a,b,output wire gt,eq);
 wire [31:0] g,e;
 genvar j;
 generate for(j=0;j<32;j=j+1)begin: bits2
  LUT4 #(.INIT(16'h20f2)) greater(.I0(a[2*j]),.I1(b[2*j]),
    .I2(a[2*j+1]),.I3(b[2*j+1]),.F(g[j]));
  LUT4 #(.INIT(16'h9009)) equal(.I0(a[2*j]),.I1(b[2*j]),
    .I2(a[2*j+1]),.I3(b[2*j+1]),.F(e[j]));
 end endgenerate
 nav_cancel_order_reduce_before #(.N(32)) tree(.g(g),.e(e),.gt(gt),.eq(eq));
endmodule
module nav_cancel_order_reduce_before #(parameter integer N=32)(
 input wire [N-1:0] g,e,output wire gt,eq);
 generate if(N==1)begin: leaf
  assign gt=g[0];assign eq=e[0];
 end else begin: level
  wire [N/2-1:0] ng,ne;
  genvar j;
  for(j=0;j<N/2;j=j+1)begin: pairs
   LUT4 #(.INIT(16'heaea)) greater(.I0(g[2*j+1]),.I1(e[2*j+1]),
     .I2(g[2*j]),.I3(1'b0),.F(ng[j]));
   LUT4 #(.INIT(16'h8888)) equal(.I0(e[2*j+1]),.I1(e[2*j]),
     .I2(1'b0),.I3(1'b0),.F(ne[j]));
  end
  nav_cancel_order_reduce_before #(.N(N/2)) next(.g(ng),.e(ne),.gt(gt),.eq(eq));
 end endgenerate
endmodule
