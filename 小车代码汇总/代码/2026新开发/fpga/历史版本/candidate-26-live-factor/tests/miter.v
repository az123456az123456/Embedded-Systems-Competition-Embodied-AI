module miter(input [1:0] owner,input [385:0] cancel_desc,input [1:0] pwm_flags,input bound,task_valid,started,continuation,plan_ready,map_fenced,revoked,exhausted,pose_fresh,permit_fresh,auth_fresh,cpu_fresh,cpu_trap,trap_previous,mot_accept,cancel_new,permit_qualified,auth_qualified,pose_qualified,raw_block,output mismatch);
localparam EV_MOT=32'h1,EV_CANCEL=32'h2,EV_REVOKE=32'h4,EV_MOTION=32'h8,EV_AUTH=32'h10,EV_POSE=32'h20,EV_TRAP=32'h40,EV_REFERENCE=32'h80,EV_EXHAUST=32'h100;
wire reference_live,nav_live;
 wire [31:0] asynchronous_reasons=(mot_accept?EV_MOT:0)|
  (cancel_new&&!revoked?(cancel_desc[1:0]==1?EV_CANCEL:EV_REVOKE):0)|
  (started&&permit_qualified&&!permit_fresh?EV_MOTION:0)|
  (started&&auth_qualified&&!auth_fresh?EV_AUTH:0)|
  (started&&pose_qualified&&!pose_fresh?EV_POSE:0)|
  (cpu_trap&&!trap_previous?EV_TRAP:0);
 wire hard_event=asynchronous_reasons!=0;
 assign reference_live=owner==2&&bound&&task_valid&&started&&continuation&&plan_ready&&!map_fenced&&!revoked&&!exhausted&&
  pose_fresh&&permit_fresh&&auth_fresh&&cpu_fresh&&pwm_flags[1:0]==2'b11&&!cpu_trap&&!raw_block&&!hard_event;
 assign nav_live=owner==2&&bound&&task_valid&&started&&continuation&&plan_ready&&!map_fenced&&!revoked&&!exhausted&&
  pose_fresh&&permit_fresh&&auth_fresh&&cpu_fresh&&pwm_flags[1:0]==2'b11&&!cpu_trap&&!raw_block&&!mot_accept&&!cancel_new;
assign mismatch=nav_live!=reference_live;endmodule
