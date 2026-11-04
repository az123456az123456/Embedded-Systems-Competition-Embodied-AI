// Five independent registered launch grants. Producers own complete Ethernet
// frames, including their 12-byte IFG, through busy. Grant is a one-cycle
// opportunity, never an asynchronous permission to preempt a running sender.
// One cooldown clock lets each producer's registered busy settle after launch.
// Camera intentionally has no pending input: it gets an idle opportunity.
module robot_gmii_share(
 input wire clk,rst_n,
 input wire camera_busy,camera_valid,input wire[7:0]camera_data,
 input wire imu_pending,imu_busy,imu_valid,input wire[7:0]imu_data,
 input wire motor_pending,motor_busy,motor_valid,input wire[7:0]motor_data,
 input wire sensor_pending,sensor_busy,sensor_valid,input wire[7:0]sensor_data,
 input wire planner_pending,planner_busy,planner_valid,input wire[7:0]planner_data,
 output wire camera_allow,imu_allow,motor_allow,sensor_allow,planner_allow,
 output wire[7:0]gmii_data,output wire gmii_valid
);
 reg[4:0]grant;
 reg cooldown;
 wire any_busy=camera_busy||imu_busy||motor_busy||sensor_busy||planner_busy||
               camera_valid||imu_valid||motor_valid||sensor_valid||planner_valid;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin grant<=0;cooldown<=0;end
  else if(|grant)begin grant<=0;cooldown<=1;end
  else if(cooldown)cooldown<=0;
  else if(!any_busy)begin
   if(motor_pending)grant<=5'b00100;
   else if(sensor_pending)grant<=5'b01000;
   else if(planner_pending)grant<=5'b10000;
   else if(imu_pending)grant<=5'b00010;
   else grant<=5'b00001;
  end
 end
 // Direct flip-flop outputs: do not gate a shared planner grant with pending
 // or unrelated state, which reintroduces the failed long CE timing cone.
 assign {planner_allow,sensor_allow,motor_allow,imu_allow,camera_allow}=grant;
 assign gmii_valid=camera_valid||imu_valid||motor_valid||sensor_valid||planner_valid;
 assign gmii_data=motor_valid?motor_data:sensor_valid?sensor_data:planner_valid?planner_data:imu_valid?imu_data:camera_data;
endmodule
