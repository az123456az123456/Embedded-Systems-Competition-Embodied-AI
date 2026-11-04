`timescale 1ns/1ps
// Local MMIO router. Packet data and authority live in their assigned modules;
// this wrapper never grants permission, copies packet bodies, or renews time.
module nav_exec_io (
 input wire cpu_clk,cpu_reset_n,rx_clk,rx_reset_n,tx_clk,tx_reset_n,
 input wire [31:0] device_ms,
 input wire [7:0] rx_data,input wire rx_valid,rx_error,
 input wire bus_req,input wire [11:0] bus_offset,
 input wire [31:0] bus_wdata,input wire [3:0] bus_wstrb,
 output wire bus_rsp_valid,output wire [31:0] bus_rsp_data,
 input wire cpu_trap,mot_accept,manual_quiet,
 input wire [31:0] local_sample_id,local_sample_ms,local_imu_ms,
 input wire [31:0] local_sample_deadline,local_imu_deadline,
    input wire local_sample_valid,
 input wire [47:0] raw_sonar_mm,input wire [2:0] raw_sonar_online,
 output wire nav_owned,nav_live,output wire [63:0] nav_pwm,
 output wire [15:0] nav_linear,nav_angular,
 output wire [63:0] nav_epoch,output wire [31:0] context_generation,
 input wire tx_allow,output wire tx_pending,tx_busy,gmii_valid,
 output wire [7:0] gmii_data,output reg reserved_access
);
 wire ingress_projection=(bus_offset==12'hbc4||bus_offset==12'hbc8||bus_offset==12'hbcc);
 wire [11:0] ingress_offset=ingress_projection ? (bus_offset-12'h72c) : bus_offset;
 wire to_ingress=(ingress_projection||(bus_offset>=12'h424&&bus_offset<12'h434)||
                  (bus_offset>=12'h440&&bus_offset<12'h48c)||
                  (bus_offset>=12'h498&&bus_offset<12'h4a4)||
                   bus_offset==12'h4a8||
                  (bus_offset>=12'h500&&bus_offset<12'h600)||
                  (bus_offset>=12'h700&&bus_offset<12'h8c0));
 wire to_tx=((bus_offset>=12'h434&&bus_offset<12'h440)||
             (bus_offset>=12'h490&&bus_offset<12'h498)||
             (bus_offset>=12'h600&&bus_offset<12'h700));
 wire to_owner=((bus_offset>=12'h400&&bus_offset<12'h424)||
                 bus_offset==12'h48c||bus_offset==12'h4a4||
                (bus_offset>=12'h4ac&&bus_offset<12'h500)||
                (bus_offset>=12'h900&&bus_offset<12'hc00&&!ingress_projection));
 wire aligned=bus_offset[1:0]==2'b00;
 wire ingress_req=bus_req&&aligned&&to_ingress;
 wire owner_req=bus_req&&aligned&&to_owner;
 wire tx_req=bus_req&&aligned&&to_tx;
 wire irsp,orsp,trsp;
 wire [31:0] idata,odata,tdata;
 reg invalid_rsp;
 always @(posedge cpu_clk or negedge cpu_reset_n)begin
  if(!cpu_reset_n)begin invalid_rsp<=0;reserved_access<=0;end
  else begin
   invalid_rsp<=bus_req&&(!aligned||!(to_ingress||to_owner||to_tx));
   if(bus_req&&(!aligned||!(to_ingress||to_owner||to_tx)))reserved_access<=1;
  end
 end
 assign bus_rsp_valid=irsp||orsp||trsp||invalid_rsp;
 assign bus_rsp_data=({32{irsp}}&idata)|({32{orsp}}&odata)|({32{trsp}}&tdata);

 wire [127:0] pin_tokens,pin_rx_ms;
 wire [15:0] pin_flags;
 wire [35:0] pin_bytes;
 wire source_req,source_grant,source_valid;
 wire [1:0] source_lane;
 wire [5:0] source_word;
 wire [31:0] source_data,last_tx_ticket;
 wire cancel_valid,cancel_ready;
 wire [417:0] cancel_desc;
 nav3_ingress ingress(
  .rx_clk(rx_clk),.rx_reset_n(rx_reset_n),.cpu_clk(cpu_clk),.cpu_reset_n(cpu_reset_n),
  .device_ms(device_ms),.gmii_data(rx_data),.gmii_valid(rx_valid),.gmii_error(rx_error),
  .bus_req(ingress_req),.bus_offset(ingress_offset),.bus_wdata(bus_wdata),.bus_wstrb(bus_wstrb),
  .bus_rsp_valid(irsp),.bus_rsp_data(idata),
  .pin_tokens(pin_tokens),.pin_rx_ms(pin_rx_ms),.pin_flags(pin_flags),.pin_bytes(pin_bytes),
  .source_req(source_req),.source_lane(source_lane),.source_word(source_word),
  .source_grant(source_grant),.source_valid(source_valid),.source_data(source_data),
  .cancel_valid(cancel_valid),.cancel_desc(cancel_desc),.cancel_ready(cancel_ready));
 nav_owner_gate owner_gate(
  .clk(cpu_clk),.reset_n(cpu_reset_n),.device_ms(device_ms),
  .cpu_trap(cpu_trap),.mot_accept(mot_accept),.manual_quiet(manual_quiet),
  .raw_sonar_mm(raw_sonar_mm),.raw_sonar_online(raw_sonar_online),
  .local_sample_id(local_sample_id),.local_sample_ms(local_sample_ms),.local_imu_ms(local_imu_ms),.local_sample_valid(local_sample_valid),
  .local_sample_deadline(local_sample_deadline),.local_imu_deadline(local_imu_deadline),
  .bus_req(owner_req),.bus_offset(bus_offset),.bus_wdata(bus_wdata),.bus_wstrb(bus_wstrb),
  .bus_rsp_valid(orsp),.bus_rsp_data(odata),
  .pin_tokens(pin_tokens),.pin_rx_ms(pin_rx_ms),.pin_flags(pin_flags),.pin_bytes(pin_bytes),
  .source_req(source_req),.source_lane(source_lane),.source_word(source_word),
  .source_grant(source_grant),.source_valid(source_valid),.source_data(source_data),
  .cancel_valid(cancel_valid),.cancel_desc(cancel_desc),.cancel_ready(cancel_ready),
  .last_tx_ticket(last_tx_ticket),.nav_owned(nav_owned),.nav_live(nav_live),.nav_pwm(nav_pwm),
  .nav_linear(nav_linear),.nav_angular(nav_angular),.nav_epoch(nav_epoch),
  .context_generation(context_generation));
 nav3_tx_queue transmit(
  .cpu_clk(cpu_clk),.cpu_reset_n(cpu_reset_n),.tx_clk(tx_clk),.tx_reset_n(tx_reset_n),
  .bus_req(tx_req),.bus_offset(bus_offset),.bus_wdata(bus_wdata),.bus_wstrb(bus_wstrb),
  .bus_rsp_valid(trsp),.bus_rsp_data(tdata),.last_tx_ticket(last_tx_ticket),
  .tx_allow(tx_allow),.tx_pending(tx_pending),.tx_busy(tx_busy),
  .gmii_valid(gmii_valid),.gmii_data(gmii_data));
endmodule
