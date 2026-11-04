`timescale 1ns/1ps
module tb_status_ce_equivalence;
 reg clk=0,rst_n=0,valid=0;
 reg [127:0] payload=0;reg [207:0] nav_payload=0;
 wire [1:0] ready,ref_ready,dv,ref_dv;
 wire [7:0] data[0:1],ref_data[0:1];
 integer cycles=0,checks=0,accepted=0,active=0;
 always #4 clk=~clk;
 genvar g;
 generate for(g=0;g<2;g=g+1)begin: pair
  motor_status_tx #(.NAV_ENABLE(g)) dut(.clk(clk),.rst_n(rst_n),.valid(valid),.payload(payload),.nav_payload(nav_payload),.ready(ready[g]),.gmii_data(data[g]),.gmii_valid(dv[g]));
  motor_status_tx_pre_ce #(.NAV_ENABLE(g)) original(.clk(clk),.rst_n(rst_n),.valid(valid),.payload(payload),.nav_payload(nav_payload),.ready(ref_ready[g]),.gmii_data(ref_data[g]),.gmii_valid(ref_dv[g]));
 end endgenerate
 always @(negedge clk)if(rst_n)begin
  valid=$urandom_range(0,3)!=0;
  // Change every byte even while backpressured and immediately after grant.
  // A sender that captures early, late or consumes before ready diverges.
  payload={$urandom,$urandom,$urandom,$urandom};
  nav_payload={$urandom,$urandom,$urandom,$urandom,$urandom,$urandom,$urandom};
 end
 integer i;
 always @(posedge clk)begin
  if(rst_n)begin
   cycles=cycles+1;
   if(valid&&ready[1])accepted=accepted+1;
   #1;
   for(i=0;i<2;i=i+1)begin
    checks=checks+1;
    if({ready[i],dv[i],data[i]} !== {ref_ready[i],ref_dv[i],ref_data[i]})
     $fatal(1,"sender cycle mismatch cycle=%0d NAV_ENABLE=%0d new=%h old=%h",cycles,i,{ready[i],dv[i],data[i]},{ref_ready[i],ref_dv[i],ref_data[i]});
    if(dv[i])active=active+1;
   end
   if(cycles==20000)begin
    if(accepted<50||active<10000)$fatal(1,"insufficient coverage");
    $display("PASS sender CE cycle-equivalence checks=%0d accepted_NAV_snapshots=%0d emitted_bytes=%0d, changing idle/accept/busy bundles, exact ACK/NAV/CRC/FCS/IFG timing",checks,accepted,active);$finish;
   end
  end
 end
 initial begin repeat(4)@(negedge clk);rst_n=1;end
 initial begin #200000;$fatal(1,"equivalence timeout");end
endmodule
