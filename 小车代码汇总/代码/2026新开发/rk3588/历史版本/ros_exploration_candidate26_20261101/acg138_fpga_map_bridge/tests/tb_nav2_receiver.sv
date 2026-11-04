`timescale 1ns/1ps
module tb_nav2_receiver;
  reg clk=0;always #4 clk=~clk;
  reg rst_n=0,valid=0,error=0;reg[7:0] data=0;
  wire command_valid;wire[511:0] command_data;wire[31:0] accepted,rejected;
  nav_packet_rx dut(.clk(clk),.rst_n(rst_n),.data(data),.valid(valid),.error(error),
    .command_valid(command_valid),.command_data(command_data),.accepted_frames(accepted),.rejected_frames(rejected));
  reg[7:0] bytes[0:551];integer i,j,seen=0;reg[511:0] body;
  always @(posedge clk)begin #1;if(command_valid)begin
    if(command_data!==body)$fatal(1,"receiver payload does not match encoded body");seen=seen+1;
  end end
  initial begin
    $readmemh("frames.hex",bytes);
    repeat(4)@(negedge clk);rst_n=1;
    for(i=0;i<6;i=i+1)begin
      body=0;for(j=0;j<64;j=j+1)body={body[503:0],bytes[i*92+22+j]};
      for(j=0;j<92;j=j+1)begin @(negedge clk);valid=1;data=bytes[i*92+j];end
      @(negedge clk);valid=0;data=0;repeat(5)@(negedge clk);
    end
    if(accepted!==2||rejected!==4||seen!=2)$fatal(1,"receiver accepted/rejected count wrong %d %d %d",accepted,rejected,seen);
    $display("PASS nav2_receiver actual_rtl=yes accepted=%0d rejected=%0d payloads=%0d",accepted,rejected,seen);$finish;
  end
  initial begin #100000;$fatal(1,"timeout");end
endmodule
