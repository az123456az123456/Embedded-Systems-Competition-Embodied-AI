`timescale 1ns/1ps
// Electrical-protocol model: actual bit shifter, ACKs and repeated START.
module tb_i2c;
 reg clk=0,rst_n=0,wr=0,rd=0;always #10 clk=~clk;
 reg [15:0]addr=0;reg [7:0]data=0;wire [7:0]readback;wire done,nack,scl;
 tri1 sda;reg pull_low=0,deny_ack=0;
 assign sda=pull_low?1'b0:1'bz;
 i2c_control dut(.Clk(clk),.Rst_n(rst_n),.wrreg_req(wr),.rdreg_req(rd),.addr(addr),.addr_mode(1'b1),
 .wrdata(data),.rddata(readback),.device_id(8'h78),.RW_Done(done),.ack(nack),.dly_cnt_max(32'd0),.i2c_sclk(scl),.i2c_sdat(sda));
 reg [7:0]regs[0:65535];integer i,state=4,nextstate=0,bitnum=0,bytephase=0;
 reg [7:0]shift=0,received;reg [15:0]pointer=0;
 initial begin for(i=0;i<65536;i=i+1)regs[i]=0;regs['h300a]='h56;regs['h300b]='h40;end
 always @(negedge sda) if(scl===1'b1 && rst_n)begin state=0;bitnum=0;bytephase=0;shift=0;end
 always @(posedge sda) if(scl===1'b1 && rst_n)state=4;
 always @(posedge scl)if(rst_n)begin
  case(state)
   0:begin
    shift={shift[6:0],sda};
    if(bitnum==7)begin
     received=shift;nextstate=0;bitnum=0;
     case(bytephase)
      0:begin
       if((received&8'hfe)!=8'h78)$fatal(1,"wrong device address %h",received);
       nextstate=received[0]?2:0;bytephase=1;
      end
      1:begin pointer[15:8]=received;bytephase=2;end
      2:begin pointer[7:0]=received;bytephase=3;end
      3:regs[pointer]=received;
     endcase
     state=1;
    end else bitnum=bitnum+1;
   end
   1:begin state=nextstate;bitnum=0;end
   2:begin if(bitnum==7)begin state=3;bitnum=0;end else bitnum=bitnum+1;end
   3:state=4; // single-byte reads end with master NACK and STOP
  endcase
 end
 always @(negedge scl)begin
  case(state)
   1:pull_low=!deny_ack;
   2:pull_low=!regs[pointer][7-bitnum];
   default:pull_low=0;
  endcase
 end
 task transaction(input isread,input [15:0]a,input [7:0]v,input expected_nack,input [7:0]expected);
  begin
   @(negedge clk);addr=a;data=v;wr=!isread;rd=isread;
   @(negedge clk);wr=0;rd=0;
   wait(done);#1;
   if(nack!==expected_nack)$fatal(1,"NACK mismatch addr=%h got=%b",a,nack);
   if(isread && !expected_nack && readback!==expected)$fatal(1,"read mismatch addr=%h got=%h expected=%h",a,readback,expected);
   wait(!done);repeat(5)@(negedge clk);
  end
 endtask
 initial begin
  #1000;rst_n=1;
  transaction(1,'h300a,0,0,'h56);transaction(1,'h300b,0,0,'h40);
  transaction(0,'h3503,3,0,0);transaction(1,'h3503,0,0,3);
  transaction(0,'h3406,1,0,0);transaction(1,'h3406,0,0,1);
  transaction(0,'h3501,'h21,0,0);transaction(1,'h3501,0,0,'h21);
  deny_ack=1;transaction(1,'h300a,0,1,0);
  $display("PASS actual SCCB bit-level writes / readback / repeated START / NACK");$finish;
 end
 initial begin #5000000;$fatal(1,"I2C test timeout");end
endmodule
