`timescale 1ns/1ps
module tb_wire;
 reg txclk=0,pclk0=0,pclk1=0,rst_n=0;
 always #4 txclk=~txclk;
 always #6 pclk0=~pclk0;
 initial begin #2;forever #6 pclk1=~pclk1;end
 reg h0=0,v0=0,h1=0,v1=0;reg [7:0]d0=0,d1=0;
 wire c0,c1,w0,w1;wire [7:0]q0,q1;
 reg [7:0]profile_bytes[0:151];reg [1215:0]profile;integer k;
 initial begin $readmemh("build/profile.hex",profile_bytes);for(k=0;k<152;k=k+1)profile[1215-k*8 -:8]=profile_bytes[k];end
 Camera_ETH_Formator f0(pclk0,rst_n,1'b1,h0,v0,d0,c0,q0,w0);
 Camera_ETH_Formator f1(pclk1,rst_n,1'b1,h1,v1,d1,c1,q1,w1);
 wire wc,clear,we,fault;wire [7:0]data;
 controller_top #(.META_INTERVAL(125000)) rows(.profile_info(profile),.pair_fault(fault),.clk(txclk),.rst_n(rst_n),
  .wrclk1(pclk0),.fifo1_aclr(c0),.wrdata1(q0),.wrreq1(w0),.wrclk2(pclk1),.fifo2_aclr(c1),.wrdata2(q1),.wrreq2(w1),
  .wrclk(wc),.fifo_aclr(clear),.fifo_wrreq(we),.fifo_wrdata(data));
 wire [7:0]txdata;wire valid;
 UDP_Send send(.tx_allow(1'b1),.tx_busy(),.Clk(1'b0),.Rst_n(rst_n),.Tx_Done(),
  .des_mac(48'hffffffffffff),.src_mac(48'h000a3501fec0),.des_port(16'd6102),.src_port(16'd5000),
  .des_ip(32'hc0a80003),.src_ip(32'hc0a80002),.data_length(16'd1922),.GMII_GTXC(txclk),.GMII_TXD(txdata),.GMII_TXEN(valid),
  .wrreq(we),.wrdata(data),.wrclk(wc),.aclr(clear),.wrusedw());
 integer wirefile,indexfile,offset=0,length=0,cycles=0,last_end=-1000,start_cycle,packets=0;reg wasvalid=0;
 initial begin wirefile=$fopen("build/wire.bin","wb");indexfile=$fopen("build/wire-index.txt","w");end
 always @(negedge txclk)begin
  cycles=cycles+1;
  if(valid)begin
   if(!wasvalid)begin start_cycle=cycles;if(cycles-last_end-1<12)$fatal(1,"short IFG");end
   $fwrite(wirefile,"%c",txdata);length=length+1;
  end else if(wasvalid)begin
   $fwrite(indexfile,"%0d %0d %0d\n",offset,length,start_cycle-last_end-1);offset=offset+length;length=0;last_end=cycles-1;packets=packets+1;
  end
  wasvalid=valid;
 end
 reg complete0=0,complete1=0;
 initial begin:cam0 integer f,y,x;wait(rst_n);repeat(10)@(negedge pclk0);
  for(f=0;f<2;f=f+1)begin
   v0=1;repeat(8)@(negedge pclk0);v0=0;repeat(16)@(negedge pclk0);
   for(y=0;y<1080;y=y+1)begin
    @(negedge pclk0);h0=1;d0=(3*y+7*f)&255;
    for(x=1;x<960;x=x+1)begin @(negedge pclk0);d0=(x+3*y+7*f)&255;end
    @(negedge pclk0);h0=0;repeat(919)@(negedge pclk0);
   end
   repeat(1880*40)@(negedge pclk0);
  end
  complete0=1;
 end
 initial begin:cam1 integer f,y,x;wait(rst_n);repeat(10)@(negedge pclk1);
  for(f=0;f<2;f=f+1)begin
   v1=1;repeat(8)@(negedge pclk1);v1=0;repeat(16)@(negedge pclk1);
   for(y=0;y<1080;y=y+1)begin
    @(negedge pclk1);h1=1;d1=(5*y+11*f+40)&255;
    for(x=1;x<960;x=x+1)begin @(negedge pclk1);d1=(2*x+5*y+11*f+40)&255;end
    @(negedge pclk1);h1=0;repeat(919)@(negedge pclk1);
   end
   repeat(1880*40)@(negedge pclk1);
  end
  complete1=1;
 end
 initial begin #200;rst_n=1;wait(complete0 && complete1);repeat(20000)@(negedge txclk);
  if(fault)$fatal(1,"unexpected pairing fault");$fclose(wirefile);$fclose(indexfile);$display("PASS full DVP wire packets=%0d bytes=%0d",packets,offset);$finish;end
 initial begin #80000000;$fatal(1,"wire test timeout");end
endmodule
