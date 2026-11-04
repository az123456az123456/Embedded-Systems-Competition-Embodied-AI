#!/usr/bin/env python3
import binascii,subprocess,struct
from pathlib import Path
r=Path(__file__).resolve().parents[2];b=r/'build';b.mkdir(exist_ok=True)
q=b'NAV2'+bytes([1,6,0,0])+bytes(56)
head=bytes.fromhex('02ac1380000102ac1380000288b9')
def make(h=head,p=q,badcrc=False):
 c=binascii.crc_hqx(p,65535)^(1 if badcrc else 0);x=h+p+struct.pack('>H',c)
 return b'\x55'*7+b'\xd5'+x+struct.pack('<I',binascii.crc32(x))
valid=make();cases=[(valid,1)]
cases +=[(make(bytes([head[0]^1])+head[1:]),0),(make(head[:-1]+b'\xb8'),0),
 (make(p=q[:4]+b'\x02'+q[5:]),0),(make(p=q[:28]+b'\x01'+q[29:]),0),(make(badcrc=True),0),
 (valid[:-1]+bytes([valid[-1]^1]),0),(valid[:-1],0),(valid+b'\0',0)]
(b/'nav_rx_frames.hex').write_text('\n'.join(f.hex().ljust(93*2,'0') for f,_ in cases)+'\n')
bench='''`timescale 1ns/1ps
module tb_nav_rx;
reg clk=0,rst=0,valid=0;reg[7:0]data=0;wire accepted;wire[511:0]payload;wire[31:0]ac,rej;
always #4 clk=~clk;
nav_packet_rx dut(.clk(clk),.rst_n(rst),.data(data),.valid(valid),.error(1'b0),.command_valid(accepted),.command_data(payload),.accepted_frames(ac),.rejected_frames(rej));
reg[743:0] frames[0:8];integer i,j;integer lengths[0:8];
initial begin $readmemh("build/nav_rx_frames.hex",frames);
for(i=0;i<9;i=i+1)lengths[i]=92;lengths[7]=91;lengths[8]=93;
repeat(5)@(negedge clk);rst=1;
for(i=0;i<9;i=i+1)begin
for(j=0;j<lengths[i];j=j+1)begin @(negedge clk);valid=1;data=frames[i][743-j*8 -:8];end
@(negedge clk);valid=0;repeat(15)@(negedge clk);
end
if(ac!=1||rej!=8)$fatal(1,"NAV RX validation failed accepted=%0d rejected=%0d",ac,rej);
$display("PASS NAV RX destination, EtherType, magic/version/reserved, CRC16, FCS, exact length rejection");$finish;end
endmodule
'''
(b/'tb_nav_rx.v').write_text(bench)
subprocess.run(['iverilog','-g2012','-s','tb_nav_rx','-o','build/nav-rx','rtl/nav_packet_rx.v','build/tb_nav_rx.v'],cwd=r,check=True)
subprocess.run(['vvp','build/nav-rx'],cwd=r,check=True)
