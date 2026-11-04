#!/usr/bin/env python3
"""Exercise the actual shared DDR receiver, pipeline and both protocol parsers."""
import binascii
import os
from pathlib import Path
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build'
BUILD.mkdir(exist_ok=True)
head = bytes.fromhex('02ac1380000102ac13800002')
motor = b'MOT1' + bytes([1, 1]) + struct.pack('>IhhBB', 12, 300, -300, 12, 15)
nav = b'NAV2' + bytes([1, 6, 0, 0]) + bytes(56)


def frame(payload, ethertype, bad_crc=False):
    data = head + struct.pack('>H', ethertype) + payload
    data += struct.pack('>H', binascii.crc_hqx(payload, 65535) ^ bad_crc)
    data = data.ljust(60, b'\0')
    return b'\x55' * 7 + b'\xd5' + data + struct.pack('<I', binascii.crc32(data))


m = frame(motor, 0x88b6)
n = frame(nav, 0x88b9)
cases = [(m, 1, 0, -1), (m[:-1] + bytes([m[-1] ^ 1]), 0, 0, -1),
         (frame(motor, 0x88b6, True), 0, 0, -1), (m, 1, 0, -1),
         (n, 0, 1, -1), (frame(b'FAIL' + nav[4:], 0x88b9), 0, 0, -1),
         (frame(nav, 0x88b9, True), 0, 0, -1),
         (n[:-1] + bytes([n[-1] ^ 1]), 0, 0, -1), (n[:-1], 0, 0, -1),
         (n + b'\0', 0, 0, -1), (n, 0, 0, 35), (n, 0, 1, -1), (m, 1, 0, -1)]
width = max(len(f) for f, *_ in cases)
(BUILD / 'robot_rx_frames.hex').write_text(''.join(f.hex().ljust(width * 2, '0') + '\n' for f, *_ in cases))
lengths = '\n'.join(f'lengths[{i}]={len(f)}; errbyte[{i}]={e}; expect_m[{i}]={a}; expect_n[{i}]={b};'
                    for i, (f, a, b, e) in enumerate(cases))
bench = f'''`timescale 1ns/1ps
module tb_robot_rx;
GSR GSR(.GSRI(1'b1));
reg clk=0,rst=0,ctl=0;reg[3:0]rxd=0;always #4 clk=~clk;
wire mv,nv;wire[127:0]md;wire[511:0]nd;
robot_rgmii_rx dut(.reset_n(rst),.rgmii_rx_clk(clk),.rgmii_rxd(rxd),.rgmii_rx_ctl(ctl),
 .command_valid(mv),.command_data(md),.nav_valid(nv),.nav_data(nd),.receive_reset_n());
reg[{width*8-1}:0] frames[0:{len(cases)-1}];
integer lengths[0:{len(cases)-1}],errbyte[0:{len(cases)-1}],expect_m[0:{len(cases)-1}],expect_n[0:{len(cases)-1}];
integer i,j,mc=0,nc=0,lastm,lastn;reg[7:0]v;
always @(posedge clk)begin
 if(mv)begin mc=mc+1;if(md!==128'h{motor.hex()})$fatal(1,"MOT1 payload");end
 if(nv)begin nc=nc+1;if(nd!==512'h{nav.hex()})$fatal(1,"NAV2 payload");end
end
initial begin
 $readmemh("build/robot_rx_frames.hex",frames);
 {lengths}
 repeat(6)@(negedge clk);rst=1;repeat(6)@(negedge clk);
 for(i=0;i<{len(cases)};i=i+1)begin
  lastm=mc;lastn=nc;
  for(j=0;j<lengths[i];j=j+1)begin
   v=frames[i][{width*8-1}-j*8 -:8];
   @(negedge clk);#1;rxd=v[3:0];ctl=1;
   @(posedge clk);#1;rxd=v[7:4];ctl=(j==errbyte[i])?0:1;
  end
  @(negedge clk);#1;ctl=0;rxd=0;repeat(12)@(negedge clk);
  if(mc-lastm!=expect_m[i]||nc-lastn!=expect_n[i])$fatal(1,"case%0d MOT=%0d NAV=%0d",i,mc-lastm,nc-lastn);
 end
 $display("PASS actual Gowin IDDR + registered GMII:13 cases, MOT1/NAV2 payload/FCS/CRC16/RX_ER/length/recovery/minimum IFG");$finish;
end
initial begin #200000;$fatal(1,"timeout");end
endmodule
'''
(BUILD / 'tb_robot_rx.sv').write_text(bench)
subprocess.run(['iverilog', '-g2012', '-s', 'tb_robot_rx', '-o', 'build/robot-rx',
                'rtl/robot_rgmii_rx.v', 'rtl/motor_packet_rx.v', 'rtl/nav_packet_rx.v',
                'build/tb_robot_rx.sv', str(Path(os.environ.get('GOWIN_IDE', ROOT.parents[1] / 'Gowin_V1.9.12.03/IDE')) / 'simlib/gw5a/prim_sim.v')],
               cwd=ROOT, check=True)
subprocess.run(['vvp', 'build/robot-rx'], cwd=ROOT, check=True)
