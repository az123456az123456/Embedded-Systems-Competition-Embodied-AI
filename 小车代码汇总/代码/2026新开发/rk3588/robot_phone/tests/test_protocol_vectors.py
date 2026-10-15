"""Cross-check the native host codec against the FPGA project's wire fixtures.

The fixtures come from robot_fmc_remote's protocol/RTL verification. This builds
the real host motor_link.cc, rather than duplicating its encoder in Python.
No raw socket, development board, or network access is used.
"""
import binascii
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT.parent.parent / "fpga/robot_fmc_remote/reports/protocol_vectors.json"

TOOL_SOURCE = r'''
#include "motor_link.h"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace robotphone;
static const uint8_t host[]={2,3,4,5,6,7};
static std::string hex(const std::vector<uint8_t> &bytes){
 std::ostringstream out;
 for(uint8_t byte:bytes)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);
 return out.str();
}
int main(int argc,char **argv){
 if(argc<2)return 2;
 std::string command=argv[1];
 if(command=="stop"){
  std::cout<<hex(motorFrame(host,1,false,0,0,3,12))<<'\n';return 0;
 }
 if(command=="drive"){
  std::cout<<hex(motorFrame(host,2,true,300,-300,3,12))<<'\n';return 0;
 }
 if(command=="ack"&&argc==3){
  std::string input=argv[2];std::vector<uint8_t> bytes;
  if(input.size()%2)return 2;
  for(size_t i=0;i<input.size();i+=2){
   char *end=nullptr;std::string pair=input.substr(i,2);
   unsigned long byte=std::strtoul(pair.c_str(),&end,16);
   if(!end||*end||byte>255)return 2;
   bytes.push_back(static_cast<uint8_t>(byte));
  }
  MotorAck a;
  if(!decodeMotorAck(bytes.data(),bytes.size(),host,a))return 3;
  if(a.sequence!=2||!a.armed||a.watchdog||!a.configured||
     a.left!=300||a.right!=-300||a.left_mask!=3||a.invert_mask!=12)return 4;
  std::cout<<"ACK1 seq2 armed configured left300 right-300 mask3 invert12\n";
  return 0;
 }
 return 2;
}
'''


def main():
    vectors = json.loads(FIXTURE.read_text())
    assert len(vectors) == 3, "Unexpected FPGA fixture schema/count"
    commands = [v for v in vectors if v["magic"] == "MOT1"]
    acks = [v for v in vectors if v["magic"] == "ACK1"]
    assert len(commands) == 2 and len(acks) == 1
    with tempfile.TemporaryDirectory(prefix="robot-host-fpga-vectors-") as temporary:
        source, tool = Path(temporary) / "codec.cc", Path(temporary) / "codec"
        source.write_text(TOOL_SOURCE)
        subprocess.run(
            shlex.split(os.environ.get("CXX", "c++"))
            + ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2",
               "-I", str(ROOT / "backend"), str(ROOT / "backend/motor_link.cc"),
               str(source), "-o", str(tool)],
            check=True,
        )
        checks = 0
        for vector in vectors:
            frame = bytes.fromhex(vector["ethernet60_hex"])
            payload = bytes.fromhex(vector["payload18_hex"])
            wire = bytes.fromhex(vector["wire64_hex"])
            assert len(frame) == 60 and len(payload) == 18 and len(wire) == 64
            assert frame[14:32] == payload and wire[:60] == frame
            assert wire[60:] == binascii.crc32(frame).to_bytes(4, "little")
            checks += 3
        for mode, vector in zip(("stop", "drive"), commands):
            encoded = subprocess.check_output([str(tool), mode], text=True).strip()
            assert encoded == vector["ethernet60_hex"], f"C++ {mode} differs from FPGA bytes"
            checks += 1
        for field in ("ethernet60_hex", "wire64_hex"):
            decoded = subprocess.check_output([str(tool), "ack", acks[0][field]], text=True)
            assert "ACK1 seq2 armed configured left300 right-300 mask3 invert12" in decoded
            checks += 1
        corrupted = bytearray.fromhex(acks[0]["wire64_hex"])
        corrupted[-1] ^= 1
        rejected = subprocess.run([str(tool), "ack", corrupted.hex()], capture_output=True)
        assert rejected.returncode == 3, "Host accepted corrupt FPGA ACK Ethernet FCS"
        checks += 1
    print(f"PASS {checks} FPGA/host fixture integration checks: exact C++ MOT1 STOP/drive, ACK1 payload/FCS")


if __name__ == "__main__":
    main()
