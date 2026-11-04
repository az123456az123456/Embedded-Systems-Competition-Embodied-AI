#!/usr/bin/env python3
"""Cycle-exact randomized reference check for idle-preloaded ACK/NAV sender."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def main():
 target=ROOT/'build/status-ce-equivalence';target.parent.mkdir(exist_ok=True)
 commands=[['iverilog','-g2012','-s','tb_status_ce_equivalence','-o',str(target),'rtl/motor_status_tx.v','tests/motor_status_tx_ce_reference.v','tests/tb_status_ce_equivalence.sv'],['vvp',str(target)]]
 for command in commands:subprocess.run(command,cwd=ROOT,check=True)
if __name__=='__main__':main()
