---
name: fpga-i2c-master
description: Design and verify a synthesizable I2C master in Verilog, with the timing/handshake pitfalls that break RTL simulation (sda-change-while-scl-high races, level-triggered op re-issue, missing busy handshake, read-burst NACK handling). Use when writing an I2C master or debugging I2C transactions in simulation.
metadata:
  short-description: I2C master Verilog design + simulation pitfalls
---

# I2C master design pitfalls (Verilog)

Verified lessons from an I2C master + simulated MPU6050 slave (iverilog).

1. **Bit engine must change sda only while scl is low.** Use a 3-phase bit engine per bit: (a) drive scl low, (b) set sda during the low phase, (c) drive scl high. If sda and the scl falling edge change in the same clock, a slave watching sda can falsely detect a START (sda falls while scl still reads high) and lose byte alignment.
2. **Trigger ops on rising edges, not levels.** If a controller FSM holds `op_write=1` until the op finishes, the master returns to idle and re-issues the byte (two bytes per op). Detect `op && !op_d` in the master's idle state.
3. **Use a proper handshake.** Issue an op, then wait for `busy` to rise (op started), then wait for the `busy` falling edge (op done) before advancing. Waiting only for "not busy" races and double-issues the next op.
4. **Read bursts: the slave must stop driving after NACK.** After the master NACKs the last read byte, the slave must release sda; otherwise it contends with the master's STOP and sda goes X, so the STOP is never seen. The slave should capture the master's ACK/NACK at each byte's 9th scl edge and gate its data driving on "not NACK".
5. **9 edges per byte.** Each byte = 8 data bits + 1 ACK bit; count scl edges 0..8 and process at the 9th.
6. **Verify with a simulated slave**, preloading registers, and watch bus state (sda contention shows as X in iverilog).
