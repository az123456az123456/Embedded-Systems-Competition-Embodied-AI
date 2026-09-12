---
name: fpga-motor-pid
description: Design a motor speed PID in Verilog (encoder quadrature decode + velocity measurement + PWM) and avoid the integration bugs that break closed-loop behavior, especially integral windup defeating emergency stop. Use when writing or debugging FPGA motor control loops.
metadata:
  short-description: Verilog motor PID + integration pitfalls
---

# Motor PID in Verilog — pitfalls

Verified lessons from an encoder + velocity + PID + PWM chain (iverilog).

1. **Emergency stop must disable the PID (clear the integrator).** If the obstacle/safety logic only forces `target=0` but leaves the PID enabled, the accumulated integral keeps the duty at full scale and the motor never stops. Gate the PID `enable` on "not emergency-stop", so the integrator resets. This bug is easy to miss in tests — assert that PWM actually drops to 0 on a forced stop, not just that `target==0`.
2. **Tune gains against the plant model.** In simulation, if motor gain × KP ≥ 2 the loop oscillates/diverges. Start with P only, add I for steady-state error, then D for overshoot. Document the tuning.
3. **Encoder decode is 4x**: each 00→01→11→10→00 cycle = 4 counts. A test that walks only 3 states per "revolution" will read 3 instead of 4.
4. **Velocity measurement** uses a fixed time window; the window length trades update rate vs resolution. Speed units are "counts per window", so tests must compute expected values from window and pulse period.
5. **Declaration order matters** in strict Verilog: declare signals (e.g. a modeled `speed`) before instantiating modules that consume them, or elaboration fails.
6. Prefer plain Verilog-2001 in testbenches (no array task args / SV constructs) when targeting iverilog defaults.
