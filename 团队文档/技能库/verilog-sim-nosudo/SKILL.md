---
name: verilog-sim-nosudo
description: Install Icarus Verilog (iverilog/vvp) and GTKWave for Verilog RTL simulation on a Linux host without sudo or apt, using micromamba + conda-forge prebuilt binaries. Use when building from source is impractical (missing autoconf/flex/bison) and system packages are unavailable.
metadata:
  short-description: Install Verilog simulation tools without sudo via micromamba
---

# Verilog simulation tools without sudo

Source-building iverilog needs autoconf/automake/flex/bison, which are often missing on minimal hosts. `micromamba` (a static binary) installs prebuilt `iverilog` + `gtkwave` from conda-forge into a project-local prefix.

## Workflow
1. Download micromamba from GitHub releases (GitHub is usually reachable when other mirrors are slow):
   `curl -fsSL -o <tools>/micromamba https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64`
2. `chmod +x <tools>/micromamba`
3. **Gotcha:** micromamba writes its package cache under `$HOME/.cache/conda`, which can be read-only in sandboxes. Export `HOME` to a writable dir first (e.g. `<tools>/.mamba-home`), and set `MAMBA_ROOT_PREFIX=<tools>/.mamba`.
4. Create the env:
   `micromamba create -y -p <tools>/eda-env -c conda-forge iverilog gtkwave`
5. Verify and use:
   - `iverilog -o sim design.v tb.v` then `vvp sim`
   - Dump waves with `$dumpfile("x.vcd"); $dumpvars(0, tb);` and view with `gtkwave x.vcd`.
6. Export PATH to `<tools>/eda-env/bin` (a `setup_env.sh` the team can `source` is convenient).

See [scripts/install_iverilog_micromamba.sh](scripts/install_iverilog_micromamba.sh) for the full steps.
