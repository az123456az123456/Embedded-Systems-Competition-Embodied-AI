---
name: gowin-eda-setup
description: Download and install Gowin EDA (云源软件, GowinDesigner) for Gowin FPGA development, including the account/license step that cannot be automated. Use when a team needs the Gowin toolchain (synthesis/place-and-route/bitstream) for GW2A/GW5A/GW5AT-series chips.
metadata:
  short-description: Guide for downloading/installing Gowin EDA and getting a license
---

# Gowin EDA setup

Gowin EDA (云源软件) is the official IDE for Gowin FPGAs. Downloading requires a registered account (with slider captcha), so the account/license step must be done by the user — it cannot be automated.

## Steps
1. User registers/logs in at `https://www.gowinsemi.com.cn/software_share` (school email recommended).
2. Download **Gowin EDA For Linux** (choose a version covering the target family, e.g. GW5AT). The site is JS-rendered; the actual file link appears after login.
3. Also request the free license at the profile/license page.
4. Install into a project-local folder, then run the GUI (needs a display).
5. For Verilog RTL **simulation** during early development, Gowin EDA is not required — iverilog + GTKWave are sufficient (see the `verilog-sim-nosudo` skill). Gowin EDA is mainly needed for synthesis, place-and-route, and bitstream generation when the board arrives.

## Notes
- Keep the download and license steps with the user; an agent can install/extract and verify the tool once the installer file is provided.
- Learning resources: Gowin official docs center and Bilibili official courses.

## Known issue: fontconfig/freetype symbol error on new Ubuntu
Symptom: `gw_ide` fails with `symbol lookup error: /usr/lib/x86_64-linux-gnu/libfontconfig.so.1: undefined symbol: FT_Done_MM_Var`.
Cause: the IDE bundles an old `libfreetype.so.6` in `IDE/lib`; the system `libfontconfig` needs a newer freetype symbol.
Fix: preload the system freetype before launching:
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libfreetype.so.6
"$GOWIN/IDE/bin/gw_ide"
```
Wrap this in a launcher script so teammates don't have to remember it.
