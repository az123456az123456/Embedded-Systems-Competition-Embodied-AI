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
Fix (verified on Ubuntu 26.04): the IDE bundles old libstdc++/freetype and its own Qt 5.15.14; the system Qt 5.15.18 gets pulled in for plugins, causing "Cannot mix incompatible Qt library" and GLIBCXX_3.4.29 errors. The working launch combo is:
```bash
export LD_LIBRARY_PATH="$GOWIN/IDE/lib"          # use bundled Qt consistently
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libfreetype.so.6  # satisfy system fontconfig
export QT_QPA_PLATFORM=xcb                        # Wayland session: use XWayland, not the wayland plugin
"$GOWIN/IDE/bin/gw_ide"
```
`license_config_gui` needs the same env. Wrap this in a launcher script so teammates don't have to remember it.

## License (free, node-locked)
1. Apply at `https://www.gowinsemi.com.cn/profile/license` after login.
2. Get the machine code from `license_config_gui` (run with the launch env above) or the IDE's license dialog.
3. Submit machine code + email on the website; Gowin provides a `license.dat`.
4. Import the `.dat` via `license_config_gui`, or place it where the IDE expects (the tool shows the path). The default `IDE/bin/gwlicense.ini` points at a license server (`lic=...`); a local node-locked license overrides it.
