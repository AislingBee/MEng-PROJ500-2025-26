Repository build helper for STM32 firmware

This directory contains helper scripts to prepare a portable build environment (no admin
required) and to build `firmware.bin` for the Nucleo-F429ZI.

Files added
- `setup_portable_toolchain.ps1`: Downloads and extracts a portable GNU Arm Embedded toolchain
  and stlink (if available) into `./tools`. Optionally downloads STM32CubeF4 into this folder.
- `build_firmware.ps1`: Uses the portable toolchain to run `make` and build `firmware.bin`.

Quick workflow for the BUILD PC (the machine that will compile and flash):
1. Copy this repo to the BUILD PC (or clone it).
2. Open PowerShell and run:
   pwsh -ExecutionPolicy Bypass -File .\setup_portable_toolchain.ps1 -DownloadCubeF4
   # this will try to download an arm toolchain and stlink and place them under ./tools

3. Run the build script (optionally specify the STM32CubeF4 directory if you downloaded it elsewhere):
   pwsh -ExecutionPolicy Bypass -File .\build_firmware.ps1 -STM32CubeDir .\STM32CubeF4-master -Board F429

4. After a successful build, `firmware.bin` will be created in this directory. Use your preferred
   flasher (st-flash or STM32CubeProgrammer) on the BUILD PC to flash the Nucleo.

Notes and caveats
- This approach avoids requiring admin/install on the BUILD PC; it downloads portable zips and
  extracts them inside the repo. If you want the repo to contain the actual toolchain binaries,
  commit the `tools` folder, but beware of very large files.
- The Makefile in this repo is configured for `STM32F429xx` by default (matching Nucleo-F429ZI).
  If you need to target a different MCU (e.g., F446RE), edit the Makefile accordingly before building.
- On Windows you will need `make` (MSYS2/Git-Bash or mingw32-make). The `build_firmware.ps1` script
  tries `mingw32-make` if `make` is not present.

If you want, I can:
- Add an optional `st-flash` wrapper script that uses the portable `stlink` binaries to flash
  `firmware.bin` automatically.
- Modify the Makefile so you can pass `BOARD=F446`/`BOARD=F429` at build time and it picks the
  correct startup/ld files.
