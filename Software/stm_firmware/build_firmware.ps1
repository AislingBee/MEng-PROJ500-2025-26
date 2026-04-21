<#
build_firmware.ps1

Builds `firmware.bin` using the portable toolchain located under `./tools/gcc` and the
`Makefile` in this directory. Designed to be run on the build PC (the PC that will flash).

Usage:
  pwsh -ExecutionPolicy Bypass -File .\build_firmware.ps1 [-STM32CubeDir <path>] [-Board F429|F446]

Notes:
 - This script assumes you have run `setup_portable_toolchain.ps1` already and that
   the gcc toolchain exists under `./tools/gcc` (the actual bin folder will be inside
   that extracted directory; the script attempts to locate `arm-none-eabi-gcc`.
 - On Windows you will need 'make' available. If you don't have make, the script will
   try to use 'mingw32-make' if present in PATH. Alternatively you can run this script
   inside an MSYS2 / Git-Bash shell where make exists.
#>

param(
    [string]$STM32CubeDir = '',
    [ValidateSet('F429','F446')]
    [string]$Board = 'F429'
)

$base = Split-Path -Parent $MyInvocation.MyCommand.Definition
Set-Location $base

# locate arm-none-eabi-gcc
$gccBin = Get-ChildItem -Path (Join-Path $base 'tools\gcc') -Recurse -Filter 'arm-none-eabi-gcc.exe' -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $gccBin) { Write-Error "arm-none-eabi-gcc not found under ./tools/gcc. Run setup_portable_toolchain.ps1 first."; exit 1 }
$gccPath = Split-Path $gccBin.FullName -Parent
$env:Path = "$gccPath;$env:Path"
Write-Host "Using gcc from $gccPath"

# export STM32CUBE_DIR if provided
if ($STM32CubeDir) {
    $env:STM32CUBE_DIR = $STM32CubeDir
}

# choose Make command
$makeCmd = 'make'
if (-not (Get-Command $makeCmd -ErrorAction SilentlyContinue)) {
    if (Get-Command mingw32-make -ErrorAction SilentlyContinue) { $makeCmd = 'mingw32-make' }
    else { Write-Warning "'make' not found in PATH. Please install make (e.g. from MSYS2) or run this script from a shell with make." }
}

# set BOARD-specific changes (Makefile in repo defaults to F429)
if ($Board -eq 'F446') {
    Write-Host "Note: If building for F446 you need to edit Makefile MCU flags and startup/ld files." -ForegroundColor Yellow
}

# run make
$rc = & $makeCmd
if ($LASTEXITCODE -ne 0) { Write-Error "make failed with exit code $LASTEXITCODE"; exit $LASTEXITCODE }

if (Test-Path '.\firmware.bin') { Write-Host "Build complete: firmware.bin created in $(Get-Location)" -ForegroundColor Green }
else { Write-Warning "Build finished but firmware.bin not found." }
