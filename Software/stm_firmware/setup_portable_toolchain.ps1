<#
setup_portable_toolchain.ps1

Downloads a portable GNU Arm Embedded toolchain and stlink (st-flash) into the repository
under `Software/stm_firmware/tools` so a separate PC (without admin) can build and flash.

This script is intentionally conservative: it will try known URLs and otherwise ask the user
for direct download URLs so automated CI or an offline PC can prepare the repo for building.

Usage (on the target PC which will build/flash):
  pwsh.exe -ExecutionPolicy Bypass -File .\setup_portable_toolchain.ps1

The script will create the following layout in the current folder (assumed to be
`Software/stm_firmware`):
  - ./tools/gcc/           (portable arm-none-eabi toolchain)
  - ./tools/stlink/        (st-flash / st-link binaries)
  - ./STM32CubeF4/         (optional: STM32CubeF4 firmware package)

Notes:
 - The script does not add large binaries to git automatically (you can if you want), but
   it places downloads into the repo so you can commit them if desired.
 - If automatic downloads fail, the script will prompt for manual URLs.
#>

param(
    [string]$GccUrl = '',
    [string]$StlinkUrl = '',
    [switch]$DownloadCubeF4
)

$ErrorActionPreference = 'Stop'

$base = Split-Path -Parent $MyInvocation.MyCommand.Definition
Set-Location $base

$toolsDir = Join-Path $base 'tools'
$gccDir = Join-Path $toolsDir 'gcc'
$stlinkDir = Join-Path $toolsDir 'stlink'
$cubeDir = Join-Path $base 'STM32CubeF4'

New-Item -ItemType Directory -Path $gccDir -Force | Out-Null
New-Item -ItemType Directory -Path $stlinkDir -Force | Out-Null

function Try-Download($url, $outPath) {
    Write-Host "Downloading $url -> $outPath"
    try {
        Invoke-WebRequest -Uri $url -OutFile $outPath -UseBasicParsing -TimeoutSec 300
        return $true
    } catch {
        Write-Warning "Download failed: $($_.Exception.Message)"
        return $false
    }
}

# Candidate GCC URLs (examples) - these links may change. If they fail the script will ask.
$gccCandidates = @(
    'https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.zip',
    'https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-win32.zip'
)

# Candidate stlink zip (st-flash) - example GitHub release asset (may change)
$stlinkCandidates = @(
    'https://github.com/stlink-org/stlink/releases/download/v1.7.0/stlink_win32.zip',
    'https://github.com/stlink-org/stlink/releases/download/v1.8.0/stlink_win32.zip'
)

if (-not $GccUrl) {
    foreach ($u in $gccCandidates) {
        $tmp = Join-Path $toolsDir (Split-Path $u -Leaf)
        if (Try-Download $u $tmp) {
            Write-Host "Extracting $tmp to $gccDir"
            Expand-Archive -Path $tmp -DestinationPath $gccDir -Force
            Remove-Item $tmp -Force
            break
        }
    }
    if (-not (Get-ChildItem -Path $gccDir -Recurse -ErrorAction SilentlyContinue)) {
        $GccUrl = Read-Host "Automatic GCC download failed. Please paste a direct URL for gcc-arm-none-eabi win32 zip"
        if (-not (Try-Download $GccUrl (Join-Path $toolsDir (Split-Path $GccUrl -Leaf)))) { Write-Error "Failed to download GCC. Exiting."; exit 1 }
        Expand-Archive -Path (Join-Path $toolsDir (Split-Path $GccUrl -Leaf)) -DestinationPath $gccDir -Force
    }
}
else {
    $out = Join-Path $toolsDir (Split-Path $GccUrl -Leaf)
    if (-not (Try-Download $GccUrl $out)) { Write-Error "GCC download failed."; exit 1 }
    Expand-Archive -Path $out -DestinationPath $gccDir -Force
}

if (-not $StlinkUrl) {
    foreach ($u in $stlinkCandidates) {
        $tmp = Join-Path $toolsDir (Split-Path $u -Leaf)
        if (Try-Download $u $tmp) {
            Write-Host "Extracting $tmp to $stlinkDir"
            Expand-Archive -Path $tmp -DestinationPath $stlinkDir -Force
            Remove-Item $tmp -Force
            break
        }
    }
    if (-not (Get-ChildItem -Path $stlinkDir -Recurse -ErrorAction SilentlyContinue)) {
        $StlinkUrl = Read-Host "Automatic stlink download failed. Paste a direct URL for stlink win32 zip (or press Enter to skip)"
        if ($StlinkUrl) {
            $out = Join-Path $toolsDir (Split-Path $StlinkUrl -Leaf)
            if (-not (Try-Download $StlinkUrl $out)) { Write-Warning "stlink download failed; continuing without stlink." }
            else { Expand-Archive -Path $out -DestinationPath $stlinkDir -Force; Remove-Item $out -Force }
        }
    }
}
else {
    $out = Join-Path $toolsDir (Split-Path $StlinkUrl -Leaf)
    if (-not (Try-Download $StlinkUrl $out)) { Write-Error "stlink download failed."; exit 1 }
    Expand-Archive -Path $out -DestinationPath $stlinkDir -Force
}

if ($DownloadCubeF4.IsPresent) {
    New-Item -ItemType Directory -Path $cubeDir -Force | Out-Null
    $cubeZip = Join-Path $base 'STM32CubeF4-master.zip'
    $cubeUrl = 'https://github.com/STMicroelectronics/STM32CubeF4/archive/refs/heads/master.zip'
    if (Try-Download $cubeUrl $cubeZip) {
        Expand-Archive -Path $cubeZip -DestinationPath $base -Force
        Remove-Item $cubeZip -Force
        Write-Host "STM32CubeF4 downloaded into $base\STM32CubeF4-master"
    } else {
        Write-Warning "Failed to download STM32CubeF4. You can download it manually and place it at $cubeDir"
    }
}

Write-Host "
Portable toolchain setup complete.
 - gcc:  $gccDir (look for bin\arm-none-eabi-gcc.exe)
 - stlink: $stlinkDir
 - If you downloaded STM32CubeF4, it will be under $base\STM32CubeF4-master

Next steps:
 - On the build PC run: `pwsh -ExecutionPolicy Bypass -File .\setup_portable_toolchain.ps1`
 - Then run `build_firmware.ps1` (created in this repo) to build using the portable toolchain.
" -ForegroundColor Green
