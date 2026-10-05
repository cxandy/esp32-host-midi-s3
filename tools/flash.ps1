<#
.SYNOPSIS
  Flash an ESP32_Host_MIDI CI artifact onto the AciduinoV2Box ESP32-S3.

.DESCRIPTION
  The CI build runs on GitHub (the 3.3.12 toolchain is ~1.85 GB and would not
  fit on the local C: drive), but flashing happens locally through the native
  USB port with the esptool that ships inside the already-installed
  arduino-esp32 2.0.17 core -- so nothing extra has to be downloaded.

  Usage:
    # 1. build (produces the artifact)
    gh workflow run build.yml --repo cxandy/esp32-host-midi-s3
    gh run download <run-id> --repo cxandy/esp32-host-midi-s3 -n sketch-build-ProbeB -D .

    # 2. flash
    .\tools\flash.ps1 -Port COM8 -ArtifactDir .\artifacts\ProbeB

.PARAMETER Port
  Serial port. The board enumerates as VID_303A&PID_1001 (Espressif native USB).

.PARAMETER ArtifactDir
  Directory holding the unzipped artifact: <Sketch>.bin plus bootloader.bin,
  partitions.bin and boot_app0.bin from the CI build path.

.PARAMETER Baud
  Flash speed. Drop to 115200 if you get "Failed to connect" on a long cable.
#>
[CmdletBinding()]
param(
  [string]$Port = 'COM8',
  [Parameter(Mandatory)][string]$ArtifactDir,
  [int]$Baud = 921600
)

$ErrorActionPreference = 'Stop'

# --- locate esptool from the locally installed arduino-esp32 core --------------
$esptoolRoot = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\esp32\tools\esptool_py'
if (-not (Test-Path $esptoolRoot)) {
  throw "esptool not found under $esptoolRoot -- install arduino-esp32 via arduino-cli first."
}
$esptool = Get-ChildItem $esptoolRoot -Recurse -Filter 'esptool.exe' |
  Sort-Object { [version]($_.Directory.Name) } -Descending |
  Select-Object -First 1 -ExpandProperty FullName
if (-not $esptool) { throw "no esptool.exe found under $esptoolRoot" }
Write-Host "esptool: $esptool"

# --- resolve artifacts -------------------------------------------------------
$ArtifactDir = (Resolve-Path $ArtifactDir).Path
$app = Get-ChildItem $ArtifactDir -Recurse -Filter '*.bin' |
  Where-Object { $_.Name -notmatch '^(bootloader|partitions|boot_app0)\.bin$' } |
  Sort-Object Length -Descending | Select-Object -First 1
if (-not $app) { throw "no application .bin under $ArtifactDir" }

$need = 'bootloader.bin', 'partitions.bin', 'boot_app0.bin'
$files = @{}
foreach ($n in $need) {
  $f = Get-ChildItem $ArtifactDir -Recurse -Filter $n | Select-Object -First 1
  if (-not $f) { throw "missing $n -- the artifact must include the full CI build path, not just the exported sketch binaries." }
  $files[$n] = $f.FullName
}

Write-Host ""
Write-Host "app     : $($app.FullName)  ($([math]::Round($app.Length/1KB)) KB)"
foreach ($n in $need) { Write-Host ("{0,-9}: {1}" -f $n, $files[$n]) }
Write-Host "port    : $Port  @ $Baud"

# --- flash -------------------------------------------------------------------
# Offsets are the standard arduino-esp32 layout for a 16 MB board.
$imageArgs = @(
  '0x0',    $files['bootloader.bin'],
  '0x8000', $files['partitions.bin'],
  '0xe000', $files['boot_app0.bin'],
  '0x10000',$app.FullName
)

Write-Host ""
& $esptool --chip esp32s3 --port $Port --baud $Baud `
  --before default_reset --after hard_reset `
  write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB @imageArgs

if ($LASTEXITCODE -ne 0) { throw "esptool failed with exit code $LASTEXITCODE" }
Write-Host ""
Write-Host "Flash OK. Open COM8 at 115200 baud for the [PROBE-B] banner."
Write-Host "If the port never appeared: hold BOOT, tap RESET, release BOOT."