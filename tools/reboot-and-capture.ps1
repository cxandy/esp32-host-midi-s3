param(
  [string]$Port = 'COM8',
  [int]$Baud = 115200
)

# Reset the board with DTR asserted (GPIO0 high on the S3 USB-Serial-JTAG,
# so the reset boots from flash rather than the ROM downloader), then close
# the port, let TinyUSB re-enumerate, reopen it and capture.
#
# The port handle goes stale when the device re-enumerates after the reset,
# so keeping it open across the reset misses the output -- it has to be
# closed and reopened.

$ErrorActionPreference = 'Stop'

function Capture([int]$Seconds) {
  $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
  $sp.ReadTimeout = 200
  $sp.DtrEnable = $true
  $sp.RtsEnable = $false
  $sp.Open()
  $sb = New-Object Text.StringBuilder
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
      $n = $sp.BytesToRead
      if ($n -gt 0) {
        $b = New-Object byte[] $n
        $r = $sp.Read($b, 0, $n)
        if ($r -gt 0) { [void]$sb.Append([Text.Encoding]::UTF8.GetString($b, 0, $r)) }
      }
    } catch { }
    Start-Sleep -Milliseconds 25
  }
  $sp.Close()
  return $sb.ToString()
}

Write-Host "resetting with DTR asserted (GPIO0 high -> normal boot)..."
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.DtrEnable = $true
$sp.RtsEnable = $false
$sp.Open()
Start-Sleep -Milliseconds 300
$sp.RtsEnable = $true
Start-Sleep -Milliseconds 150
$sp.RtsEnable = $false
$sp.Close()

Write-Host "waiting for TinyUSB to re-enumerate..."
Start-Sleep -Seconds 3

Write-Host "capturing on a fresh handle..."
$out = Capture 15
Write-Host ""
Write-Host "--- $($out.Length) bytes ---"
Write-Host $out
if ($out -match 'ALIVE') { Write-Host "`n>>> HEARTBEAT SEEN -- application is running" }
else { Write-Host "`n>>> no heartbeat" }
