param(
  [string]$Port = 'COM8',
  [int]$Baud = 115200,
  [int]$Seconds = 12
)

# Pure listener. Deliberately never touches DTR/RTS: on the ESP32-S3 native USB
# port those lines drive IO0/EN, and pulsing them drops the chip into the ROM
# download mode ("waiting for download") instead of running the sketch. The
# sketch prints a heartbeat every 2 s, so attaching late is not a problem.

$ErrorActionPreference = 'Stop'
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.Open()
Write-Host "listening on $Port @ $Baud for $Seconds s (DTR/RTS left alone)"

$sb = New-Object Text.StringBuilder
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
  try {
    $n = $sp.BytesToRead
    if ($n -gt 0) {
      $buf = New-Object byte[] $n
      $r = $sp.Read($buf, 0, $n)
      if ($r -gt 0) { [void]$sb.Append([Text.Encoding]::UTF8.GetString($buf, 0, $r)) }
    } else { Start-Sleep -Milliseconds 30 }
  } catch { }
}
$sp.Close()

$out = $sb.ToString()
Write-Host ""
Write-Host "--- $($out.Length) bytes ---"
Write-Host $out
if ($out -notmatch 'ALIVE') { Write-Host ">>> no heartbeat seen" }