param(
  [string]$Port = 'COM8',
  [int]$Baud = 115200,
  [int]$Seconds = 12
)

# On the ESP32-S3 USB-Serial-JTAG, DTR drives GPIO0 and RTS drives EN.
# Asserting DTR holds GPIO0 high, so an RTS reset boots from flash
# rather than dropping into the ROM downloader. Hold the port open the
# whole time so the boot log is captured.

$ErrorActionPreference = 'Stop'
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200
$sp.Open()
$sp.DtrEnable = $true
$sp.RtsEnable = $false
Start-Sleep -Milliseconds 400

$sb = New-Object Text.StringBuilder
Write-Host "DTR asserted (GPIO0 high), pulsing RTS to reset..."
$sp.RtsEnable = $true
Start-Sleep -Milliseconds 150
$sp.RtsEnable = $false

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

$t = $sb.ToString()
Write-Host ""
Write-Host "--- $($t.Length) bytes ---"
Write-Host $t
