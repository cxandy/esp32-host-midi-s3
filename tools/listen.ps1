param(
  [string]$Port = 'COM8',
  [int]$Baud = 115200,
  [int]$Seconds = 240
)

# Capture window for a human-triggered boot on the AciduinoV2Box.
#
# Why a human has to do this: on this board IO0 *is* the SHIFT button
# (硬件接线规划.md: "0 已被 SHIFT 使用"; 使用说明.md: "SHIFT=IO0"). IO0 low =
# ROM downloader, IO0 high = boot the sketch. Every reset the host can cause
# over native USB arrives as rst:0x15 USB_UART_CHIP_RESET, and the S3 ROM
# forces the downloader on that path no matter what DTR says -- verified by
# reading GPIO_IN (0x60004004) and by esptool's own USBJTAGSerialReset source.
# So the only reset that can boot the sketch is the EN pin.
#
# DTR/RTS are left deasserted so the port is electrically neutral and the
# board's own IO0 pull-up decides the boot mode. ProbeC prints every 500 ms, so
# the RESET press does not have to be timed against the first line of output.

$ErrorActionPreference = 'Stop'
$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200
$sp.Open()
$sp.DtrEnable = $false
$sp.RtsEnable = $false
Write-Host "listening on $Port for $Seconds s -- RELEASE the SHIFT button, then press RESET"

$sb = New-Object Text.StringBuilder
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
  try {
    $n = $sp.BytesToRead
    if ($n -gt 0) {
      $b = New-Object byte[] $n
      $r = $sp.Read($b, 0, $n)
      if ($r -gt 0) {
        [void]$sb.Append([Text.Encoding]::UTF8.GetString($b, 0, $r))
        Write-Host ("[{0,4:N0}s] +{1} bytes (total {2})" -f $sw.Elapsed.TotalSeconds, $r, $sb.Length)
      }
    } else { Start-Sleep -Milliseconds 50 }
  } catch { }
}
$sp.Close()

$t = $sb.ToString()
Write-Host ""
Write-Host "=== captured $($t.Length) bytes ==="
Write-Host $t
