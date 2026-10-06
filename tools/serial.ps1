<#
.SYNOPSIS
  Drive the board over the serial port: send command characters, then capture.

.DESCRIPTION
  MidiMonitor listens for single characters on the USB console (see serviceSerial()
  in the sketch): 1 = hold OUT HIGH, 0 = hold OUT LOW, z = release OUT, u = give
  the pad back to the UART, n = step through the four states, a = toggle the
  hand-free HIGH/LOW/hiZ sweep. That exists so the OUT levels can be exercised
  without a hand on the board -- which matters when the thing being measured is
  a multimeter reading on a DIN jack.

  The port is opened once for the whole window, with DTR and RTS left low, so no
  reset is signalled. (poll.ps1 deliberately re-opens instead of holding, because
  there a physical RESET press has to land in a quiet window; not needed here.)

.PARAMETER Send
  Characters to write immediately after opening the port, before capturing.

.EXAMPLE
  .\tools\serial.ps1 -Send 'a' -Seconds 70 -Out "$env:TEMP\sweep.txt"
#>
[CmdletBinding()]
param(
  [string]$Port = 'COM8',
  [int]$Baud = 115200,
  [int]$Seconds = 60,
  [string]$Send = '',
  [string]$Out = '',
  [switch]$Quiet
)

$ErrorActionPreference = 'Stop'

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.Open()

try {
  if ($Send.Length -gt 0) {
    $sp.Write($Send)
    "sent '$Send' to $Port"
  }

  $sb = New-Object Text.StringBuilder
  $line = New-Object Text.StringBuilder
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
      $n = $sp.BytesToRead
      if ($n -gt 0) {
        $b = New-Object byte[] $n
        $r = $sp.Read($b, 0, $n)
        if ($r -gt 0) {
          $chunk = [Text.Encoding]::UTF8.GetString($b, 0, $r)
          [void]$sb.Append($chunk)
          [void]$line.Append($chunk)
          $txt = $line.ToString()
          while ($txt.Contains("`n")) {
            $i = $txt.IndexOf("`n")
            $one = $txt.Substring(0, $i).TrimEnd("`r")
            $txt = $txt.Substring($i + 1)
            if ($one.Length -gt 0 -and -not $Quiet) { Write-Host ("  | {0}" -f $one) }
          }
          [void]$line.Clear()
          [void]$line.Append($txt)
        }
      }
    } catch { }
  }

  $t = $sb.ToString()
} finally {
  try { $sp.Close() } catch { }
}

if ($Out) { Set-Content -Path $Out -Value $t -Encoding UTF8 }
if (-not $Quiet) {
  "=== captured $($t.Length) bytes ==="
  $t
}