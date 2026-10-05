param(
  [string]$Port = 'COM8',
  [int]$Baud = 115200,
  [int]$Seconds = 240
)

# Poll the port instead of holding it open.
#
# A continuously held port keeps the USB CDC link asserted for the whole window,
# and the S3 ROM then treats any reset that arrives as USB-driven and drops into
# the downloader. This leaves the port fully closed for half of every cycle, so
# a physical RESET press can land in a clean window where nothing is driving
# DTR/RTS and the board's own IO0 (SHIFT) pull-up decides the boot mode.
#
# ProbeC prints every 500 ms, so it does not matter whether the first line of
# boot output is caught -- just needs one poll cycle after the sketch starts.

$ErrorActionPreference = 'Continue'
$sb = New-Object Text.StringBuilder
$line = New-Object Text.StringBuilder
$sw = [Diagnostics.Stopwatch]::StartNew()
$cycles = 0

Write-Host "polling $Port for $Seconds s -- RELEASE SHIFT (IO0), then press RESET"

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
  $cycles++
  $sp = $null
  try {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
    $sp.ReadTimeout = 200
    $sp.DtrEnable = $false
    $sp.RtsEnable = $false
    $sp.Open()
    $until = (Get-Date).AddMilliseconds(200)
    while ((Get-Date) -lt $until) {
      try {
        $n = $sp.BytesToRead
        if ($n -gt 0) {
          $b = New-Object byte[] $n
          $r = $sp.Read($b, 0, $n)
          if ($r -gt 0) {
            $chunk = [Text.Encoding]::UTF8.GetString($b, 0, $r)
            [void]$sb.Append($chunk)
            [void]$line.Append($chunk)
            # Emit whole lines as they complete so progress is visible live
            # rather than only in the final dump.
            $txt = $line.ToString()
            while ($txt.Contains("`n")) {
              $i = $txt.IndexOf("`n")
              $one = $txt.Substring(0, $i).TrimEnd("`r")
              $txt = $txt.Substring($i + 1)
              if ($one.Length -gt 0) { Write-Host ("  | {0}" -f $one) }
            }
            [void]$line.Clear()
            [void]$line.Append($txt)
          }
        }
      } catch { }
    }
  } catch {
    # port busy or briefly absent during re-enumeration; ignore and retry
  } finally {
    if ($sp) { try { $sp.Close() } catch { } }
  }
  Start-Sleep -Milliseconds 200
}

$t = $sb.ToString()
Write-Host ""
Write-Host "=== polled $cycles cycles, captured $($t.Length) bytes ==="
Write-Host $t
