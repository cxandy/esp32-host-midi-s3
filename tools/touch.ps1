<#
.SYNOPSIS
  Reset the board into the running sketch with the USB 1200 bps touch.

.DESCRIPTION
  The ESP32-S3 ROM always enters its USB downloader when the chip is reset
  while the USB link is up, so esptool's reset leaves it at "waiting for
  download" and the sketch never runs. There is no way around that with a
  physical cable pull on a bench.

  TinyUSB CDC supports the 1200 bps touch: opening the port at 1200 baud and
  toggling DTR makes the firmware's CDC task reboot the chip, this time
  normally, so the app runs.

  This is also why the CI FQBN uses USBMode=default instead of USBMode=hwcdc.

.PARAMETER Port
  Serial port. After the touch the device re-enumerates; give it a moment.
#>
param(
  [string]$Port = 'COM8',
  [int]$WaitSeconds = 10
)

$ErrorActionPreference = 'Stop'

$before = (Get-CimInstance Win32_PnPEntity |
  Where-Object { $_.Name -match 'COM\d+' } |
  Select-Object -ExpandProperty Name)

Write-Host "ports before: $($before -join ', ')"

# The touch: open at 1200 baud with DTR asserted, then close. Timing and the
# DTR edge are what the CDC task in the firmware looks for.
$sp = New-Object System.IO.Ports.SerialPort $Port, 1200, 'None', 8, 'One'
try {
  $sp.Open()
  $sp.DtrEnable = $true
  $sp.RtsEnable = $false
  Start-Sleep -Milliseconds 200
  $sp.DtrEnable = $false
  Start-Sleep -Milliseconds 100
} finally {
  $sp.Close()
}
Write-Host "1200 bps touch sent on $Port"

# Wait for the CDC device to drop and come back.
$deadline = (Get-Date).AddSeconds($WaitSeconds)
$gone = $false
while ((Get-Date) -lt $deadline) {
  Start-Sleep -Milliseconds 500
  $now = (Get-CimInstance Win32_PnPEntity |
    Where-Object { $_.Name -match 'COM\d+' } |
    Select-Object -ExpandProperty Name)
  if ($gone -or ($now -notcontains $Port)) { $gone = $true }
  if ($gone -and $now -contains $Port) {
    Write-Host "device re-enumerated: $($now -join ', ')"
    Start-Sleep -Seconds 2
    exit 0
  }
}

Write-Host "no re-enumeration seen within $WaitSeconds s; ports now: $(($now) -join ', ')"
Write-Host "If the sketch never prints, hold BOOT, tap RESET, release BOOT once."
exit 1
