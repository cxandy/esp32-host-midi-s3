# Fetch a GitHub Actions artifact and verify it.
#
# `gh run download` hangs on this link and a single-stream curl crawls at
# ~35 KB/s, so the 37 MB artifacts the workflow used to publish were a
# 15-minute wait per flashing iteration. The workflow now uploads only the
# four flash images (~1 MB), which one stream fetches in seconds; the ranged
# multi-stream path stays here for when a build really is large.
#
# Why the care about verification: an artifact is served as one zip blob, and a
# ranged fetch has come back the right total length with the wrong bytes at
# offset 0. A wrong zip is worse than a slow one, because it extracts, flashes
# and reboots happily. So -Sha256 (the digest GitHub publishes for every
# artifact) is checked before the file is handed on.
param(
  [Parameter(Mandatory = $true)][string]$Url,    # artifact API url (302 source)
  [Parameter(Mandatory = $true)][long]$Size,     # expected total size
  [Parameter(Mandatory = $true)][string]$Out,    # final zip path
  [string]$Sha256,                              # digest from the artifact API
  [int]$Chunks = 1
)

$ErrorActionPreference = 'Stop'
$dir = Join-Path $env:TEMP ("chunks-" + [IO.Path]::GetFileNameWithoutExtension($Out))
# An existing chunk directory makes an interrupted fetch resumable: only ranges
# whose size is exactly right are kept, so a truncated one is refetched rather
# than trusted.
New-Item -ItemType Directory -Path $dir -Force | Out-Null
$tok = ((gh auth token) | Select-Object -First 1).Trim()

function Get-PartLength([string]$p) {
  if (Test-Path $p) { return (Get-Item $p).Length }
  return 0
}

if ($Chunks -le 1) {
  # The known-good path: straight off the API url with redirects followed.
  $part = Join-Path $dir 'c00.part'
  if ((Get-PartLength $part) -ne $Size) {
    "fetching $Size bytes in one stream"
    curl.exe -sS -L --fail --retry 3 --max-time 900 `
      -H "Authorization: Bearer $tok" -o $part $Url
    "curl exit=$LASTEXITCODE"
  } else {
    "file already on disk"
  }
} else {
  # Ranged: the API 302s to a signed Azure blob that honours Range (verified --
  # a 0-1023 request comes back as exactly 1024 bytes), so N chunks fetched at
  # once give ~3x the throughput of one stream.
  $final = curl.exe -sSIL -o NUL -w "%{url_effective}" -H "Authorization: Bearer $tok" $Url 2>$null
  if (-not $final) { throw "could not resolve artifact url" }
  $per = [long][Math]::Ceiling($Size / $Chunks)
  $ca = @("--parallel", "--parallel-max", "$Chunks", "--fail", "--silent", "--show-error", "--retry", "3")
  for ($i = 0; $i -lt $Chunks; $i++) {
    $start = [long]$i * $per
    if ($start -ge $Size) { break }
    $end = [long][Math]::Min($start + $per - 1, $Size - 1)
    $part = Join-Path $dir ("c{0:D2}.part" -f $i)
    if ((Get-PartLength $part) -eq ($end - $start + 1)) { continue }
    $ca += @("--range", "$start-$end", "--url", $final, "-o", $part)
  }
  if ($ca.Count -gt 5) {
    "fetching $Size bytes in up to $Chunks parallel ranges"
    curl.exe @ca
    "curl exit=$LASTEXITCODE"
  } else {
    "every range already on disk"
  }

  # Per-chunk repair: a transfer that comes back short shifts every following
  # byte, so the archive breaks somewhere far from the actual failure.
  for ($i = 0; $i -lt $Chunks; $i++) {
    $start = [long]$i * $per
    if ($start -ge $Size) { break }
    $end = [long][Math]::Min($start + $per - 1, $Size - 1)
    $expected = [long]($end - $start + 1)
    $part = Join-Path $dir ("c{0:D2}.part" -f $i)
    for ($t = 1; $t -le 5; $t++) {
      $len = Get-PartLength $part
      if ($len -eq $expected) { break }
      curl.exe -sS --fail -L --max-time 300 --retry 3 -r "$start-$end" -o $part $final
      if ($t -eq 5) { throw "chunk $i stuck at $len bytes, expected $expected" }
    }
  }
}

$parts = Get-ChildItem $dir -Filter *.part | Sort-Object Name
$sum = 0
$fs = [IO.File]::Open($Out, [IO.FileMode]::Create)
try {
  foreach ($p in $parts) {
    $b = [IO.File]::ReadAllBytes($p.FullName)
    # A chunk the size of the whole file means the range was ignored and the
    # server returned everything; concatenating it would duplicate data.
    if ($Chunks -gt 1 -and $b.Length -ge $Size) {
      throw "chunk $($p.Name) is $($b.Length) bytes -- range ignored"
    }
    $fs.Write($b, 0, $b.Length)
    $sum += $b.Length
  }
} finally { $fs.Close() }
if ($sum -ne $Size) { Remove-Item $Out -Force; throw "got $sum bytes, expected $Size" }

if ($Sha256) {
  $got = (Get-FileHash $Out -Algorithm SHA256).Hash.ToLower()
  if ($got -ne $Sha256.ToLower()) {
    Remove-Item $Out -Force
    Remove-Item $dir -Recurse -Force
    throw "sha256 $got != expected $Sha256"
  }
  "sha256 verified"
} else {
  "WARNING: no -Sha256 given; archive not verified"
}

Remove-Item $dir -Recurse -Force
"complete $sum -> $Out"