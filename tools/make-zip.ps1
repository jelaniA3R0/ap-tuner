# Builds the portable package D:\AP Tuner.zip from a staging copy (excludes sessions, simulator run folders,
# demo results and logs). Usage: powershell -ExecutionPolicy Bypass -File tools\make-zip.ps1
$root = Split-Path -Parent $PSScriptRoot
$stage = Join-Path $env:TEMP 'aptuner-stage'
$zip = Join-Path (Split-Path -Parent $root) 'AP Tuner.zip'
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force (Join-Path $stage 'AP Tuner') | Out-Null
robocopy $root (Join-Path $stage 'AP Tuner') /E /NFL /NDL /NJH /NJS /NP /XD sessions run run-* demo _old /XF *.log eeprom.bin | Out-Null
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path (Join-Path $stage 'AP Tuner') -DestinationPath $zip -CompressionLevel Optimal
Remove-Item -Recurse -Force $stage
$f = Get-Item $zip
Write-Output ("{0}  {1:N1} MB" -f $f.FullName, ($f.Length / 1MB))
