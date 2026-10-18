# Builds the portable package D:\AP Tuner.zip (the JavaScript app) from a staging copy. Excludes sessions,
# simulator run folders, demo results, logs, the git history, the C++ edition (it has its own release zip,
# tools\make-release.ps1) and editor folders. Usage: powershell -ExecutionPolicy Bypass -File tools\make-zip.ps1
$root = Split-Path -Parent $PSScriptRoot
# stage next to the zip, not in %TEMP%: drive C: may be nearly full
$stage = Join-Path (Split-Path -Parent $root) 'aptuner-stage'
$zip = Join-Path (Split-Path -Parent $root) 'AP Tuner.zip'
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force (Join-Path $stage 'AP Tuner') | Out-Null
robocopy $root (Join-Path $stage 'AP Tuner') /E /NFL /NDL /NJH /NJS /NP /XD sessions run run-* demo _old .git .claude .vscode cpp build node_modules /XF *.log eeprom.bin | Out-Null
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path (Join-Path $stage 'AP Tuner') -DestinationPath $zip -CompressionLevel Optimal
Remove-Item -Recurse -Force $stage
$f = Get-Item $zip
Write-Output ("{0}  {1:N1} MB" -f $f.FullName, ($f.Length / 1MB))
