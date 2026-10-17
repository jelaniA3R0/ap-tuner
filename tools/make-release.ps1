# Builds the C++ edition and packages it: D:\AP Tuner C++.zip (programs, web UI, docs, installer).
# Usage: powershell -ExecutionPolicy Bypass -File tools\make-release.ps1 [-SkipBuild]
param([switch]$SkipBuild)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (-not $SkipBuild) {
  & cmd /c "`"$root\cpp\build.cmd`""
  if ($LASTEXITCODE -ne 0) { throw 'build failed' }
}
$bin = Join-Path $root 'cpp\build\release'
$stage = Join-Path $env:TEMP 'aptuner-cpp-stage'
$dst = Join-Path $stage 'AP Tuner C++'
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force $dst | Out-Null
Copy-Item (Join-Path $bin 'app\aptuner.exe') $dst
Copy-Item (Join-Path $bin 'cli\aptuner-analyze.exe') $dst
Copy-Item (Join-Path $bin 'cli\aptuner-link.exe') $dst
robocopy (Join-Path $root 'web') (Join-Path $dst 'web') /E /NFL /NDL /NJH /NJS /NP | Out-Null
New-Item -ItemType Directory -Force (Join-Path $dst 'docs') | Out-Null
robocopy (Join-Path $root 'docs') (Join-Path $dst 'docs') /E /NFL /NDL /NJH /NJS /NP | Out-Null
Copy-Item (Join-Path $root 'cpp\packaging\Install AP Tuner.cmd') $dst
Copy-Item (Join-Path $root 'cpp\packaging\Uninstall AP Tuner.cmd') $dst
Copy-Item (Join-Path $root 'cpp\packaging\README.txt') $dst
Copy-Item (Join-Path $root 'LICENSE') $dst
Copy-Item (Join-Path $root 'AP Tuner.ico') $dst
$zip = Join-Path (Split-Path -Parent $root) 'AP Tuner C++.zip'
if (Test-Path $zip) { Remove-Item -Force $zip }
Compress-Archive -Path $dst -DestinationPath $zip -CompressionLevel Optimal
Remove-Item -Recurse -Force $stage
$f = Get-Item $zip
Write-Output ("{0}  {1:N1} MB" -f $f.FullName, ($f.Length / 1MB))
