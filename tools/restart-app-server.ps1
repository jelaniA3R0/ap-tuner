# Restarts the hidden AP Tuner server of the installed copy (the app window reconnects by itself).
$dir = Join-Path $env:LOCALAPPDATA 'Programs\AP Tuner'
Get-CimInstance Win32_Process | Where-Object { $_.Name -eq 'node.exe' -and $_.CommandLine -like '*server.js*' -and $_.CommandLine -like '*--app*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force; Write-Output ('stopped server ' + $_.ProcessId) }
Start-Sleep -Seconds 1
Start-Process -FilePath (Join-Path $dir 'node\node.exe') -ArgumentList ('"' + (Join-Path $dir 'src\server.js') + '" --app') -WorkingDirectory $dir -WindowStyle Hidden
Start-Sleep -Seconds 3
try { $r = Invoke-WebRequest -UseBasicParsing -Uri 'http://127.0.0.1:8090/api/presets' -TimeoutSec 5; Write-Output ('server up: ' + $r.StatusCode) } catch { Write-Output ('server not answering: ' + $_.Exception.Message) }
