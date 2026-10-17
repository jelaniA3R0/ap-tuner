# Stops a running demonstration: the demo-run script, scripted pilots, the oscillation experiment and
# the simulator processes. The AP Tuner app itself keeps running.
$procs = Get-CimInstance Win32_Process | Where-Object {
  ($_.Name -eq 'node.exe' -and (($_.CommandLine -like '*demo-run.js*') -or ($_.CommandLine -like '*pilot.js*') -or (  ($_.Name -eq 'node.exe' -and (($_.CommandLine -like '*demo-run.js*') -or ($_.CommandLine -like '*pilot.js*') -or ($_.CommandLine -like '*osc-experiment.js*'))) -or $_.Name -eq 'ArduCopter.exe' -or $_.Name -eq 'ArduPlane.exe'.CommandLine -like '*osc-experiment.js*') -or (  ($_.Name -eq 'node.exe' -and (($_.CommandLine -like '*demo-run.js*') -or ($_.CommandLine -like '*pilot.js*') -or ($_.CommandLine -like '*osc-experiment.js*'))) -or $_.Name -eq 'ArduCopter.exe' -or $_.Name -eq 'ArduPlane.exe'.CommandLine -like '*flight-demo.js*'))) -or $_.Name -eq 'ArduCopter.exe' -or $_.Name -eq 'ArduPlane.exe'
}
foreach ($p in $procs) { try { Stop-Process -Id $p.ProcessId -Force; Write-Output ('stopped ' + $p.Name + ' ' + $p.ProcessId) } catch {} }
if (-not $procs) { Write-Output 'nothing to stop' }
