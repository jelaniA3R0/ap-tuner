@echo off
rem Starts an ArduCopter SITL (software in the loop) for testing AP Tuner without a real vehicle.
rem Then run "fly test quad.cmd" to arm, take off and fly stick taps, and connect AP Tuner to TCP 127.0.0.1:5762.
cd /d "%~dp0"
if not exist ArduCopter.exe (
  echo ArduCopter.exe not found. Download CopterStable ArduCopter.elf from firmware.ardupilot.org/Tools/MissionPlanner/sitl/ and rename it to ArduCopter.exe
  pause & exit /b 1
)
if not exist run mkdir run
cd run
..\ArduCopter.exe --model quad --speedup 1 --home -35.363261,149.165230,584,353 --defaults ..\aptuner-sitl.parm
