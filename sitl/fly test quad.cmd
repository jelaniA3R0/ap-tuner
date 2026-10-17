@echo off
rem Flies the SITL quad: arms, climbs to 12 m in AltHold and taps the sticks for 10 minutes.
rem Start run-sitl.cmd first. Then connect AP Tuner to TCP 127.0.0.1 port 5762.
cd /d "%~dp0.."
if exist "node\node.exe" ( node\node.exe sitl\pilot.js 5760 10 ) else ( node sitl\pilot.js 5760 10 )
pause
