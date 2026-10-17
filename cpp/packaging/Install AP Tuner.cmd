@echo off
rem Installs AP Tuner (C++ edition) for the current user: copies this folder to
rem %LOCALAPPDATA%\Programs\AP Tuner C++ and creates Desktop and Start Menu shortcuts.
rem No admin rights, no runtime to install. It runs next to the JavaScript edition (its own folder and port 8092).
setlocal
set "SRC=%~dp0"
set "DST=%LOCALAPPDATA%\Programs\AP Tuner C++"
echo Installing AP Tuner (C++ edition) to "%DST%" ...
if not exist "%DST%" mkdir "%DST%"
rem stop a running copy so its files can be replaced
taskkill /im aptuner.exe /f >nul 2>nul
robocopy "%SRC%." "%DST%" /E /NFL /NDL /NJH /NJS /XD sessions /XF "Install AP Tuner.cmd" >nul
if not exist "%DST%\sessions" mkdir "%DST%\sessions"
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ws = New-Object -ComObject WScript.Shell;" ^
  "foreach ($p in @([Environment]::GetFolderPath('Desktop'), [Environment]::GetFolderPath('Programs'))) {" ^
  "  $s = $ws.CreateShortcut((Join-Path $p 'AP Tuner (C++).lnk'));" ^
  "  $s.TargetPath = '%DST%\aptuner.exe'; $s.Arguments = '--port 8092';" ^
  "  $s.WorkingDirectory = '%DST%'; $s.IconLocation = '%DST%\aptuner.exe,0'; $s.Description = 'AP Tuner (C++ edition) - ArduPilot tuning'; $s.Save() }"
echo.
echo Done. You now have "AP Tuner (C++)" on the Desktop and in the Start Menu.
echo Starting it now ...
start "" "%DST%\aptuner.exe" --port 8092
pause
