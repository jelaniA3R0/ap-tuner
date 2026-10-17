@echo off
rem Installs AP Tuner for the current user: copies this folder to %LOCALAPPDATA%\Programs\AP Tuner
rem and creates Desktop and Start Menu shortcuts. No admin rights needed. Run "Uninstall AP Tuner.cmd" to remove.
setlocal
set "SRC=%~dp0"
set "DST=%LOCALAPPDATA%\Programs\AP Tuner"
echo Installing AP Tuner to "%DST%" ...
if not exist "%DST%" mkdir "%DST%"
robocopy "%SRC%." "%DST%" /E /NFL /NDL /NJH /NJS /XD sessions run /XF "Install AP Tuner.cmd" >nul
if not exist "%DST%\sessions" mkdir "%DST%\sessions"
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ws = New-Object -ComObject WScript.Shell;" ^
  "foreach ($p in @([Environment]::GetFolderPath('Desktop'), [Environment]::GetFolderPath('Programs'))) {" ^
  "  $s = $ws.CreateShortcut((Join-Path $p 'AP Tuner.lnk'));" ^
  "  $s.TargetPath = 'wscript.exe'; $s.Arguments = '\"' + '%DST%\AP Tuner.vbs' + '\"';" ^
  "  $s.WorkingDirectory = '%DST%'; $s.IconLocation = '%DST%\AP Tuner.ico'; $s.Description = 'AP Tuner - ArduPilot tuning'; $s.Save() }"
echo.
echo Done. You now have "AP Tuner" on the Desktop and in the Start Menu.
echo Starting it now ...
start "" wscript.exe "%DST%\AP Tuner.vbs"
pause
