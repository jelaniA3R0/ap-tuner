@echo off
rem Removes the installed C++ edition of AP Tuner and its shortcuts (the JavaScript edition is not touched).
setlocal
set "DST=%LOCALAPPDATA%\Programs\AP Tuner C++"
echo This removes "%DST%" and the "AP Tuner (C++)" shortcuts. Your session journals in that folder go too.
pause
taskkill /im aptuner.exe /f >nul 2>nul
rem the Desktop may be redirected (OneDrive), so ask Windows where it is
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "foreach ($p in @([Environment]::GetFolderPath('Desktop'), [Environment]::GetFolderPath('Programs'))) {" ^
  "  Remove-Item -LiteralPath (Join-Path $p 'AP Tuner (C++).lnk') -ErrorAction SilentlyContinue }"
if exist "%DST%" rmdir /s /q "%DST%"
echo Removed.
pause
