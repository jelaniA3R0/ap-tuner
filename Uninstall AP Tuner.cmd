@echo off
rem Removes the installed copy of AP Tuner and its shortcuts (keeps nothing else).
setlocal
set "DST=%LOCALAPPDATA%\Programs\AP Tuner"
echo This removes "%DST%" and the AP Tuner shortcuts. Your session journals in that folder go too.
pause
powershell -NoProfile -Command "Get-Process node -ErrorAction SilentlyContinue | Where-Object { $_.Path -like '*AP Tuner*' } | Stop-Process -Force"
del /q "%USERPROFILE%\Desktop\AP Tuner.lnk" 2>nul
del /q "%APPDATA%\Microsoft\Windows\Start Menu\Programs\AP Tuner.lnk" 2>nul
if exist "%DST%" rmdir /s /q "%DST%"
echo Removed.
pause
