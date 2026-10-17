@echo off
cd /d "%~dp0"
title AP Tuner
echo Starting AP Tuner ... the app opens in your browser at http://127.0.0.1:8090
echo Keep this window open while you use the app. Close it to stop.
if exist "%~dp0node\node.exe" (
  "%~dp0node\node.exe" src\server.js --open
) else (
  where node >nul 2>nul || (echo Node.js was not found. Put the "node" folder back next to this file, or install Node.js from https://nodejs.org & pause & exit /b 1)
  node src\server.js --open
)
pause
