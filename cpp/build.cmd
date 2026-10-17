@echo off
rem Build and test the C++ AP Tuner with the portable toolchain (llvm-mingw + CMake + Ninja).
rem   build.cmd            release build into cpp\build\release, then run the tests
rem   build.cmd debug      debug build into cpp\build\debug
rem The toolchain folder defaults to D:\toolchain; set APTUNER_TOOLCHAIN to use another one.
setlocal
if "%APTUNER_TOOLCHAIN%"=="" set "APTUNER_TOOLCHAIN=D:\toolchain"
if exist "%APTUNER_TOOLCHAIN%\env.cmd" call "%APTUNER_TOOLCHAIN%\env.cmd"
set "CFG=Release"
set "DIR=release"
if /i "%~1"=="debug" (set "CFG=Debug" & set "DIR=debug")
set "SRC=%~dp0."
set "BUILD=%~dp0build\%DIR%"

where clang++ >nul 2>nul || (echo clang++ not found: unpack llvm-mingw into %APTUNER_TOOLCHAIN% & goto :fail)
cmake -G Ninja -S "%SRC%" -B "%BUILD%" -DCMAKE_BUILD_TYPE=%CFG% -DCMAKE_CXX_COMPILER=clang++ -DAPTUNER_WARNINGS_AS_ERRORS=ON || goto :fail
cmake --build "%BUILD%" || goto :fail
ctest --test-dir "%BUILD%" --output-on-failure || goto :fail
echo.
echo BUILD OK: %BUILD%
exit /b 0

:fail
echo.
echo BUILD FAILED
exit /b 1
