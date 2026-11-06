@echo off
rem ============================================================
rem  One-command build (no manual CMake configuration needed)
rem    build.bat              -> Debug
rem    build.bat Release      -> Release
rem    build.bat Debug clean  -> wipe build dir first
rem  Requires on PATH: cmake, ninja, arm-none-eabi-gcc
rem ============================================================
setlocal
set PRESET=%~1
if "%PRESET%"=="" set PRESET=Debug
if /i "%PRESET%"=="clean" set PRESET=Debug

where cmake >nul 2>nul
if errorlevel 1 goto no_cmake
where ninja >nul 2>nul
if errorlevel 1 goto no_ninja
where arm-none-eabi-gcc >nul 2>nul
if errorlevel 1 goto no_gcc

if /i "%~2"=="clean" (
    echo [clean] removing build\%PRESET%
    if exist "build\%PRESET%" rmdir /s /q "build\%PRESET%"
)

echo [1/2] configure  preset=%PRESET%
cmake --preset %PRESET%
if errorlevel 1 goto fail

echo [2/2] build      preset=%PRESET%
cmake --build --preset %PRESET%
if errorlevel 1 goto fail

echo.
echo ============================================================
echo  BUILD OK: build\%PRESET%\Qust_version.elf
echo  flash:  openocd -f openocd.cfg -c "program build\%PRESET%\Qust_version.elf verify reset exit"
echo ============================================================
exit /b 0

:no_cmake
echo [ERROR] cmake not found in PATH.  https://cmake.org/download/
exit /b 1
:no_ninja
echo [ERROR] ninja not found in PATH.  https://github.com/ninja-build/ninja/releases
exit /b 1
:no_gcc
echo [ERROR] arm-none-eabi-gcc not found in PATH.
echo         https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
exit /b 1
:fail
echo.
echo [FAILED] see errors above
exit /b 1
