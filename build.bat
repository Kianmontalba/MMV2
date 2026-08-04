@echo off
REM MMV2 GDExtension Build Script (Windows)
REM Usage: build.bat [platform] [target] [arch]

set PLATFORM=%1
if "%PLATFORM%"=="" set PLATFORM=windows

set TARGET=%2
if "%TARGET%"=="" set TARGET=template_debug

set ARCH=%3
if "%ARCH%"=="" set ARCH=x86_64

echo ========================================
echo Building MMV2 GDExtension
echo Platform: %PLATFORM%
echo Target: %TARGET%
echo Arch: %ARCH%
echo ========================================

REM Check if godot-cpp exists
if not exist "godot-cpp\" (
    echo Error: godot-cpp submodule not found!
    echo Run: git submodule add https://github.com/godotengine/godot-cpp.git
    exit /b 1
)

REM Build
scons platform=%PLATFORM% target=%TARGET% arch=%ARCH% -j4

if %ERRORLEVEL% == 0 (
    echo.
    echo Build successful!
    echo Output: demo\bin\
) else (
    echo.
    echo Build failed!
    exit /b 1
)
