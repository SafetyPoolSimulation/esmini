@echo off
rem Configure and build esmini on Windows with Visual Studio 2022.
rem Run from the esmini repo root: .\build_windows.bat
rem
rem First run will also download prebuilt externals (OSG, OSI, SUMO, models, ...)
rem via CMake's DOWNLOAD_EXTERNALS option, so it needs internet access.

setlocal

where cmake >nul 2>nul
if errorlevel 1 (
    if exist "C:\Program Files\CMake\bin\cmake.exe" (
        set "PATH=%PATH%;C:\Program Files\CMake\bin"
    ) else (
        echo Could not find cmake. Install CMake and/or add it to PATH, then re-run.
        exit /b 1
    )
)

if not exist build mkdir build
cd build

echo Configuring esmini...
cmake -G "Visual Studio 17 2022" -A x64 -D USE_OSG=ON ..
if errorlevel 1 goto :error

echo Building esmini (Release)...
cmake --build . --config Release --parallel
if errorlevel 1 goto :error

cd ..
echo.
echo Build succeeded.
echo Run the COLREGs boat scenario with:
echo   build\EnvironmentSimulator\Applications\esmini\Release\esmini.exe --osc resources\xosc\COLREGs-Rule13-Overtaking-WideGap-with-extra-boats-smoothyaw.xosc --path resources/xodr --window 60 60 1024 768
exit /b 0

:error
cd ..
echo.
echo Build failed.
exit /b 1
