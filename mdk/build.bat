@echo off
echo ===============================
echo Building RT1064 Project
echo ===============================

REM Clean previous build
if exist "Objects\*.o" del "Objects\*.o" /Q
if exist "Objects\*.d" del "Objects\*.d" /Q
if exist "Objects\*.axf" del "Objects\*.axf" /Q

REM Build project
"D:\Keil_v5\UV4\UV4.exe" -j0 -b "%~dp0rt1064.uvprojx"

echo.
echo ===============================
echo Build Complete
echo ===============================
pause
