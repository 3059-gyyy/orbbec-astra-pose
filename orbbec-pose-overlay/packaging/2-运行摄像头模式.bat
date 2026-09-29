@echo off
chcp 65001 >nul
title pose_overlay - camera
cd /d "%~dp0"
echo ============================================================
echo   Camera mode (mock skeleton)
echo.
echo   Camera index on THIS machine:
echo     0 = Orbbec Astra+ RGB   (real color image - use this)
echo     1 = Astra+ Depth        (black in color mode)
echo     2 = Astra+ IR           (black in color mode)
echo.
echo   For real skeleton detection use script 5 instead.
echo ============================================================
echo.
set /p CAM=Enter camera index [0]:
if "%CAM%"=="" set CAM=0
pose_overlay.exe --source opencv --pose mock --camera %CAM% --backend dshow --depth-peek
echo.
echo [exited] press any key to close
pause >nul
