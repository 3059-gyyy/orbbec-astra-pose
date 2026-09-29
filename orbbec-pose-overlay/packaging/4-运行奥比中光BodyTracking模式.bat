@echo off
chcp 65001 >nul
title pose_overlay - Orbbec Astra Body Tracking
cd /d "%~dp0"
echo ============================================================
echo   Orbbec official Body Tracking mode (--pose orbbec-astra)
echo.
echo   REQUIRED before this works:
echo     1) Astra SDK 2.x installed
echo     2) Body Tracking module + valid License (trial expires!)
echo     3) This build compiled with -DHAVE_ASTRA_SDK=ON
echo        (run  "pose_overlay.exe --list"  to check)
echo ============================================================
echo.
pose_overlay.exe --list
echo.
pause
pose_overlay.exe --pose orbbec-astra --depth-peek
echo.
echo [exited] press any key to close
pause >nul
