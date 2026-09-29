@echo off
chcp 65001 >nul
title pose_overlay - Nuitrack
cd /d "%~dp0"
echo ============================================================
echo   Nuitrack skeleton mode  (--source nuitrack --pose nuitrack)
echo.
echo   REQUIRED before this works:
echo     1) Nuitrack SDK installed + License activated
echo     2) This build must have been compiled with -DHAVE_NUITRACK=ON
echo        (run  "pose_overlay.exe --list"  to check)
echo   If Nuitrack was not compiled in, the program says so and
echo   falls back to synthetic skeleton.
echo ============================================================
echo.
pose_overlay.exe --list
echo.
pause
pose_overlay.exe --source nuitrack --pose nuitrack --depth-peek
echo.
echo [exited] press any key to close
pause >nul
