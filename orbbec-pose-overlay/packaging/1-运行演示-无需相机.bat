@echo off
chcp 65001 >nul
title pose_overlay - mock demo
cd /d "%~dp0"
echo ============================================================
echo   Offline demo (no camera needed)
echo   Window keys: 1 skeleton  2 joint angles  3 HUD  4 depth
echo                5 3D/2D     m mirror        t threshold  q quit
echo ============================================================
echo.
pose_overlay.exe --source mock --pose mock --depth-peek
echo.
echo [exited] press any key to close
pause >nul
