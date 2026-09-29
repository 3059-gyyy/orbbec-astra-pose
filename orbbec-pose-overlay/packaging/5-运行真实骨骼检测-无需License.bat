@echo off
chcp 65001 >nul
title pose_overlay - 真实骨骼检测 (ONNX)
cd /d "%~dp0"
echo ============================================================
echo   Real skeleton detection on the color image (no license needed)
echo   Model: models\yolov8n-pose.onnx  (ONNX, runs on CPU)
echo.
echo   Camera: index 0 = Astra+ RGB (confirmed on this machine)
echo   Keys  : 1 skeleton  2 joint angles  3 HUD  4 depth
echo           5 3D/2D     m mirror        t threshold   q quit
echo.
echo   Tip: stand 1.5-3 m in front of the camera, whole body in view.
echo ============================================================
echo.

REM 保持窗口在前台，避免用户找不到画面
if not exist "models\yolov8n-pose.onnx" (
  echo [ERROR] models\yolov8n-pose.onnx not found.
  echo         Download it with:
  echo         curl -L -o models\yolov8n-pose.onnx https://huggingface.co/Xenova/yolov8n-pose/resolve/main/onnx/model.onnx
  pause
  exit /b 1
)

pose_overlay.exe --source opencv --pose dnn --pose-model models\yolov8n-pose.onnx ^
  --camera 0 --backend dshow --width 1280 --height 720 ^
  --pose-score 0.35 --pose-kpt 0.25 --depth-peek
echo.
echo [exited] press any key to close
pause >nul
