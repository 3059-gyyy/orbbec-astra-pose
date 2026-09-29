@echo off
REM ===========================================================================
REM  build.bat  —  Windows 一键构建（MSVC + CMake）
REM
REM  用法：
REM    build.bat                     只启用 OpenCV（mock / opencv 数据源可用）
REM    build.bat nuitrack            额外启用 Nuitrack（需先设 NUITRACK_ROOT）
REM    build.bat nuitrack orbbec     全部启用（需先设 NUITRACK_ROOT / ORBBEC_SDK_ROOT）
REM
REM  前置条件：
REM    1) Visual Studio 2022（含"使用 C++ 的桌面开发"工作负载，即有 cl.exe）
REM    2) CMake 3.16+
REM    3) OpenCV（推荐 vcpkg：vcpkg install opencv4[core,imgproc,imgcodecs,highgui,videoio]:x64-windows）
REM       并设置环境变量 OpenCV_DIR 指向 OpenCVConfig.cmake 所在目录
REM ===========================================================================
setlocal enabledelayedexpansion

set ROOT=%~dp0
set BUILD_DIR=%ROOT%build

REM ---- 寻找 vcvars64.bat -----------------------------------------------------
set VCVARS=
for %%P in (
  "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
  "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
) do (
  if exist %%P set VCVARS=%%~P
)
if "%VCVARS%"=="" (
  echo [错误] 未找到 vcvars64.bat。
  echo        请安装 Visual Studio 2022 并勾选"使用 C++ 的桌面开发"工作负载。
  exit /b 1
)
echo [1/3] 使用编译器环境: %VCVARS%
call "%VCVARS%" >nul
if errorlevel 1 (
  echo [错误] 初始化 MSVC 环境失败。
  exit /b 1
)

REM ---- 组装 CMake 参数 -------------------------------------------------------
set EXTRA=
if /I "%1"=="nuitrack" set EXTRA=%EXTRA% -DHAVE_NUITRACK=ON
if /I "%2"=="nuitrack" set EXTRA=%EXTRA% -DHAVE_NUITRACK=ON
if /I "%1"=="orbbec"   set EXTRA=%EXTRA% -DHAVE_ORBBEC_SDK=ON
if /I "%2"=="orbbec"   set EXTRA=%EXTRA% -DHAVE_ORBBEC_SDK=ON
if /I "%1"=="all"      set EXTRA=-DHAVE_NUITRACK=ON -DHAVE_ORBBEC_SDK=ON
if /I "%2"=="all"      set EXTRA=-DHAVE_NUITRACK=ON -DHAVE_ORBBEC_SDK=ON

if defined NUITRACK_ROOT set EXTRA=%EXTRA% -DNUITRACK_ROOT="%NUITRACK_ROOT%"
if defined ORBBEC_SDK_ROOT set EXTRA=%EXTRA% -DORBBEC_SDK_ROOT="%ORBBEC_SDK_ROOT%"

echo [2/3] 配置 CMake ...%EXTRA%
cmake -S "%ROOT%." -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release %EXTRA%
if errorlevel 1 (
  echo [提示] Ninja 不可用，改用 Visual Studio 生成器重试 ...
  cmake -S "%ROOT%." -B "%BUILD_DIR%" -G "Visual Studio 17 2022" -A x64 %EXTRA%
  if errorlevel 1 (
    echo [错误] CMake 配置失败。请检查 OpenCV_DIR 是否指向 OpenCVConfig.cmake 所在目录。
    exit /b 1
  )
  echo [3/3] 编译 ...
  cmake --build "%BUILD_DIR%" --config Release --parallel
) else (
  echo [3/3] 编译 ...
  cmake --build "%BUILD_DIR%" --parallel
)
if errorlevel 1 (
  echo [错误] 编译失败，请查看上面的报错。
  exit /b 1
)

echo.
echo ==========================================================
echo  构建完成。可执行文件通常在：
echo    %BUILD_DIR%\pose_overlay.exe
echo    %BUILD_DIR%\Release\pose_overlay.exe
echo.
echo  推荐先做一次无相机自检：
echo    pose_overlay.exe --source mock --pose mock --exit-after 120
echo ==========================================================
endlocal
