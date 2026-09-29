@echo off
REM ===========================================================================
REM  tools\build_ui.cmd  -  build the GUI app (pose_ui)
REM
REM  Overridable environment variables:
REM    OPENCV_DIR_PATH   OpenCV build dir (contains OpenCVConfig.cmake)
REM    GLFW_ROOT         GLFW root (contains include/GLFW and lib-vc20xx)
REM    IMGUI_ROOT        Dear ImGui root (contains imgui.cpp and backends/)
REM    VS_VCVARS         path to vcvars64.bat
REM    CMAKE_EXE         path to cmake.exe
REM ===========================================================================
setlocal enabledelayedexpansion

if not defined OPENCV_DIR_PATH set "OPENCV_DIR_PATH=D:\pdf\_deps\opencv\opencv\build"
if not defined GLFW_ROOT set "GLFW_ROOT=D:/pdf/_deps/glfw/glfw-3.4.bin.WIN64"
if not defined IMGUI_ROOT set "IMGUI_ROOT=D:/pdf/_deps/imgui/imgui-docking"
if not defined VS_VCVARS set "VS_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not defined CMAKE_EXE set "CMAKE_EXE=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

call "%VS_VCVARS%" >nul 2>&1
if errorlevel 1 ( echo VCVARS_FAILED & exit /b 1 )

set "PROJ=%~dp0.."
pushd "%PROJ%"

"%CMAKE_EXE%" -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo ^
  -DOpenCV_DIR="%OPENCV_DIR_PATH%" -DGLFW_ROOT="%GLFW_ROOT%" -DIMGUI_ROOT="%IMGUI_ROOT%" ^
  > build_configure.log 2>&1
set "CFG=!errorlevel!"
if not "!CFG!"=="0" (
  echo CONFIGURE_EXIT=!CFG!
  echo --- log tail ---
  powershell -NoProfile -Command "Get-Content build_configure.log -Tail 20"
  popd
  exit /b !CFG!
)

"%CMAKE_EXE%" --build build --parallel >> build_configure.log 2>&1
set "BLD=!errorlevel!"
popd
echo CONFIGURE_EXIT=!CFG!
echo BUILD_EXIT=!BLD!
exit /b !BLD!
