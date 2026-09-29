@echo off
REM ===========================================================================
REM  tools\build_local.cmd  -  fast local build (Ninja + RelWithDebInfo)
REM
REM  Difference from build.bat: build.bat is for end users and probes vcvars
REM  automatically; this script targets the exact toolchain verified on this
REM  machine, so it configures and builds faster.
REM
REM  Overridable environment variables:
REM    OPENCV_DIR_PATH   OpenCV build dir containing OpenCVConfig.cmake
REM    VS_VCVARS         path to vcvars64.bat
REM    CMAKE_EXE         path to cmake.exe
REM ===========================================================================
setlocal enabledelayedexpansion

if not defined OPENCV_DIR_PATH set "OPENCV_DIR_PATH=D:\pdf\_deps\opencv\opencv\build"
if not defined VS_VCVARS set "VS_VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not defined CMAKE_EXE set "CMAKE_EXE=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

if not exist "%VS_VCVARS%" (
  echo VCVARS_NOT_FOUND: "%VS_VCVARS%"
  exit /b 1
)
if not exist "%CMAKE_EXE%" (
  echo CMAKE_NOT_FOUND: "%CMAKE_EXE%"
  exit /b 1
)

call "%VS_VCVARS%" >nul 2>&1
if errorlevel 1 (
  echo VCVARS_FAILED
  exit /b 1
)

set "PROJ=%~dp0.."
pushd "%PROJ%"

"%CMAKE_EXE%" -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DOpenCV_DIR="%OPENCV_DIR_PATH%" > build_configure.log 2>&1
set "CFG=!errorlevel!"
if not "!CFG!"=="0" (
  echo CONFIGURE_EXIT=!CFG!
  echo --- log tail ---
  powershell -NoProfile -Command "Get-Content build_configure.log -Tail 15"
  popd
  exit /b !CFG!
)

"%CMAKE_EXE%" --build build --parallel >> build_configure.log 2>&1
set "BLD=!errorlevel!"
popd

echo CONFIGURE_EXIT=!CFG!
echo BUILD_EXIT=!BLD!
exit /b !BLD!
