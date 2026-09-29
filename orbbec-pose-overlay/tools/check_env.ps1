# ============================================================================
#  tools/check_env.ps1  —  pose_overlay 环境体检
#
#  只读检查，不修改任何东西：确认编译器 / CMake / OpenCV / 各 SDK 是否就位，
#  并直接打印对应的构建命令。
#
#  用法：
#    powershell -ExecutionPolicy Bypass -File tools\check_env.ps1
# ============================================================================
[CmdletBinding()]
param(
  [string]$AstraSdkRoot  = $env:ASTRA_SDK_ROOT,
  [string]$NuitrackRoot  = $env:NUITRACK_ROOT,
  [string]$OrbbecSdkRoot = $env:ORBBEC_SDK_ROOT
)

$ErrorActionPreference = 'SilentlyContinue'
function Line($ok, $name, $detail) {
  $mark = if ($ok) { '[ OK ]' } elseif ($ok -eq $null) { '[ ?? ]' } else { '[FAIL]' }
  Write-Host ("{0} {1,-26} {2}" -f $mark, $name, $detail)
}

Write-Host "==== pose_overlay 环境体检 ====" -ForegroundColor Cyan
Write-Host ""

# ---- 1. MSVC 编译器 --------------------------------------------------------
$vcvarsCandidates = @(
  "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat",
  "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
)
$vcvars = $vcvarsCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if ($vcvars) {
  Line $true 'vcvars64.bat' $vcvars
  # cl.exe 是否存在（vcvars 存在但缺 C++ 工作负载是常见坑）
  $vsRoot = Split-Path (Split-Path (Split-Path (Split-Path $vcvars)))
  $clFound = Get-ChildItem -Path $vsRoot -Filter cl.exe -Recurse -ErrorAction SilentlyContinue |
             Select-Object -First 1
  if ($clFound) {
    Line $true 'cl.exe' $clFound.FullName
  } else {
    Line $false 'cl.exe' "未找到！请在 VS Installer 中勾选「使用 C++ 的桌面开发」工作负载"
  }
} else {
  Line $false 'vcvars64.bat' '未找到，请安装 Visual Studio 2022（含 C++ 桌面开发）'
}

# ---- 2. CMake -------------------------------------------------------------
$cmake = Get-Command cmake -ErrorAction SilentlyContinue
if ($cmake) { Line $true 'cmake' (& cmake --version | Select-Object -First 1) }
else { Line $false 'cmake' '未找到，请安装 CMake 3.16+ 并加入 PATH' }

# ---- 3. OpenCV ------------------------------------------------------------
$ocvDir = $env:OpenCV_DIR
$ocvConfig = $null
if ($ocvDir) {
  $ocvConfig = Get-ChildItem -Path $ocvDir -Filter 'OpenCVConfig.cmake' -Recurse -ErrorAction SilentlyContinue |
               Select-Object -First 1
}
if (-not $ocvConfig) {
  # 常见位置兜底查找
  $guess = @('C:\opencv\build', 'C:\tools\opencv\build', 'C:\vcpkg\installed\x64-windows\share\opencv4') |
           Where-Object { Test-Path $_ }
  foreach ($g in $guess) {
    $ocvConfig = Get-ChildItem -Path $g -Filter 'OpenCVConfig.cmake' -Recurse -ErrorAction SilentlyContinue |
                 Select-Object -First 1
    if ($ocvConfig) { break }
  }
}
if ($ocvConfig) {
  Line $true 'OpenCVConfig.cmake' $ocvConfig.FullName
  Write-Host "        提示：配置时加 -DOpenCV_DIR=`"$(Split-Path $ocvConfig.FullName)`"" -ForegroundColor DarkGray
} else {
  Line $false 'OpenCV' @'
未找到 OpenCVConfig.cmake。
       安装方式（二选一）：
         vcpkg install opencv4[core,imgproc,imgcodecs,highgui,videoio]:x64-windows
         https://opencv.org/releases/ 解压后 setx OpenCV_DIR "C:\opencv\build"
'@
}

# ---- 4. 奥比中光 Astra SDK Body Tracking ----------------------------------
if ($AstraSdkRoot) {
  $astraHpp = Get-ChildItem -Path $AstraSdkRoot -Filter 'astra.hpp' -Recurse -ErrorAction SilentlyContinue |
              Where-Object { $_.FullName -match 'include' } | Select-Object -First 1
  $astraLib = Get-ChildItem -Path $AstraSdkRoot -Filter 'astra.lib' -Recurse -ErrorAction SilentlyContinue |
              Select-Object -First 1
  Line ([bool]$astraHpp) 'Astra SDK 头文件' ($(if ($astraHpp) { $astraHpp.FullName } else { "未在 $AstraSdkRoot 找到 include/astra/astra.hpp" }))
  Line ([bool]$astraLib) 'Astra SDK 库'     ($(if ($astraLib) { $astraLib.FullName } else { "未在 $AstraSdkRoot 找到 astra.lib" }))
  $license = Get-ChildItem -Path $AstraSdkRoot -Recurse -ErrorAction SilentlyContinue |
             Where-Object { $_.Name -match 'licen|\.lic$' } | Select-Object -First 3
  if ($license) {
    Line $true 'License 相关文件' (($license | ForEach-Object { $_.Name }) -join ', ')
  } else {
    Line $null 'License 相关文件' '未发现明显的 License 文件；Body Tracking 试用版过期会启动失败，需向奥比中光申请'
  }
} else {
  Line $null 'Astra SDK' '未设置 ASTRA_SDK_ROOT（要做 --pose orbbec-astra 才需要）'
}

# ---- 5. Nuitrack ----------------------------------------------------------
if ($NuitrackRoot) {
  $ntHpp = Get-ChildItem -Path $NuitrackRoot -Filter 'Nuitrack.h' -Recurse -ErrorAction SilentlyContinue |
           Select-Object -First 1
  $ntLib = Get-ChildItem -Path $NuitrackRoot -Filter 'nuitrack.lib' -Recurse -ErrorAction SilentlyContinue |
           Select-Object -First 1
  $ntDll = Get-ChildItem -Path $NuitrackRoot -Filter 'nuitrack.dll' -Recurse -ErrorAction SilentlyContinue |
           Select-Object -First 1
  Line ([bool]$ntHpp) 'Nuitrack 头文件' ($(if ($ntHpp) { $ntHpp.FullName } else { "未在 $NuitrackRoot 找到 Nuitrack.h" }))
  Line ([bool]$ntLib) 'Nuitrack 库'     ($(if ($ntLib) { $ntLib.FullName } else { "未在 $NuitrackRoot 找到 nuitrack.lib" }))
  Line ([bool]$ntDll) 'nuitrack.dll'    ($(if ($ntDll) { $ntDll.FullName } else { '未找到，运行时需要' }))
  $sample = Join-Path $NuitrackRoot 'examples'
  if (Test-Path $sample) {
    Line $true 'Nuitrack 示例' "存在：$sample（建议先跑通 console sample 再跑本程序）"
  } else {
    Line $null 'Nuitrack 示例' '未找到 examples 目录'
  }
} else {
  Line $null 'Nuitrack' '未设置 NUITRACK_ROOT（要做 --pose nuitrack 才需要）'
}

# ---- 6. Orbbec SDK v2 -----------------------------------------------------
if ($OrbbecSdkRoot) {
  $obHpp = Get-ChildItem -Path $OrbbecSdkRoot -Filter 'ObSensor.hpp' -Recurse -ErrorAction SilentlyContinue |
           Select-Object -First 1
  $obLib = Get-ChildItem -Path $OrbbecSdkRoot -Include 'ob.lib','obsensor.lib','OrbbecSDK.lib' -Recurse -ErrorAction SilentlyContinue |
           Select-Object -First 1
  Line ([bool]$obHpp) 'Orbbec SDK 头文件' ($(if ($obHpp) { $obHpp.FullName } else { "未在 $OrbbecSdkRoot 找到 libobsensor/ObSensor.hpp" }))
  Line ([bool]$obLib) 'Orbbec SDK 库'     ($(if ($obLib) { $obLib.FullName } else { '未找到 ob.lib / obsensor.lib' }))
} else {
  Line $null 'Orbbec SDK v2' '未设置 ORBBEC_SDK_ROOT（要做 --source orbbec 才需要）'
}

# ---- 7. 相机枚举（USB 设备粗略检查）--------------------------------------
Write-Host ""
Write-Host "USB 上疑似深度相机设备：" -ForegroundColor Cyan
$devs = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.FriendlyName -match 'Orbbec|Astra|Femto|Gemini|Depth|3D Camera|PrimeSense' }
if ($devs) {
  $devs | ForEach-Object { Write-Host ("   - {0}  [{1}]" -f $_.FriendlyName, $_.Status) }
} else {
  Write-Host "   （未识别到；若相机已插好，检查 USB 3.0 接口与驱动）" -ForegroundColor DarkGray
}

# ---- 8. 给出建议的构建命令 ------------------------------------------------
Write-Host ""
Write-Host "==== 建议的构建命令 ====" -ForegroundColor Cyan
$flags = @()
if ($AstraSdkRoot)  { $flags += "-DHAVE_ASTRA_SDK=ON -DASTRA_SDK_ROOT=`"$AstraSdkRoot`"" }
if ($NuitrackRoot)  { $flags += "-DHAVE_NUITRACK=ON -DNUITRACK_ROOT=`"$NuitrackRoot`"" }
if ($OrbbecSdkRoot) { $flags += "-DHAVE_ORBBEC_SDK=ON -DORBBEC_SDK_ROOT=`"$OrbbecSdkRoot`"" }
if ($ocvConfig)     { $flags += "-DOpenCV_DIR=`"$(Split-Path $ocvConfig.FullName)`"" }
Write-Host ("cmake -S . -B build -G `"Visual Studio 17 2022`" -A x64 " + ($flags -join ' '))
Write-Host "cmake --build build --config Release --parallel"
Write-Host ""
Write-Host "首次自检（不需要相机）：build\Release\pose_overlay.exe --source mock --pose mock --exit-after 120" -ForegroundColor Green
