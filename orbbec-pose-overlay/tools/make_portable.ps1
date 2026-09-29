# ============================================================================
#  tools/make_portable.ps1  —  生成免安装绿色包
#
#  产物：release\pose_overlay_portable\
#    pose_overlay.exe          主程序（Release）
#    *.dll                     运行库（OpenCV + VC++ 运行时），随包携带
#    *.bat                     双击即可运行的入口脚本
#    README.txt / LICENSE-*    使用说明与许可
#    samples\                  示例产物（演示视频 / 快照 / 角度 CSV）
#
#  用法：
#    powershell -ExecutionPolicy Bypass -File tools\make_portable.ps1 `
#        -BuildDir build -OpenCvBin D:\pdf\_deps\opencv\opencv\build\x64\vc16\bin `
#        -OutDir release\pose_overlay_portable
# ============================================================================
[CmdletBinding()]
param(
  [string]$BuildDir = "build",
  [Parameter(Mandatory=$true)][string]$OpenCvBin,
  [string]$OutDir = "release\pose_overlay_portable",
  [string]$OpenCvRoot = "",
  [switch]$SkipSamples
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$exe  = Join-Path $root (Join-Path $BuildDir 'pose_overlay.exe')
if (-not (Test-Path $exe)) { throw "找不到 $exe，请先构建项目" }

if (-not $OpenCvRoot) { $OpenCvRoot = Split-Path -Parent (Split-Path -Parent $OpenCvBin) }

$out = Join-Path $root $OutDir
if (Test-Path $out) { Remove-Item $out -Recurse -Force }
New-Item -ItemType Directory -Force -Path $out | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $out 'samples') | Out-Null

Write-Host "[1/5] 复制主程序与入口脚本..." -ForegroundColor Cyan
Copy-Item $exe (Join-Path $out 'pose_overlay.exe')
Get-ChildItem (Join-Path $root 'packaging') -File | ForEach-Object {
  Copy-Item $_.FullName $out -Force
  Write-Host "      + $($_.Name)"
}

Write-Host "[2/5] 复制 OpenCV 运行库 ..." -ForegroundColor Cyan
$ocvDlls = @('opencv_world4100.dll', 'opencv_videoio_ffmpeg4100_64.dll',
             'opencv_videoio_msmf4100_64.dll')
foreach ($d in $ocvDlls) {
  $src = Join-Path $OpenCvBin $d
  if (Test-Path $src) { Copy-Item $src $out; Write-Host "      + $d" }
  else { Write-Warning "      未找到 $d（若版本号不同，请用 -OpenCvBin 指定正确目录）" }
}

# ONNX 姿态模型（有了它才能做"无需 License 的真实骨骼检测"）
$modelSrc = Join-Path $root 'models\yolov8n-pose.onnx'
if (-not (Test-Path $modelSrc)) {
  $alt = 'D:\pdf\_deps\models\yolov8n-pose.onnx'
  if (Test-Path $alt) { $modelSrc = $alt } else { $modelSrc = '' }
}
if ($modelSrc -ne '') {
  New-Item -ItemType Directory -Force -Path (Join-Path $out 'models') | Out-Null
  Copy-Item $modelSrc (Join-Path $out 'models\yolov8n-pose.onnx') -Force
  Write-Host ("      + models\yolov8n-pose.onnx ({0} MB)" -f [math]::Round((Get-Item $modelSrc).Length/1MB,1)) -ForegroundColor Gray
} else {
  Write-Warning "      未找到 yolov8n-pose.onnx，5 号脚本将不可用"
}

Write-Host "[3/5] 复制 VC++ 运行库（免装 Redistributable）..." -ForegroundColor Cyan
$crtNames = @('msvcp140.dll','msvcp140_1.dll','msvcp140_2.dll','vcruntime140.dll',
              'vcruntime140_1.dll','concrt140.dll')
$crtFound = @{}
# 从 VS 的 Redist 目录或系统目录搜集合乎 x64 的版本
$searchDirs = @("$env:SystemRoot\System32")
$vsRedist = Get-ChildItem "C:\Program Files (x86)\Microsoft Visual Studio" -Directory -ErrorAction SilentlyContinue |
            ForEach-Object { Join-Path $_.FullName 'VC\Redist\MSVC' } | Where-Object { Test-Path $_ }
foreach ($r in $vsRedist) {
  Get-ChildItem $r -Directory -ErrorAction SilentlyContinue | ForEach-Object {
    $searchDirs += (Join-Path $_.FullName 'x64\Microsoft.VC143.CRT')
    $searchDirs += (Join-Path $_.FullName 'x64\Microsoft.VC142.CRT')
  }
}
foreach ($n in $crtNames) {
  foreach ($d in $searchDirs) {
    $p = Join-Path $d $n
    if ((Test-Path $p) -and -not $crtFound.ContainsKey($n)) {
      $crtFound[$n] = $p
      Copy-Item $p $out -Force
      Write-Host "      + $n"
      break
    }
  }
  if (-not $crtFound.ContainsKey($n)) { Write-Warning "      未找到 $n（目标机器可能已有）" }
}

if (-not $SkipSamples) {
  Write-Host "[4/5] 生成示例产物（mock 演示 120 帧）..." -ForegroundColor Cyan
  $samples = Join-Path $out 'samples'
  Push-Location $samples
  try {
    & (Join-Path $out 'pose_overlay.exe') --source mock --pose mock --exit-after 120 `
        --headless --save-video demo_skeleton_angles.avi `
        --csv demo_angles.csv --snapshot-every 60 --threshold | Out-Null
    Get-ChildItem (Join-Path $samples 'snapshot_*.png') -ErrorAction SilentlyContinue |
      ForEach-Object { Rename-Item $_.FullName ("demo_snapshot{0}.png" -f $_.BaseName.Replace('snapshot_','')) }
  } catch {
    Write-Warning "      示例生成失败：$($_.Exception.Message)"
  } finally { Pop-Location }
  # 示例不需要 CRT/OpenCV 之外的任何东西，产物很小
  Get-ChildItem $samples | Select-Object Name, @{n='KB';e={[math]::Round($_.Length/1KB,0)}} |
    Format-Table -AutoSize
}

Write-Host "[5/5] 写入许可与说明 ..." -ForegroundColor Cyan
$licDir = Join-Path $out 'licenses'
New-Item -ItemType Directory -Force -Path $licDir | Out-Null
foreach ($n in @('LICENSE.txt','LICENSE-APACHE-2.0.txt','LICENSE-MIT.txt','ThirdPartyLicenses.txt')) {
  $p = Join-Path $OpenCvRoot $n
  if (Test-Path $p) { Copy-Item $p (Join-Path $licDir "OpenCV-$n") }
}
Copy-Item (Join-Path $root 'README.md') (Join-Path $licDir 'pose_overlay-README.md') -ErrorAction SilentlyContinue

$sizeMB = [math]::Round(((Get-ChildItem $out -Recurse | Measure-Object Length -Sum).Sum / 1MB), 1)
Write-Host ""
Write-Host "完成：$out  （总计约 $sizeMB MB）" -ForegroundColor Green
Get-ChildItem $out | Select-Object Name, @{n='MB';e={ if ($_.PSIsContainer) { '' } else { [math]::Round($_.Length/1MB,1) } }} |
  Format-Table -AutoSize
