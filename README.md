# Orbbec Astra+ 姿态与三维可视化

基于 **Orbbec Astra+** 深度相机的人体姿态检测与三维可视化工具。在彩色画面上实时叠加
人体骨骼与关节角度，并提供深度图、点云三维、骨骼坐标等视图。

本项目为 Windows 桌面程序（C++17 / OpenCV / Dear ImGui），**打包为免安装绿色包**，
目标机器无需安装 OpenCV、CUDA 或任何运行时环境。

---

## 目录结构

```
.
├── pose-ui/                 图形界面版（主程序）
│   ├── src/                 界面、采集线程、渲染、设备枚举、截图与录像
│   ├── third_party/pose/    姿态模块（骨骼模型、DNN 推理、角度、坐标系）
│   ├── tools/               构建脚本
│   └── release/             打包输出（不入库）
└── orbbec-pose-overlay/     命令行版（可处理视频文件、导出 CSV / AVI）
    ├── src/
    ├── include/pose/
    ├── tools/               构建与打包脚本
    └── packaging/           免安装包的启动脚本
```

> `pose-ui` 自带 `third_party/pose`，可独立编译；`orbbec-pose-overlay` 是功能更全的命令行版本。

---

## 功能

### 图形界面版（pose-ui）

| 功能 | 说明 |
|---|---|
| 设备选择 | 按 USB 父设备序列号聚合，**同一台相机只显示一行**；自动按视图选择数据通道 |
| 彩色叠加 | 在彩色画面上叠加骨骼、关键点、（可选）关节角度 |
| 骨骼坐标视图 | 以**地面为 Y=0、人体中心为 X=0** 输出相对坐标，带网格与地面线 |
| 深度图 / 点云三维 | 深度伪彩显示；点云可鼠标旋转缩放 |
| 截图 / 录像 | 一键存 PNG；一键录 AVI（MJPG），按时间戳命名不覆盖 |
| 免安装 | 单目录携带 OpenCV 运行时与模型，双击即用 |

### 命令行版（orbbec-pose-overlay）

支持 `--source opencv/nuitrack/orbbec/mock`、`--pose dnn/nuitrack/orbbec-astra/mock`、
`--csv` 导出角度、`--snapshot-every` 定时截图、视频文件输入等，适合批处理与回归测试。

---

## 硬件与依赖

- **相机**：Orbbec Astra+（也适用于其他 UVC 深度相机）
- **系统**：Windows 10 / 11 x64
- **编译**：Visual Studio 2022（MSVC）、CMake ≥ 3.20、Ninja（可选）
- **第三方库**：OpenCV 4.10、GLFW 3.4、Dear ImGui（docking 分支）

### 模型

姿态推理使用 **YOLOv8n-pose**（ONNX，COCO-17 关键点）：

- 放置路径：`pose-ui/models/yolov8n-pose.onnx`（与 exe 同目录）
- 下载：<https://huggingface.co/Xenova/yolov8n-pose>（`onnx/model.onnx` 重命名即可）
- 许可：**AGPL-3.0**（Ultralytics），详见 `pose-ui/models/README.txt`

若不放置模型，程序仍可运行，但骨骼检测不可用（界面会给出提示，不会崩溃）。

---

## 编译

```bat
:: 图形界面版
cd pose-ui
tools\build_ui.cmd

:: 命令行版
cd orbbec-pose-overlay
tools\build_local.cmd
```

脚本会打印 `CONFIGURE_EXIT=` 与 `BUILD_EXIT=`，均为 `0` 表示成功。
首次配置需在 `CMakeLists.txt` 顶部按本机路径调整 OpenCV / GLFW / ImGui 的位置。

---

## 打包（免安装）

编译后的 `pose_ui.exe` 需要与 OpenCV 运行时 DLL、姿态模型放在同一目录。手动组包步骤：

1. 新建目录（例如 `pose_ui_portable\`）
2. 复制 `build\pose_ui.exe`
3. 复制 OpenCV 运行时（位于 OpenCV 安装目录 `build\x64\vc16\bin\`）：
   `opencv_world4100.dll`、`opencv_videoio_ffmpeg4100_64.dll`、`opencv_videoio_msmf4100_64.dll`
4. 复制 MSVC 运行库：`msvcp140*.dll`、`vcruntime140*.dll`、`concrt140.dll`
   （位于 `C:\Windows\System32`，或使用 VS 的 `vc_redist` 安装包）
5. 复制 `models\` 目录（含 `yolov8n-pose.onnx`）与 `licenses\`
6. 复制 `config.json` 与启动脚本

命令行版提供了自动打包脚本可作参考：`orbbec-pose-overlay\tools\make_portable.ps1`。

组好的目录可直接压缩分发，目标机器**无需安装任何环境**（实测在干净 PATH 下运行正常）。

---

## 配置

`pose-ui/config.json`（与 exe 同目录，缺省时使用内置默认值）：

```jsonc
{
  "developerMode": false,   // true=显示命令行窗口看日志；false=直接进界面（默认）
  "logToFile": true,        // 隐藏窗口时仍写 pose_ui.log
  "cameraIndex": 0,
  "view": "color",          // color / depth / cloud / skeleton / coords
  "mirror": true,
  "depthStream": false,
  "poseEnabled": true,
  "poseModel": "models/yolov8n-pose.onnx",
  "captureBackend": "auto",
  "captureDir": "captures", // 截图在 snapshots\，录像在 videos\
  "videoFps": 25
}
```

命令行参数优先于配置文件。常用参数：

| 参数 | 说明 |
|---|---|
| `--dev` | 开发者模式（显示命令行窗口） |
| `--view color\|depth\|cloud\|skeleton\|coords` | 指定启动视图 |
| `--backend dshow\|obsensor\|auto` | 采集后端 |
| `--snap-after 秒` | 启动后延迟自动截图 |
| `--record 秒` | 自动录制指定时长 |
| `--test-skeleton` | 显示演示骨骼（含醒目警告，非真实检测） |

---

## 关键技术说明

### 设备聚合

同一台 Astra+ 在 Windows 上注册为两条独立 USB 链路：

```
USB\VID_2BC5&PID_0536\BY1N73300F8     彩色（独立设备，带序列号）
USB\VID_2BC5&PID_0636\BY1N73300F8     深度 + 红外（复合设备的 MI_00 / MI_02）
```

接口节点本身不含序列号，因此程序用 `CM_Locate_DevNode` + `CM_Get_Parent`
**沿设备树向上找到 USB 父节点取真序列号**，以它为唯一标识聚合，
从而把「3 个设备号」正确显示为「1 台设备 + 3 个数据通道」。

### 相对坐标系

原点 = 地面 × 人体中心：

- `X = 像素x − 人体中心x`（向右为正）
- `Y = 地面像素y − 像素y`（向上为正，地面为 0）
- 单位为**相对像素**，不做物理单位换算

地面取双脚踝中较低者，需**全身入画**；只有上半身时坐标显示 `--`（宁可不显示，也不猜原点）。

### 镜像与平滑的顺序

镜像**只作用于本帧的渲染副本**，绝不写回平滑状态。否则镜像后的坐标会被存进平滑缓冲、
下一帧又被镜像一次，导致每帧翻转 180°（X、Y 同时取反），表现为画面上下颠倒。

### 帧行序

`obsensor` 后端返回的帧行序是**自下而上**的（与 DShow 相反），需上下翻转，
否则画面颠倒、坐标 Y 出现负值。

---

## 许可

- 本项目代码：见 `LICENSE`
- **YOLOv8n-pose 模型：AGPL-3.0**（Ultralytics）——商用需自行确认授权
- OpenCV：Apache-2.0；GLFW：Zlib；Dear ImGui：MIT
- 奥比中光 SDK / Nuitrack 为可选后端，需自行遵守其各自许可

---

## 已知限制

- 深度图上的骨骼叠加按分辨率比例缩放，**未做深度-彩色外参对齐**，边缘处可能有几像素偏差
- 仅在 Orbbec Astra+ 上实测；其他 UVC 相机需自行验证
- 姿态推理为 CPU 单线程 ONNX（约 10 FPS @ 1280×720），未启用 GPU
