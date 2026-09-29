# pose_overlay — Astra+ 彩色图骨骼叠加与关节角度姿态应用

在**彩色图像**上实时叠加**人体骨骼**与**关节角度**的 C++ 桌面应用。
数据流：`Orbbec Astra+ 彩色/深度` → `Nuitrack 骨骼` → `角度计算` → `OpenCV 叠加渲染`。

```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  Astra+ /    │──▶│  Nuitrack    │──▶│  角度计算     │──▶│  OpenCV 叠加  │
│  USB 摄像头   │   │ SkeletonTracker │  │ 2D/3D 双模式 │   │ 骨骼+角度+HUD │
└──────────────┘   └──────────────┘   └──────────────┘   └──────────────┘
     取流层              骨骼层              算法层              渲染层
```

> ✅ **交付状态（已在本机实际编译验证）**
> 本工程已在 Windows + MSVC 14.44（VS2022 BuildTools）+ CMake 3.31.6 + Ninja + OpenCV 4.10
> 环境下**真实编译通过并跑通**，实测结果：
> * 首次编译暴露 9 处「成员变量声明在 `#if` 开关内」的错误，**已修复**；
> * `--source mock --pose mock --exit-after 200` 跑通，约 **500+ FPS**（1280×720）；
> * 叠加渲染、10 项角度计算、HUD、阈值告警、快照 PNG、角度 CSV、AVI 录制均已实测出产物；
> * 免安装绿色包已生成并**在「PATH 仅剩 System32」的干净环境变量下验证可运行**
>   （见 `release/pose_overlay_portable_v1.0.zip`，39.3 MB，含运行库、入口脚本、示例产物）。
>
> ⚠️ **仍未验证的部分**：Nuitrack / 奥比中光 Body Tracking / Orbbec SDK v2 三条真实 SDK 链路
> 本机没有 SDK、没有相机、也没有 License，**无法编译验证**。它们的调用点全部关在编译开关后
> （`HAVE_ASTRA_SDK` / `HAVE_NUITRACK` / `HAVE_ORBBEC_SDK`），并用 `// VERIFY:` 标出需要
> 按你本地 SDK 头文件核对的每一处。开启这些开关后若报错，把编译输出发我即可逐条修。
>
> 请按下文 **「4. 分三步落地」** 的顺序推进，每步都能独立验证。

---

## 1. 目录结构与交付物

```
orbbec-pose-overlay/
├─ CMakeLists.txt            构建脚本（OpenCV 必需，Astra / Nuitrack / Orbbec SDK 可选）
├─ build.bat                 Windows 一键构建（自动找 vcvars64.bat）
├─ packaging/                绿色包内的入口脚本与说明（由 make_portable.ps1 复制）
├─ tools/
│  ├─ check_env.ps1          环境体检：编译器 / CMake / OpenCV / SDK 是否就位
│  └─ make_portable.ps1      生成免安装绿色包（含运行库与示例产物）
├─ include/pose/
│  ├─ types.hpp              关节定义、骨骼连线、帧结构、AppConfig
│  ├─ geometry.hpp           纯几何工具（向量、三点角、平滑）
│  ├─ angles.hpp             角度计算与跨帧平滑接口
│  ├─ color_source.hpp       取流抽象（Mock/OpenCV/Nuitrack/Orbbec）
│  ├─ pose_provider.hpp      骨骼后端抽象（Mock/Nuitrack/Astra BodyTracking）
│  └─ renderer.hpp           叠加渲染器
└─ src/
   ├─ main.cpp               命令行解析 + 主循环 + 快捷键
   ├─ types.cpp              名称表 / 骨骼连线表 / 枚举解析
   ├─ angles.cpp             10 项关节角度 + 肢体可见性 + 平滑
   ├─ renderer.cpp           骨架、关节、角度标注、HUD 表格、深度小窗
   ├─ color_source.cpp       数据源工厂
   ├─ pose_provider.cpp      骨骼后端工厂
   ├─ mock_source.cpp        合成画面 + 合成骨骼（无相机自测）
   ├─ opencv_source.cpp      USB 摄像头 / 视频 / 图片
   ├─ nuitrack_provider.cpp  ★ Nuitrack 骨骼（含 VERIFY 标注）
   ├─ nuitrack_source.cpp    Nuitrack 彩色底图（复用同一 ColorSensor）
   ├─ orbbec_pose_provider.cpp ★ 奥比中光官方 Body Tracking 骨骼（含 VERIFY 标注）
   └─ orbbec_source.cpp      ★ Orbbec SDK v2 取流（含 VERIFY 标注）
```

**骨骼后端三选一**（可用 `--list` 查看本次编译启用了哪些）：

| `--pose` | 算法来源 | 授权 | 备注 |
|---|---|---|---|
| `orbbec-astra` | 奥比中光官方 Body Tracking | 需向奥比中光申请，试用版会过期 | 官方算法，Astra/Persee 系列兼容性最好 |
| `nuitrack` | Nuitrack（3DiVi） | 非商用免费 / 商用付费 | Astra+ 支持需在官网确认 |
| `mock` | 合成骨骼 | 无需 | 无相机也能验证叠加与角度链路 |

---

## 2. 前置条件（源码构建用）

| 组件 | 是否必需 | 说明 |
|---|---|---|
| Visual Studio 2022（含「使用 C++ 的桌面开发」） | 必需 | 提供 `cl.exe` 与 Windows SDK |
| CMake ≥ 3.16 | 必需 | 或直接用 `build.bat` |
| **OpenCV**（core/imgproc/imgcodecs/highgui/videoio） | 必需 | 取图、绘制、窗口 |
| **奥比中光 Body Tracking SDK**（Astra SDK 2.x） | 骨骼方案 A | 官方算法；需申请 License，试用版会过期 |
| **Nuitrack SDK** | 骨骼方案 B | 第三方；非商用免费，商用付费 |
| **Orbbec SDK v2** | 原生取流需要 | 不装也能用骨骼后端自带彩色流 |
| Orbbec Astra+ 相机 | 实机需要 | 无相机也能用 `--source mock` 跑通全链路 |

OpenCV 安装两条路：
```powershell
# 路线 A：vcpkg（推荐，自动配好 CMake 包）
vcpkg install opencv4[core,imgproc,imgcodecs,highgui,videoio]:x64-windows
vcpkg integrate install
# 路线 B：官方预编译包 https://opencv.org/releases/ 解压后设置
setx OpenCV_DIR "C:\opencv\build"
```

先跑环境体检（不会修改任何东西）：
```powershell
powershell -ExecutionPolicy Bypass -File tools\check_env.ps1
```

---

## 3. 源码构建

```bat
:: 只启用 OpenCV（mock / opencv 数据源，立刻可跑）
build.bat

:: 启用奥比中光官方 Body Tracking 骨骼
set ASTRA_SDK_ROOT=C:\AstraSDK
build.bat astra

:: 启用 Nuitrack 骨骼
set NUITRACK_ROOT=C:\Nuitrack
build.bat nuitrack

:: 全部启用（两个骨骼后端 + Orbbec SDK 原生取流，编译后按 --pose 选择）
set ORBBEC_SDK_ROOT=C:\OrbbecSDK
build.bat all
```

手工用 CMake 也一样：

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
      -DHAVE_ASTRA_SDK=ON -DASTRA_SDK_ROOT="C:/AstraSDK" ^
      -DHAVE_NUITRACK=ON -DNUITRACK_ROOT="C:/Nuitrack" ^
      -DHAVE_ORBBEC_SDK=ON -DORBBEC_SDK_ROOT="C:/OrbbecSDK"
cmake --build build --config Release --parallel
```

运行时把依赖 DLL（`opencv_world4xx.dll`、`nuitrack.dll`、`ob.dll`）放到 exe 同目录或加入 `PATH`。

### 3.1 打包成免安装绿色包（面向最终交付）

```powershell
powershell -ExecutionPolicy Bypass -File tools\make_portable.ps1 `
    -BuildDir build `
    -OpenCvBin "C:\opencv\build\x64\vc16\bin" `
    -OpenCvRoot "C:\opencv"
```

产物：`release\pose_overlay_portable\`（可直接压缩分发），内容为

```
pose_overlay.exe                     主程序
opencv_world4100.dll                 图像/窗口运行库
opencv_videoio_ffmpeg4100_64.dll     视频编解码（录 AVI 用）
opencv_videoio_msmf4100_64.dll       Media Foundation 采集
msvcp140*.dll / vcruntime140*.dll    VC++ 运行库（免装 Redistributable）
1-运行演示-无需相机.bat                ← 双击即可验证，不需要相机与任何环境
2-运行摄像头模式.bat
3-运行Nuitrack骨骼模式.bat
4-运行奥比中光BodyTracking模式.bat
README.txt                            面向最终用户的中文说明
licenses\                             OpenCV 许可（Apache-2.0）等
samples\                              示例产物：演示 AVI、快照 PNG、角度 CSV
```

**免安装的含义**：解压即用，目标机器不需要 Python / .NET / OpenCV / VC++ Redistributable，
只要 Windows 10/11 x64。已验证方式是把 `PATH` 收缩到只剩 `C:\Windows\System32;C:\Windows`
后运行 `pose_overlay.exe --list` 与完整处理流程，均正常。

**唯一无法随包携带的是硬件驱动**：若相机在设备管理器里显示为未知设备，
仍需安装奥比中光官方 USB 驱动（这属于硬件层，不是运行环境）。

---

## 4. 分步落地（每步都能独立验证）

### 4.1 第 1 步：无相机自检 —— 验证叠加与角度链路

```bat
build\Release\pose_overlay.exe --source mock --pose mock --exit-after 120
:: 或者用绿色包里的 1-运行演示-无需相机.bat
```
应当看到：1280×720 合成彩色画面（渐变背景 + 网格 + 人形色块）上叠着骨架、
关节旁有角度数字、左上角有 HUD 角度表；控制台打印处理帧数与平均 FPS。

想留证据（无窗口也能验证）：
```bat
pose_overlay.exe --source mock --pose mock --exit-after 120 --headless ^
                 --snapshot-every 60 --csv angles.csv --save-video out.avi
```
会得到 `snapshot_000060.png` / `snapshot_000120.png`、`angles.csv`（每帧每人的 10 项角度）、
`out.avi`（带叠加的视频）。

**这一步验证的是**：渲染、角度公式、平滑、CLI、主循环 —— 全部与 SDK 无关。

### 4.2 第 2 步：接真实摄像头（可选，验证取流与画面链路）

```bat
build\Release\pose_overlay.exe --source opencv --pose mock --camera 0
```
能看到摄像头画面 + 合成的骨架（两者不对齐是正常的，因为骨骼还是假的）。

### 4.3 第 3 步：接 Astra+ 与真实骨骼

**方案 A：奥比中光官方 Body Tracking（`--pose orbbec-astra`）**

```bat
set ASTRA_SDK_ROOT=C:\AstraSDK
build.bat astra
build\Release\pose_overlay.exe --source mock --pose orbbec-astra
```
> Astra SDK 自己会打开相机并同时给出彩色流，所以此时 `--source` 可以留 `mock`
> （仅作为没有彩色流时的底图兜底）；一旦彩色流可用，画面会自动切换为真实彩色图。
> 也可以显式写 `--source orbbec` 走 Orbbec SDK v2 取流（但**不能与 Astra SDK 同时开**）。

**方案 B：Nuitrack（`--pose nuitrack`）**

```bat
build\Release\pose_overlay.exe --source nuitrack --pose nuitrack
```

**方案 C：彩色/深度走 Orbbec SDK v2，骨骼走上述任一后端**

```bat
build\Release\pose_overlay.exe --source orbbec --pose nuitrack
```

> **设备独占**：同一台 Astra+ 不能被两个 SDK 同时打开。不要一边开 OrbbecViewer
> 一边跑本程序；`--source nuitrack`、`--source orbbec`、`--pose orbbec-astra`
> 三者也不要试图同时占用同一台相机。

---

## 5. 接入奥比中光官方 Body Tracking（`src/orbbec_pose_provider.cpp`）

> 更正说明：本文早期版本写过「奥比中光没有骨骼算法」，**这是不准确的**。
> 奥比中光确实提供骨骼姿态算法，只是入口比较隐蔽、且需要授权：
> * **Body Tracking SDK**（Astra SDK 2.x 体系）：官方说明兼容 **Astra / Persee 系列**，
>   输出 19 关节的骨骼（含 3D 坐标、彩色图投影坐标、关节朝向）；
> * **应用算法 SDK**：奥比中光 2021 年随首款 iToF 标品相机一起发布的算法套件，
>   面向 AIoT 场景提供骨骼/姿态等应用级算法；
> * 论坛里能查到大量授权与过期问题的真实反馈（见下表）。

### 5.1 获取路径与授权（务必先确认，否则会白做）

| 事项 | 说明与入口 |
|---|---|
| Body Tracking 授权申请 | 论坛帖 <https://3dclub.orbbec3d.com/t/regarding-to-body-tracking-license/2856> |
| 试用版过期 | <https://3dclub.orbbec3d.com/t/orbbec-bodytracking-trial-expired-error/2175>、<https://3dclub.orbbec3d.com/t/orbbec-2-0-9-body-tracking-sdk-expiration/2020> |
| 申请无人回复的案例 | <https://3dclub.orbbec3d.com/t/no-response-on-body-tracking-license/1776> |
| 骨骼 SDK 讨论主帖 | <https://3dclub.orbbec3d.com/t/skeleton-tracking-in-sdk/229/18> |
| 骨骼精度讨论 | <https://3dclub.orbbec3d.com/t/body-tracking-accurancy/1198> |
| **Astra+ 是否支持** | 上面的资料都指向 Astra/Persee，**Astra+ 是否在支持列表内必须向奥比中光确认** |

> 授权没谈下来之前，建议先用 `--pose nuitrack` 或 `--pose mock` 推进，
> 本工程的接口层不需要改动。

### 5.2 编译与运行

```bat
set ASTRA_SDK_ROOT=C:\AstraSDK
build.bat astra
build\Release\pose_overlay.exe --source mock --pose orbbec-astra --depth-peek
```

期望目录结构：
```
<ASTRA_SDK_ROOT>/include/astra/astra.hpp
<ASTRA_SDK_ROOT>/lib/astra.lib
```

### 5.3 需要核对的接口点（该文件内所有 `// VERIFY:`）

| 位置 | 需要确认的内容 | 如何确认 |
|---|---|---|
| `astra::initialize()` / `astra::update()` / `astra::terminate()` | 名称与轮询方式 | `include/astra/astra.hpp` |
| `astra::StreamSet` | 类型名与构造方式 | `include/astra/StreamSet.hpp` |
| `streamSet.create_reader<BodyStream>()` | 模板工厂名 | `include/astra/StreamReader.hpp` |
| `bodyReader.on_frame_ready(cb)` | 回调签名 | 官方 `samples/bodytracking` |
| `frame.get_bodies(lambda)` | 遍历接口名 | `include/astra/Body.hpp` |
| `body.id()` / `body.joints()` | 返回类型与容器语义 | 同上 |
| `astra::JointType::Head/Neck/LeftShoulder/...` | 关节枚举名（19 关节） | `include/astra/JointType.hpp` |
| `joint.depth_position` / `joint.color_position` | 3D 与已投影 2D 坐标成员名 | `include/astra/Joint.hpp` |
| `astra::get_camera_params()` | 彩色内参接口与字段名 | `include/astra/CameraParams.hpp` |
| `ColorFrame.resolution()` / `.data()` / `astra::RgbPixel` | 彩色帧取数据方式 | `include/astra/ColorFrame.hpp` |

参考实现（可直接对照这三个）：
* <https://github.com/KrisPiters/astra_body_tracker>（ROS + Astra SDK 骨骼，最接近本文件的用法）
* <https://github.com/mucks/astra-rust>（Astra SDK 的 Rust 绑定，函数名映射清晰）
* 奥比中光 Astra SDK 自带 `samples/`（bodytracking / ColorViewer）

### 5.4 关于对齐
Astra SDK 的 `Joint` 同时提供 `depth_position`（深度相机坐标系，毫米）与
`color_position`（已投影到彩色图）。本工程：
1. **优先用 `color_position`** 画叠加；
2. 拿不到时用 `get_camera_params()` 的彩色内参做针孔投影兜底；
3. 角度**优先用 3D（`depth_position`）计算**，避免透视缩短误差。

---

## 6. 接入 Nuitrack（`src/nuitrack_provider.cpp`）

### 6.1 安装与授权
1. 从 <https://nuitrack.com/> 下载并安装 Nuitrack SDK，记下安装目录（下称 `NUITRACK_ROOT`）。
2. 首次运行会要求激活 License：**非商用可申请免费 License，商用必须购买**。
   Astra+ 是否在支持列表内、以及是否需要额外驱动补丁，请在官网/社区确认后再投入开发。
3. 先跑通官方示例确认 SDK 本身没问题：
   `<NUITRACK_ROOT>\examples\nuitrack_console_sample\` —— 能看到骨骼输出再继续。

### 6.2 需要你核对的接口点（本文件内所有 `// VERIFY:`）

| 位置 | 需要确认的内容 | 如何确认 |
|---|---|---|
| `nuitrack::init()` / `nuitrack::update()` / `nuitrack::release()` | 函数名与是否需要参数 | `include/nuitrack/Nuitrack.h` |
| `nuitrack::SkeletonTracker::create()` | 工厂函数是否存在同名重载 | `include/nuitrack/SkeletonTracker.h` |
| `tracker->onSkeletonUpdate(cb)` | 回调签名（是否 `SkeletonData::Ptr`） | 同上 |
| `data->getSkeletons()` → `skel.joints` | 容器类型与下标语义 | 官方 console sample |
| `j.real.x/y/z` | 3D 成员名（部分版本为 `real`/`projective`，部分为其他命名） | `struct Joint` 定义 |
| `j.projective.x/y` | 2D 已投影坐标成员名 | `struct Joint` 定义 |
| `nuitrack::JOINT_*` 枚举名 | 关节枚举是否齐全（尤其脚部关节） | `enum JointType` |
| `nuitrack::getRgbIntrinsics()` | 是否提供、返回结构字段名 | 头文件；没有就删掉该段 |

对应关系：本工程把 Nuitrack 关节映射到内部 18 关节模型（见 `src/nuitrack_provider.cpp`
的 `kMap` 表）。**若某关节你的 SDK 里不存在，把该行 `src` 设为 `-1` 即可**，
`angles.cpp` 会自动用相邻关节外推补全（例如脚部缺失时用 `踝 + (踝-膝)*0.25`）。

### 6.3 关于对齐（很重要）
Nuitrack 的 3D 关节位于**深度相机坐标系**，必须经过它自己的标定才能落到彩色图像素上。
因此本工程：
1. **优先直接使用 `j.projective`**（Nuitrack 已算好深度↔彩色对齐）；
2. 拿不到 2D 时，才用 `getRgbIntrinsics()` 做针孔投影兜底（`fillMissing2d()`）；
3. 所有角度**优先用 3D 坐标计算**（`--angles-2d` 可强制 2D），避免透视缩短导致
   「手臂朝向镜头时肘角虚高」这类系统性误差。

---

## 7. 接入 Orbbec SDK v2（`src/orbbec_source.cpp`）

1. 安装 Orbbec SDK v2（确认版本支持 Astra+），记下目录 `ORBBEC_SDK_ROOT`。
2. 期望目录结构：
   ```
   <ROOT>/include/libobsensor/ObSensor.hpp
   <ROOT>/lib/ob.lib  （或 obsensor.lib / OrbbecSDK.lib）
   ```
3. 安装后先跑官方 `OrbbecViewer` 确认相机出图正常，再跑本程序。
4. 需要核对的接口点同样用 `// VERIFY:` 标出，主要是：
   - 命名空间（`ob::` 与版本差异）
   - `pipeline->waitForFrameset(timeout)` 名称
   - `frame->getFormat()` 返回类型与 `OB_FORMAT_*` 宏
   - `frame->getTimeStampUs()` 时间戳 API 名
   - `pipeline->getCameraParam()` 及 `rgbIntrinsic` 字段名
   - Astra+ 的深度分辨率（代码默认 640×576，如不支持需改）

---

## 8. 角度定义（`src/angles.cpp`）

| 输出名 | 三点定义 | 含义与取值范围 |
|---|---|---|
| `ElbowL/R` | 肩-肘-腕 | 肘屈伸。**180° = 手臂完全伸直**，角度越小越弯曲 |
| `ShoulderL/R` | 肘-肩-髋 | 肩关节三点角。180° ≈ 手臂上举过头，角度小 ≈ 手臂贴身下垂 |
| `HipL/R` | 肩-髋-膝 | 屈髋角。**180° = 站直**，坐姿/抬腿时减小 |
| `KneeL/R` | 髋-膝-踝 | 屈膝角。**180° = 腿伸直**，下蹲时减小 |
| `Neck` | 头-颈-脊柱 | 颈部角。**180° = 头正**，低头/仰头时偏离 |
| `Torso` | 髋中心→颈 向量 vs 竖直方向 | **0° = 躯干直立**，越大越前倾/侧倾 |
| HUD 里的 `arm.L/R`、`leg.L/R` | 腕(膝) 相对 肩(髋) 偏离竖直方向的角度 | 「抬离躯干多少度」，更贴近康复/健身的表述习惯 |

补充说明：
- 每项都标注来源 `3D` 或 `2D`：`3D` 由相机坐标计算，不受透视影响，**优先采信**。
- 角度做了指数平滑（`AngleSmoother`，α=0.6），避免数值抖动；跟踪目标消失 1.5 秒后自动清理状态。
- `--threshold` 开启后，超过阈值的角度在图上标红（`--elbow-th/--knee-th/--torso-th/--neck-th` 可调）。

---

## 9. 运行参数与快捷键

常用参数（完整列表见 `--help`）：

```
--source mock|opencv|nuitrack|orbbec     数据源
--pose   mock|nuitrack                   骨骼后端
--camera N / --video PATH                摄像头索引 / 视频文件
--width N --height N --fps N             期望分辨率与帧率
--angles-2d                              角度改用 2D 计算（默认优先 3D）
--min-conf F                             关节置信度阈值（默认 0.30）
--no-skeleton --no-joint-angles --no-hud 关闭对应绘制
--depth-peek                             显示深度小窗
--threshold --elbow-th F --knee-th F ...  阈值告警
--no-mirror                              关闭镜像显示
--save-video PATH                        把叠加结果录成 MJPEG AVI
--csv PATH                               把每帧每人的角度写成 CSV（含 3D/2D 来源列）
--snapshot-every N                       每 N 帧落一张叠加结果 PNG（无窗口也能留证据）
--debug                                  打印每帧关节像素包围盒，排查对齐问题
--headless --exit-after N                无窗口批处理（自动化验证）
--list                                   打印本次编译启用了哪些能力
```

运行时按键：`q`/`ESC` 退出，`1` 骨架，`2` 关节角度标注，`3` HUD 表格，`4` 深度小窗，
`5` 3D/2D 切换，`m` 镜像，`t` 阈值告警，`s` 保存当前帧 PNG。

---

## 10. 常见问题

| 现象 | 原因与处置 |
|---|---|
| 启动提示「未启用 Nuitrack，已降级为 mock」 | 编译时没开 `-DHAVE_NUITRACK=ON`，或 `NUITRACK_ROOT` 路径不对 |
| Nuitrack 初始化失败 | ①`NUITRACK_HOME` 未设置；②License 未激活/过期；③相机被 OrbbecViewer 等占用 |
| 骨骼画在画面外或整体偏移 | 2D 投影用的是错误的内参 → 优先让 Nuitrack 自己给 `j.projective`，别手工投影 |
| 角度数字跳动厉害 | 提高平滑：调大 `AngleSmoother` 的 α；或提高 `--min-conf` 过滤不稳定关节 |
| 相机打开失败 | 被其他程序独占；或 USB 带宽不足（降低 `--width/--height/--fps`） |
| 编译报 `ob` 未定义 | Orbbec SDK 版本命名空间差异，按 `orbbec_source.cpp` 顶部说明调整 |
| 编译报 Nuitrack 某成员不存在 | 按第 6.2 节表格逐条对照本地头文件修正（都在单个文件内） |
| 编译报 `astra::` 某成员不存在 | 按第 5.3 节表格逐条对照 Astra SDK 头文件修正（集中在 orbbec_pose_provider.cpp） |

---

## 11. 已知限制

1. **真实 SDK 链路未经编译验证**：核心程序（mock / OpenCV 链路）已在 MSVC 14.44 + OpenCV 4.10
   下实测编译并跑通；但 `HAVE_ASTRA_SDK` / `HAVE_NUITRACK` / `HAVE_ORBBEC_SDK` 三条开关
   本机没有对应 SDK 与 License，**无法验证**。打开开关后若报错，把编译输出发我逐条修
   （每处需要核对的地方都已用 `// VERIFY:` 标注，且集中在单个文件内）。
2. **Astra+ 的骨骼来源需授权**：奥比中光官方 Body Tracking 与 Nuitrack 都要申请 License；
   在授权到手前，只有 mock 骨骼可用（叠加与角度算法本身是真的在跑）。
3. **深度数据未参与骨骼解算**：骨骼来自所选后端，深度仅用于可选的深度小窗显示，
   未做「深度辅助的角度校正」。
4. **多人**：支持多人（后端多骨骼 + 稳定 ID 平滑），但 HUD 表格按人依次列出，未做分屏。
5. **未做骨架数据录制回放**：`--csv` 落的是角度数值、`--save-video` 落的是叠加后视频，
   没有「骨架轨迹回放」功能。
6. **镜像与 2D 坐标**：`--no-mirror` 时才与相机原始坐标系一致；默认镜像只是显示习惯，
   3D 角度不受影响。
