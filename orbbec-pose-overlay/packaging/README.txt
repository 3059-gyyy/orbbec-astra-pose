================================================================================
 pose_overlay  免安装绿色包 v1.1  ——  彩色图骨骼叠加 + 关节角度姿态显示
================================================================================

【先看这个：真实骨骼检测（不需要任何 License）】
  双击：5-运行真实骨骼检测-无需License.bat

  它把你的 Astra+ 彩色画面 + ONNX 姿态模型（YOLOv8-pose）跑在一起：
    * 在彩色图上实时识别出人体 17 个关键点（鼻/眼/耳/肩/肘/腕/髋/膝/踝）
    * 换算成本工具统一的 18 关节骨架，画在画面上
    * 同时在关节旁标注角度，并在左上角列出 10 项角度数值
    * 全程本地 CPU 推理，不联网、不需要授权
  实测：1280x720 约 9~10 FPS（CPU 推理），关节位置准确。

  提示：站到相机前 1.5~3 米，让全身进入画面，检测效果最好。
        戴眼镜、侧身、低头都会影响精度，但一般仍可用。

【其他入口】
  1-运行演示-无需相机.bat       合成画面 + 合成骨骼，验证渲染与角度算法
  2-运行摄像头模式.bat          真实彩色画面 + 合成骨骼（骨骼是假的，仅验证取流）
  3-运行Nuitrack骨骼模式.bat    需要 Nuitrack SDK + License（未编译进本包）
  4-运行奥比中光BodyTracking模式.bat  需要 Astra SDK + License（未编译进本包）

【镜头与索引说明（本机实测）】
  camera index 0 = Orbbec Astra+ RGB    ← 彩色画面在这里
  camera index 1 = Astra+ Depth         彩色模式下是全黑
  camera index 2 = Astra+ IR            彩色模式下是全黑
  命令行查看可用设备：pose_overlay.exe --list

【镜像显示（重要）】
  默认开启 --mirror：画面左右翻转，像照镜子，符合自拍直觉。
  规则：翻转的是"画面 + 骨骼坐标"，两者始终保持对齐；
        HUD 文字单独绘制，不会跟着翻转（所以文字始终可读）。
  想按相机原始方向显示，加 --no-mirror。
  v1.1 修复：早期版本存在"骨骼与人体左右镜像错位"，已修正。

【窗口里能按的键】
  1 骨架连线开关    2 关节角度数值开关   3 HUD 角度表开关   4 深度小窗开关
  5 3D/2D 角度切换  m 镜像开关           t 阈值告警开关     s 保存当前帧 PNG
  q 或 ESC 退出

【这张盘里有什么】
  pose_overlay.exe                     主程序
  models\yolov8n-pose.onnx             姿态模型（12.9MB，本地推理）
  opencv_world4100.dll                 图像/窗口运行库（随包携带）
  opencv_videoio_*.dll                 视频编解码（录 AVI 用）
  msvcp140.dll / vcruntime140*.dll     VC++ 运行库（随包携带）
  samples\                             示例产物
      live_dnn_snapshot.png            真实骨骼检测实拍效果（推荐先看这张）
      demo_skeleton_angles.avi         合成演示视频
      demo_snapshot*.png               合成演示截图
      demo_angles.csv                  角度原始数据（每帧每人 10 项）
  licenses\                            第三方许可（OpenCV 为 Apache-2.0）

【运行环境要求】
  * Windows 10 / 11 64 位（x64 构建）
  * 不需要 .NET、Python、OpenCV、VC++ Redistributable —— 运行库已随包携带
  * 需要奥比中光相机驱动（本机已装；换机器需装官方 USB 驱动）

【命令行常用参数】
  --source mock|opencv|nuitrack|orbbec      数据源
  --pose   mock|dnn|nuitrack|orbbec-astra   骨骼后端
  --pose-model models\yolov8n-pose.onnx     DNN 模型路径
  --pose-score 0.35   人体框置信度阈值（调高可减少误检）
  --pose-kpt 0.25     关键点置信度阈值（调高可过滤不可靠关节）
  --camera 0 --backend dshow                相机与采集后端
  --no-mirror                               关闭镜像
  --threshold --elbow-th 165 --knee-th 165  阈值告警
  --headless --exit-after 300 --csv out.csv --save-video out.avi   无窗口批处理
  --list                                    查看能力与可用相机
  --help                                    完整参数表

【角度定义】
  ElbowL/R     肩-肘-腕   180° = 手臂完全伸直
  ShoulderL/R  肘-肩-髋   值越小越贴身，越大越上举
  HipL/R       肩-髋-膝   180° = 站直
  KneeL/R      髋-膝-踝   180° = 腿伸直
  Neck         头-颈-脊柱 180° = 头正
  Torso        髋中心→颈 与竖直方向夹角，0° = 躯干直立
  标注 2D 表示由图像坐标计算（当前 DNN 后端无深度对齐，故为 2D）。

【常见问题】
  Q: 画面里的人不是我 / 骨骼不跟着我动？
  A: 确认用 5 号脚本（真实骨骼）。2 号脚本的骨骼是合成动画，不会跟着你动。
  Q: 检测不到我？
  A: 站远一点让全身入画；或降低阈值 --pose-score 0.25 --pose-kpt 0.15。
  Q: 误检（把椅子/衣服认成人）？
  A: 调高阈值 --pose-score 0.5，或提高 --pose-kpt 0.4。
  Q: 只有 9~10 FPS？
  A: 模型跑在 CPU 上。可加 --width 640 --height 480 提升帧率。
  Q: 想换更好的模型？
  A: 任何 YOLOv8-pose 导出的 ONNX（输入 640x640）都可用 --pose-model 指定。
  Q: 相机打不开？
  A: 关掉 OrbbecViewer 等占用程序；换 USB 3.0 口；用 --list 确认索引。
     注意：--backend obsensor 在 OpenCV 4.10 下会导致崩溃，请用 dshow（默认）。

【许可与来源】
  OpenCV 4.10 (Apache-2.0)；YOLOv8n-pose ONNX 模型（Ultralytics，AGPL-3.0，
  仅用于本地推理演示；商用请自行确认模型许可）。
  本工具的骨骼叠加与角度算法为独立实现。
================================================================================
