// ============================================================================
//  pose/types.hpp  —  骨骼姿态叠加应用的核心数据类型
//
//  设计约束：
//   * 关节集合固定为 18 个，是「Nuitrack 19 关节」与「COCO-17」的交集，
//     因此既能直接吃 Nuitrack 数据，也能直接吃 COCO-17 姿态模型（YOLO-Pose /
//     BlazePose / MoveNet 等）的输出，不需要改上层代码。
//   * 每个关节同时保存 3D 坐标（相机坐标系，毫米，来自 Nuitrack/深度）与
//     已经投影到彩色图上的 2D 像素坐标。角度优先用 3D 算（不受透视影响），
//     3D 不可用时自动退回 2D。
// ============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace pose {

// ---------------------------------------------------------------------------
//  关节定义
// ---------------------------------------------------------------------------
enum class JointId : int {
  Head = 0,      // 头部中心（Nuitrack: HEAD；COCO: 由双眼/双耳/鼻推出）
  Neck,          // 颈部   （Nuitrack: NECK；COCO: 双肩中点）
  ShoulderL,     // 左肩
  ElbowL,        // 左肘
  WristL,        // 左手腕
  ShoulderR,     // 右肩
  ElbowR,        // 右肘
  WristR,        // 右手腕
  Spine,         // 脊柱中点/胸口（Nuitrack: TORSO；COCO: 双肩中点下移）
  HipL,          // 左髋
  KneeL,         // 左膝
  AnkleL,        // 左踝
  HipR,          // 右髋
  KneeR,         // 右膝
  AnkleR,        // 右踝
  FootL,         // 左脚（Nuitrack: LEFT_FOOT；缺失时用踝 + 竖直偏移近似）
  FootR,         // 右脚
  Count
};

constexpr int kJointCount = static_cast<int>(JointId::Count);

const char* jointName(JointId id);
const char* jointNameZh(JointId id);

// ---------------------------------------------------------------------------
//  骨骼连线（用于绘制骨架）。索引均为 JointId 的整数值。
// ---------------------------------------------------------------------------
struct Bone {
  int a;
  int b;
  bool leftSide;  // true=左侧（叠加时可用不同颜色区分左右）
};

extern const std::array<Bone, 17> kBones;

// ---------------------------------------------------------------------------
//  基础几何类型（不依赖 OpenCV，方便单元自测与复用）
// ---------------------------------------------------------------------------
struct Vec2 {
  float x = 0.f;
  float y = 0.f;
  bool valid = false;  // false = 该关节本帧未被识别/置信度太低
};

struct Vec3 {
  float x = 0.f;  // 单位：毫米（Nuitrack 输出即为毫米）
  float y = 0.f;
  float z = 0.f;  // 通常为深度方向，正 z 朝相机前方
  bool valid = false;
};

// ---------------------------------------------------------------------------
//  单帧图像
// ---------------------------------------------------------------------------
struct ColorFrame {
  int width = 0;
  int height = 0;
  int stride = 0;                 // 每行字节数
  uint64_t timestampUs = 0;       // 采集时间戳（微秒）
  std::vector<uint8_t> bgr;       // BGR8 紧凑排列，stride == width*3

  bool empty() const { return bgr.empty() || width <= 0 || height <= 0; }
};

struct DepthFrame {
  int width = 0;
  int height = 0;
  uint64_t timestampUs = 0;
  std::vector<uint16_t> mm;       // 深度值，单位 mm，0 表示无效
  std::vector<uint8_t> alignedBgr; // 与深度对齐后的彩色（可选，用于叠加对齐）

  bool empty() const { return mm.empty() || width <= 0 || height <= 0; }
};

// ---------------------------------------------------------------------------
//  一个人的姿态
// ---------------------------------------------------------------------------
struct Skeleton {
  int id = -1;                              // 跟踪 ID（跨帧稳定）
  float globalConfidence = 1.f;             // 整体置信度 0..1
  std::array<Vec3, kJointCount> joints3d;   // 相机坐标系，毫米
  std::array<Vec2, kJointCount> joints2d;   // 彩色图像素坐标
  std::array<float, kJointCount> conf{};    // 每个关节的置信度 0..1

  bool has3d() const {
    for (const auto& j : joints3d) {
      if (j.valid) return true;
    }
    return false;
  }
};

struct PoseFrame {
  uint64_t timestampUs = 0;
  std::vector<Skeleton> skeletons;
};

// ---------------------------------------------------------------------------
//  姿态角度
// ---------------------------------------------------------------------------
enum class AngleId : int {
  ElbowL = 0,   // 左肘屈伸：肩-肘-腕，180° = 完全伸直
  ElbowR,
  ShoulderL,    // 左肩外展：肘-肩-髋，0° = 手臂自然下垂
  ShoulderR,
  HipL,         // 左髋屈伸：肩-髋-膝，180° = 站直
  HipR,
  KneeL,        // 左膝屈伸：髋-膝-踝，180° = 腿伸直
  KneeR,
  Neck,         // 颈部前倾：头-颈-脊柱，180° = 头正
  Torso,        // 躯干前倾（相对相机竖直方向），0° = 直立
  Count
};

constexpr int kAngleCount = static_cast<int>(AngleId::Count);

const char* angleName(AngleId id);
const char* angleNameZh(AngleId id);

struct AngleValue {
  float deg = 0.f;
  bool valid = false;
  bool from3d = false;   // true = 用 3D 坐标计算（更准）
};

struct PoseAngles {
  std::array<AngleValue, kAngleCount> values;
  // 左右肢体是否判定为「已抬起/已跨出画面」（用于避免把侧身时的误算当真）
  bool leftArmVisible = false;
  bool rightArmVisible = false;
  bool leftLegVisible = false;
  bool rightLegVisible = false;

  const AngleValue& get(AngleId id) const { return values[static_cast<int>(id)]; }
  AngleValue& get(AngleId id) { return values[static_cast<int>(id)]; }
};

// ---------------------------------------------------------------------------
//  应用级配置
// ---------------------------------------------------------------------------
struct AppConfig {
  // ---- 数据源 -------------------------------------------------------------
  enum class SourceKind { Mock, OpenCv, Nuitrack, Orbbec };
  SourceKind source = SourceKind::Mock;
  int cameraIndex = 0;             // OpenCV 摄像头索引
  // OpenCV 采集后端：auto / dshow（普通 UVC 最稳）/ obsensor（奥比中光，可取深度）
  // / msmf / any。用 `--backend` 指定；auto 时按 dshow → obsensor → 默认 依次尝试。
  std::string captureBackend = "auto";
  std::string videoPath;           // 视频/图片文件路径（OpenCV 源）
  bool loopVideo = true;
  int colorWidth = 1280;           // 期望彩色分辨率（Nuitrack/Orbbec）
  int colorHeight = 720;
  int fps = 30;
  bool preferColorFromNuitrack = true;  // Nuitrack 模式下用 Nuitrack 彩色帧（内外参天然对齐）
                                        // 说明：这是历史命名，语义等价于
                                        // bodyTrackingColorStream。

  // ---- 姿态后端 -----------------------------------------------------------
  enum class PoseBackend {
    Dnn,            // ONNX 姿态模型（YOLOv8-pose 等），跑在彩色图上，无需 License
    Nuitrack,       // Nuitrack SkeletonTracker（第三方，需 License）
    OrbbecAstra,    // 奥比中光官方 Astra SDK Body Tracking（需 SDK + License）
    Mock            // 合成骨骼（无 SDK / 无相机时用）
  };
  PoseBackend poseBackend = PoseBackend::Mock;  // 依赖缺失时自动降级为 Mock
  std::string poseModelPath;         // ONNX 模型路径（Dnn 后端）
  float poseScoreThreshold = 0.35f;  // 人体框置信度阈值
  float poseKptThreshold = 0.25f;    // 关键点置信度阈值
  int poseInputSize = 640;           // 模型输入边长（YOLOv8-pose 为 640）
  int poseMaxPersons = 5;            // 最多输出几个人
  bool useDepthFor3d = true;         // 用深度图把 2D 关节补成 3D（角度更准）
  bool mockMirror = true;            // 模拟骨骼是否镜像（凑近摄像头时更自然）
  bool bodyTrackingColorStream = true;  // 骨骼后端自带彩色流时，顺手取回作为叠加底图
                                        // （Astra SDK / Nuitrack 都会独占相机，
                                        //   所以底图必须由同一个 SDK 提供）

  // ---- 角度 ---------------------------------------------------------------
  bool anglesFrom3d = true;        // 优先 3D 计算
  float minJointConfidence = 0.30f;

  // ---- 渲染 ---------------------------------------------------------------
  bool drawSkeleton = true;
  bool drawJointAngles = true;     // 关节处标注角度数值
  bool drawHudTable = true;        // 右上角角度表格
  bool drawDepthPeek = false;      // 左下角显示深度小窗
  bool drawFps = true;
  int boneThickness = 4;
  int jointRadius = 5;
  float fontScale = 0.55f;
  bool showChinese = false;        // 需要 cv::freetype 支持，默认关闭

  // ---- 告警阈值（超过则在图上高亮为红色）----------------------------------
  bool enableThreshold = false;
  float elbowStraightenThreshold = 165.f;  // 肘部接近伸直的角度
  float kneeStraightenThreshold = 165.f;
  float torsoLeanWarnDeg = 20.f;           // 躯干倾斜告警
  float neckTiltWarnDeg = 25.f;

  // ---- 其他 ---------------------------------------------------------------
  bool mirror = true;              // 彩色图镜像显示（自拍视角）
  bool saveAnnotatedVideo = false;
  std::string outputVideoPath = "annotated.avi";
  bool saveCsv = false;                    // 把每帧每人的角度写成 CSV，便于后续分析
  std::string csvPath = "angles.csv";
  int snapshotEveryNFrames = 0;            // >0 时每 N 帧落一张叠加结果 PNG（不用开窗口）
  std::string snapshotPrefix = "snapshot";
  bool headless = false;           // 无窗口模式，仅处理/落盘（用于服务器或调试）
  int exitAfterFrames = 0;         // >0 时处理完指定帧数自动退出（自动化测试用）
  bool debugPrint = false;         // 打印每帧关节包围盒与角度，便于定位对齐问题
};

}  // namespace pose
