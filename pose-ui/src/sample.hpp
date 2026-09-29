// ============================================================================
//  src/sample.hpp  —  采集线程与界面线程之间传递的一帧数据
//
//  设计要点：
//   * 采集/推理在独立线程完成，界面线程只读取"最新一帧"，互不阻塞；
//   * 数据自带时间戳与统计信息（FPS、推理耗时、人数），界面直接展示；
//   * BGR/深度都为紧凑排列，方便直接上传成 OpenGL 纹理。
// ============================================================================
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "pose/types.hpp"

namespace ui {

// 深度着色方式（纯计算，不需要贴图素材）
enum class DepthColorMap {
  Turbo = 0,   // 高对比彩虹（默认，近处暖色）
  Jet,
  Viridis,
  Gray,
  Count
};

const char* depthColorMapName(DepthColorMap m);

// 视图模式
enum class ViewMode {
  Color = 0,       // 彩色图像（可叠加骨骼）
  Depth,           // 深度图（伪彩色）
  PointCloud,      // 点云（OpenGL 三维）
  SkeletonOnly,    // 单独输出骨骼（深色背景，只画骨架与角度）
  SkeletonCoords,  // 骨骼坐标：地面 Y=0 / 人体中心 X=0，逐点标注 (X, Y)
  Count
};

const char* viewModeName(ViewMode m);

// 一帧完整结果
struct Sample {
  bool valid = false;

  // ---- 图像 ----
  int colorW = 0;
  int colorH = 0;
  std::vector<uint8_t> bgr;        // 紧凑 BGR8，colorW*colorH*3

  int depthW = 0;
  int depthH = 0;
  std::vector<uint16_t> depthMm;   // 深度，单位 mm，0 = 无效

  // ---- 相机内参（点云必需；某些后端不提供）----
  double fx = 0.0;
  double fy = 0.0;
  double icx = 0.0;
  double icy = 0.0;
  bool hasIntrinsics = false;
  bool usingObsensor = false;      // 是否走 obsensor（彩色+深度同设备）

  // ---- 骨骼 ----
  pose::PoseFrame poses;
  std::vector<pose::PoseAngles> angles;   // 与 poses.skeletons 同序

  // ---- 统计 ----
  uint64_t timestampUs = 0;
  double captureMs = 0.0;    // 取流耗时
  double inferMs = 0.0;      // 姿态推理耗时
  double frameFps = 0.0;     // 采集线程实际帧率
  std::string note;          // 状态提示（例如"相机未开启"）
};

// 界面侧可调参数（由 UI 线程写入，采集线程读取；用原子量/互斥保护）
struct PipelineOptions {
  int cameraIndex = 0;
  int width = 1280;
  int height = 720;
  int fps = 30;
  bool mirror = true;

  std::string captureBackend = "dshow";

  bool cameraOn = true;
  bool depthEnabled = false;      // 默认关闭：obsensor 取深度较慢，按需开启
  bool flipVertical = true;       // obsensor 帧行序自下而上，需上下翻转
  bool poseEnabled = true;        // 是否做骨骼检测
  std::string modelPath;
  float poseScoreThreshold = 0.35f;
  float poseKptThreshold = 0.25f;
  int poseInputSize = 640;
  int poseMaxPersons = 5;
  float scoreThreshold = 0.35f;
  float kptThreshold = 0.25f;
  int inputSize = 640;
  int maxPersons = 5;

  // 角度计算参数（沿用 pose_overlay 的实现）
  bool anglesFrom3d = true;
  float minJointConfidence = 0.25f;

  // 点云与深度显示
  DepthColorMap depthMap = DepthColorMap::Turbo;
  int depthMinMm = 300;
  int depthMaxMm = 4000;
  int cloudStride = 2;            // 点云抽样步长（越大点越少、越流畅）
  float pointSize = 2.0f;
  bool cloudUseColor = true;      // 点云用彩色图着色；否则按深度着色

  // 骨骼绘制
  bool drawBones = true;
  bool drawJoints = true;
  bool drawJointAngles = true;
  bool drawIds = true;
  int boneThickness = 3;
  int jointRadius = 4;
  float skeletonAlpha = 1.0f;
};

}  // namespace ui
