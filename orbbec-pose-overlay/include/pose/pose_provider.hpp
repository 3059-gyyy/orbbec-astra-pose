// ============================================================================
//  pose/pose_provider.hpp  —  骨骼姿态数据源抽象
//
//  NuitrackPoseProvider 输出 3D 关节（毫米）+ 彩色图投影后的 2D 关节；
//  MockPoseProvider 输出一段可复现的合成动作，用于无相机自测。
// ============================================================================
#pragma once

#include <memory>
#include <string>

#include "pose/types.hpp"

namespace pose {

class PoseProvider {
 public:
  virtual ~PoseProvider() = default;

  virtual bool start(const AppConfig& cfg) = 0;
  virtual void stop() = 0;
  virtual const char* name() const = 0;
  virtual std::string error() const = 0;

  // 等待并取回与 color 时间戳最接近的一帧骨骼。
  // colorTimestampUs == 0 表示不关心同步。
  virtual bool fetch(uint64_t colorTimestampUs, PoseFrame& out) = 0;

  // 3D 关节 -> 彩色图像素的投影函数（由后端提供，保证与彩色图对齐）。
  // Nuitrack 由 SkeletonTracker 的投影信息实现；Mock 由固定虚拟相机实现。
  virtual bool projectJoint(const Vec3& in, Vec2& out) const = 0;

  // 最近一次使用的手工投影是否可用（供日志提示）
  virtual bool projectionReady() const = 0;
};

// 未编译 HAVE_NUITRACK 时返回 MockPoseProvider，并带明确提示。
std::unique_ptr<PoseProvider> makePoseProvider(const AppConfig& cfg);

// ONNX 姿态后端（YOLOv8-pose）：需要在当前彩色帧上直接推理，
// 因此单独暴露一个入口；返回 false 表示当前后端不是 DNN 或未就绪。
bool dnnRunOnFrame(PoseProvider* provider, const ColorFrame& color, PoseFrame& out);
bool dnnBackendAvailable();

// ---- 给界面工程（pose-ui）用的桥接接口 ------------------------------------
// 指定后端创建 provider（不受 cfg.poseBackend 影响）
std::unique_ptr<PoseProvider> makePoseProviderWithBackend(const AppConfig& cfg,
                                                          AppConfig::PoseBackend backend);

// 直接在裸 BGR 缓冲上做一次推理（界面工程的采集线程用）。
// 返回 false 表示当前 provider 不是 DNN 后端或尚未就绪。
bool dnnInferOnFrame(PoseProvider* provider, const AppConfig& cfg, const uint8_t* bgr,
                     int width, int height, PoseFrame& out);

const char* poseBackendName(AppConfig::PoseBackend b);
bool parsePoseBackend(const std::string& s, AppConfig::PoseBackend& out);

// 编译期可用性（用于启动时打印能力清单）
bool nuitrackCompiledIn();
bool orbbecCompiledIn();
bool astraCompiledIn();

}  // namespace pose
