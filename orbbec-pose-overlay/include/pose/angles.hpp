// ============================================================================
//  pose/angles.hpp  —  姿态角度计算与跨帧平滑
//
//  角度定义（见 README「角度定义」一节）：
//    ElbowL/R     肩-肘-腕   三点角，180° = 手臂完全伸直
//    ShoulderL/R  肘-肩-髋   外展/上举角，0° ≈ 手臂自然下垂贴身
//    HipL/R       肩-髋-膝   屈髋角，180° = 站直
//    KneeL/R      髋-膝-踝   屈膝角，180° = 腿伸直
//    Neck         头-颈-脊柱 颈部角，180° = 头正不低头
//    Torso        髋中心->颈   与相机竖直方向夹角，0° = 直立
//
//  优先使用 3D 坐标（不受透视投影影响），3D 不可用时自动退回 2D。
// ============================================================================
#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "pose/types.hpp"

namespace pose {

// 计算单人角度。3D 与 2D 都会尝试，按 cfg.anglesFrom3d 决定优先顺序。
PoseAngles computeAngles(const Skeleton& sk, const AppConfig& cfg);

// 原地计算，避免每帧分配。
void computeAngles(const Skeleton& sk, const AppConfig& cfg, PoseAngles& out);

// 对每个人的每项角度做指数平滑，输出稳定读数。
// 内部按 skeleton id 维护状态：新 id 自动插入，长期未出现的 id 自动淘汰。
class AngleSmoother {
 public:
  void configure(float alpha) { alpha_ = alpha; }
  void reset() { states_.clear(); }

  void apply(int skeletonId, PoseAngles& angles, uint64_t timestampUs);

 private:
  struct State {
    std::array<float, kAngleCount> deg{};
    std::array<bool, kAngleCount> init{};
    uint64_t lastSeenUs = 0;
  };

  float alpha_ = 0.6f;  // 0 = 不平滑；越大越平滑（保留越多历史）
  std::vector<std::pair<int, State>> states_;

  State& stateFor(int id, uint64_t nowUs);
  void evictStale(uint64_t nowUs);
};

// 供 HUD/日志使用：把某人的角度格式化成多行文本
std::string formatAnglesText(const PoseAngles& angles, bool chinese);

}  // namespace pose
