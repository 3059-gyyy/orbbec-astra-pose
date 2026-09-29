// ============================================================================
//  pose/world_coords.hpp  —  骨骼点的"人体相对坐标系"
//
//  坐标系定义（按需求）：
//    * 原点：地面 × 人体中心（左右中线的正下方）
//    * Y 轴：竖直向上，地面 = Y 0，头顶方向为正
//    * X 轴：水平向右，人体中心 = X 0
//    * 单位：像素（相对坐标，不做任何物理单位换算）
//
//  实现说明：
//    这是一次纯平移，不含缩放，因此几何关系与图像完全一致、无近似误差：
//        X = 关节像素 x - 人体中心像素 x
//        Y = 地面像素 y - 关节像素 y      （图像 y 向下，故取反）
//    地面像素 y 取身体最低点（左右踝的最大 y），因此人站得越低、跳跃时，
//    Y=0 会随身体最低点移动——这是"以人体自身为基准"的必然结果。
//    若需要"世界地面固定"的坐标，则应使用深度图或多帧估计地面，属于后续增强。
// ============================================================================
#pragma once

#include <string>
#include <vector>

#include "pose/angles.hpp"
#include "pose/types.hpp"

namespace pose {

struct RelCoord {
  float x = 0.f;        // 水平相对量（像素），人体中心 = 0，向右为正
  float y = 0.f;        // 竖直相对量（像素），地面 = 0，向上为正
  bool valid = false;
};

// 每帧每人的参考系（由骨架自身推得）
struct GroundCalib {
  bool valid = false;
  float groundYpx = 0.f;   // 地面（身体最低点）在图像中的 y
  float centerXpx = 0.f;   // 人体中心在图像中的 x
  float spanPx = 0.f;      // 身体总高度（像素），供界面显示比例参考

  // 归一化显示用（按身体高度折算成 -0.5~+0.5 左右的相对比例）。
  // 仅用于"按身高归一化"的显示选项，不改变原始相对坐标。
  float bodyHeightPx() const { return spanPx; }
};

// 从骨架推出参考系：地面 = 最低踝点，人体中心 = 左右髋中点（缺失时用肩中点）
GroundCalib calibrateRelative(const Skeleton& sk);

// 换算全部关节（与 JointId 同序）
std::vector<RelCoord> toRelativeCoords(const Skeleton& sk, const GroundCalib& calib);

// 便捷接口
std::vector<RelCoord> computeRelativeCoords(const Skeleton& sk, GroundCalib* calibOut = nullptr);

// 格式化：单位像素，例如 "(120, -430)"；normalized=true 时按身高折算成比例
std::string formatCoord(const RelCoord& c, bool normalized = false, float bodyHeightPx = 0.f);

}  // namespace pose
