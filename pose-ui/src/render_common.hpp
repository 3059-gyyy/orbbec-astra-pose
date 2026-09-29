// ============================================================================
//  src/render_common.hpp  —  共用绘制工具：骨骼/角度、人体坐标系、深度着色
//  纯 CPU 绘制（OpenCV），结果直接上传成纹理；不需要任何贴图素材。
// ============================================================================
#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "pose/angles.hpp"
#include "pose/world_coords.hpp"
#include "sample.hpp"

namespace ui {

struct DrawStyle {
  bool bones = true;
  bool joints = true;
  bool jointAngles = false;  // 默认不显示关节角度数字（避免与坐标标注混淆）
  bool ids = false;   // 默认不显示人物 ID（只保留节点坐标）
  int boneThickness = 3;
  int jointRadius = 4;
  float alpha = 1.0f;      // 0..1，骨骼整体不透明度（用叠加混合实现）

  // ---- 人体相对坐标系显示 ----
  bool jointCoords = true;    // 默认在每个关节旁标注相对坐标 (X, Y)
  bool coordGrid = true;      // 画地面网格与坐标轴
  bool coordNormalized = false;  // 坐标按身高归一化显示（地面 0 / 头顶约 1）
};

// 在 BGR 图上绘制骨骼（含关节点、角度数值、ID 标签）
void drawSkeletonOverlay(cv::Mat& bgr, const pose::PoseFrame& poses,
                         const std::vector<pose::PoseAngles>& angles,
                         const DrawStyle& style);

// 单独输出骨骼：在指定背景色的画布上只画骨架（不画相机画面）
void drawSkeletonOnly(cv::Mat& canvas, const pose::PoseFrame& poses,
                      const std::vector<pose::PoseAngles>& angles,
                      const DrawStyle& style, const cv::Scalar& background);

// 人体坐标系视图：地面网格 + 坐标轴 + 骨架 + 每个关节的 (X, Y) 数值。
// 背景为纯色块，不显示相机画面。calibOut 可选，用于回传标定信息。
void drawCoordView(cv::Mat& canvas, const pose::PoseFrame& poses, const DrawStyle& style,
                   const cv::Scalar& background,
                   std::vector<pose::GroundCalib>* calibOut = nullptr);

// 深度图着色：uint16(mm) -> BGR
void colorizeDepth(const std::vector<uint16_t>& mm, int w, int h, DepthColorMap map,
                   int minMm, int maxMm, cv::Mat& bgrOut);

// 生成空画布的辅助（避免每次分配）
void ensureBgr(cv::Mat& m, int w, int h, const cv::Scalar& fill);

}  // namespace ui
