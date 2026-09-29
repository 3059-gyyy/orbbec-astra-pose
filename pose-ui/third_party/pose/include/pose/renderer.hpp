// ============================================================================
//  pose/renderer.hpp  —  在彩色图上叠加骨骼、角度、HUD
// ============================================================================
#pragma once

#include <opencv2/core.hpp>

#include "pose/angles.hpp"
#include "pose/types.hpp"

namespace pose {

// 每个人对应的角度结果（与 PoseFrame::skeletons 同序）
using AngleList = std::vector<PoseAngles>;

class OverlayRenderer {
 public:
  explicit OverlayRenderer(const AppConfig& cfg) : cfg_(cfg) {}

  // 在 bgr 上原地绘制。depth 可为空（不画深度小窗）。
  void draw(cv::Mat& bgr, const PoseFrame& poses, const AngleList& angles,
            const DepthFrame* depth, double fps, const std::string& sourceName,
            const std::string& backendName, const std::string& statusLine);

  // 无 OpenCV 时也可复用的判定：某角度是否应该高亮告警
  bool isWarn(AngleId id, float deg) const;

 private:
  void drawSkeleton(cv::Mat& img, const Skeleton& sk,
                    const PoseAngles& angles) const;
  void drawJoints(cv::Mat& img, const Skeleton& sk) const;
  void drawAngleLabels(cv::Mat& img, const Skeleton& sk,
                       const PoseAngles& angles) const;
  void drawHud(cv::Mat& img, const PoseFrame& poses, const AngleList& angles,
               double fps, const std::string& sourceName,
               const std::string& backendName,
               const std::string& statusLine) const;
  void drawDepthPeek(cv::Mat& img, const DepthFrame& depth) const;
  void putText(cv::Mat& img, const std::string& s, cv::Point org,
               double scale, cv::Scalar color, int thickness = 1) const;

  AppConfig cfg_;
};

}  // namespace pose
