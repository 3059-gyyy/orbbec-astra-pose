// ============================================================================
//  src/skeleton_draw.cpp  —  骨骼与深度的 CPU 绘制实现
// ============================================================================
#include "render_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

#include "pose/types.hpp"

namespace ui {
namespace {

// 配色（BGR）：左侧青、右侧绿、躯干黄；关节点绿/橙/红按置信度
const cv::Scalar kLeft(230, 200, 0);
const cv::Scalar kRight(60, 220, 0);
const cv::Scalar kTorso(0, 200, 255);
const cv::Scalar kText(245, 245, 245);
const cv::Scalar kOutline(20, 20, 20);

bool isLeftJoint(int id) {
  return id == static_cast<int>(pose::JointId::ShoulderL) ||
         id == static_cast<int>(pose::JointId::ElbowL) ||
         id == static_cast<int>(pose::JointId::WristL) ||
         id == static_cast<int>(pose::JointId::HipL) ||
         id == static_cast<int>(pose::JointId::KneeL) ||
         id == static_cast<int>(pose::JointId::AnkleL) ||
         id == static_cast<int>(pose::JointId::FootL);
}
bool isRightJoint(int id) {
  return id == static_cast<int>(pose::JointId::ShoulderR) ||
         id == static_cast<int>(pose::JointId::ElbowR) ||
         id == static_cast<int>(pose::JointId::WristR) ||
         id == static_cast<int>(pose::JointId::HipR) ||
         id == static_cast<int>(pose::JointId::KneeR) ||
         id == static_cast<int>(pose::JointId::AnkleR) ||
         id == static_cast<int>(pose::JointId::FootR);
}

// 关节 -> 角度项的对应（用于在关节点旁标注角度）
bool angleForJoint(int joint, pose::AngleId& out) {
  switch (static_cast<pose::JointId>(joint)) {
    case pose::JointId::ElbowL: out = pose::AngleId::ElbowL; return true;
    case pose::JointId::ElbowR: out = pose::AngleId::ElbowR; return true;
    case pose::JointId::ShoulderL: out = pose::AngleId::ShoulderL; return true;
    case pose::JointId::ShoulderR: out = pose::AngleId::ShoulderR; return true;
    case pose::JointId::HipL: out = pose::AngleId::HipL; return true;
    case pose::JointId::HipR: out = pose::AngleId::HipR; return true;
    case pose::JointId::KneeL: out = pose::AngleId::KneeL; return true;
    case pose::JointId::KneeR: out = pose::AngleId::KneeR; return true;
    case pose::JointId::Neck: out = pose::AngleId::Neck; return true;
    default: return false;
  }
}

void textOutlined(cv::Mat& img, const std::string& s, cv::Point org, double scale,
                  const cv::Scalar& color) {
  cv::putText(img, s, org, cv::FONT_HERSHEY_SIMPLEX, scale, kOutline, 3, cv::LINE_AA);
  cv::putText(img, s, org, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

// 骨骼绘制主体（供"叠加"和"单独输出"两种模式复用）
void drawSkeletonBody(cv::Mat& img, const pose::PoseFrame& poses,
                      const std::vector<pose::PoseAngles>& angles, const DrawStyle& style,
                      const std::vector<std::vector<pose::RelCoord>>* relCoords = nullptr,
                      const std::vector<pose::GroundCalib>* calibs = nullptr) {
  for (size_t pi = 0; pi < poses.skeletons.size(); ++pi) {
    const pose::Skeleton& sk = poses.skeletons[pi];
    const pose::PoseAngles empty{};
    const pose::PoseAngles& ang = pi < angles.size() ? angles[pi] : empty;

    // ---- 连线 ----
    if (style.bones) {
      for (const pose::Bone& b : pose::kBones) {
        const pose::Vec2& pa = sk.joints2d[b.a];
        const pose::Vec2& pb = sk.joints2d[b.b];
        if (!pa.valid || !pb.valid) continue;
        cv::Scalar c = kTorso;
        if (isLeftJoint(b.a) || isLeftJoint(b.b)) c = kLeft;
        if (isRightJoint(b.a) || isRightJoint(b.b)) c = kRight;
        cv::line(img, cv::Point(static_cast<int>(pa.x), static_cast<int>(pa.y)),
                 cv::Point(static_cast<int>(pb.x), static_cast<int>(pb.y)), c,
                 style.boneThickness, cv::LINE_AA);
      }
    }

    // ---- 关节点 ----
    if (style.joints) {
      for (int i = 0; i < pose::kJointCount; ++i) {
        const pose::Vec2& p = sk.joints2d[i];
        if (!p.valid) continue;
        const float conf = sk.conf[i] <= 0.f ? 1.f : sk.conf[i];
        cv::Scalar c = conf >= 0.6f ? cv::Scalar(0, 255, 0)
                       : (conf >= 0.3f ? cv::Scalar(0, 190, 255) : cv::Scalar(0, 0, 255));
        const cv::Point pt(static_cast<int>(p.x), static_cast<int>(p.y));
        cv::circle(img, pt, style.jointRadius, c, -1, cv::LINE_AA);
        cv::circle(img, pt, style.jointRadius + 1, kOutline, 1, cv::LINE_AA);
      }
    }

    // ---- 关节角度数值 ----
    if (style.jointAngles) {
      for (int i = 0; i < pose::kJointCount; ++i) {
        const pose::Vec2& p = sk.joints2d[i];
        if (!p.valid) continue;
        pose::AngleId aid;
        if (!angleForJoint(i, aid)) continue;
        const pose::AngleValue& v = ang.get(aid);
        if (!v.valid) continue;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f", static_cast<double>(v.deg));
        textOutlined(img, buf,
                     cv::Point(static_cast<int>(p.x) + style.jointRadius + 4,
                               static_cast<int>(p.y) - style.jointRadius - 2),
                     0.5, kText);
      }
    }

    // ---- 相对坐标数值 (X, Y) ----
    if (style.jointCoords && relCoords && pi < relCoords->size()) {
      const auto& rc = (*relCoords)[pi];
      const float span = (calibs && pi < calibs->size()) ? (*calibs)[pi].spanPx : 0.f;
      for (int i = 0; i < pose::kJointCount; ++i) {
        if (!rc[i].valid) continue;
        const pose::Vec2& p = sk.joints2d[i];
        if (!p.valid) continue;
        const std::string txt = pose::formatCoord(rc[i], style.coordNormalized, span);
        textOutlined(img, txt,
                     cv::Point(static_cast<int>(p.x) + style.jointRadius + 3,
                               static_cast<int>(p.y) + style.jointRadius + 12),
                     0.42, cv::Scalar(200, 225, 245));
      }
    }

  }
}

}  // namespace

void ensureBgr(cv::Mat& m, int w, int h, const cv::Scalar& fill) {
  if (m.cols != w || m.rows != h || m.type() != CV_8UC3) {
    m.create(h, w, CV_8UC3);
  }
  m.setTo(fill);
}

// ---------------------------------------------------------------------------
//  人体坐标系视图：地面网格 + 坐标轴 + 骨架 + 每个关节 (X, Y)
//  坐标系：Y 向上、地面 = 0；X 向右、人体中心 = 0；单位毫米。
// ---------------------------------------------------------------------------
void drawCoordView(cv::Mat& canvas, const pose::PoseFrame& poses, const DrawStyle& style,
                   const cv::Scalar& background,
                   std::vector<pose::GroundCalib>* calibOut) {
  (void)background;
  if (canvas.empty()) return;

  // 每人一套参考系与坐标
  std::vector<pose::GroundCalib> calibs;
  std::vector<std::vector<pose::RelCoord>> coords;
  for (const auto& sk : poses.skeletons) {
    pose::GroundCalib c;
    coords.push_back(pose::computeRelativeCoords(sk, &c));
    calibs.push_back(c);
  }
  if (calibOut) *calibOut = calibs;

  const pose::GroundCalib* ref = nullptr;
  for (const auto& c : calibs) {
    if (c.valid) { ref = &c; break; }
  }

  if (ref && style.coordGrid) {
    const int originX = static_cast<int>(ref->centerXpx);
    const int originY = static_cast<int>(ref->groundYpx);
    const int minor = 50;    // 细网格间距（像素）
    const int major = 200;   // 粗网格间距（像素）

    // 地面基线（Y = 0）
    cv::line(canvas, cv::Point(0, originY + 1), cv::Point(canvas.cols, originY + 1),
             cv::Scalar(80, 84, 96), 1);
    cv::line(canvas, cv::Point(0, originY), cv::Point(canvas.cols, originY),
             cv::Scalar(200, 180, 90), 2);

    // 水平网格（Y 为常数）：向上画，粗线带刻度文字
    for (int yy = originY - minor; yy >= 0; yy -= minor) {
      const int rel = originY - yy;              // 相对地面的像素高度
      const bool isMajor = (rel % major == 0);
      cv::line(canvas, cv::Point(0, yy), cv::Point(canvas.cols, yy),
               isMajor ? cv::Scalar(72, 76, 90) : cv::Scalar(44, 46, 56), 1);
      if (isMajor) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "y=%d", rel);
        cv::putText(canvas, buf, cv::Point(8, yy - 5), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    cv::Scalar(118, 122, 138), 1, cv::LINE_AA);
      }
    }

    // 竖直网格（X 为常数）：中心线加亮
    for (int xx = originX; xx < canvas.cols; xx += minor) {
      const int rel = xx - originX;
      const bool isMajor = (rel % major == 0);
      cv::line(canvas, cv::Point(xx, 0), cv::Point(xx, originY),
               rel == 0 ? cv::Scalar(190, 120, 90)
                        : (isMajor ? cv::Scalar(68, 72, 84) : cv::Scalar(42, 44, 54)),
               rel == 0 ? 2 : 1);
    }
    for (int xx = originX - minor; xx >= 0; xx -= minor) {
      const int rel = originX - xx;
      const bool isMajor = (rel % major == 0);
      cv::line(canvas, cv::Point(xx, 0), cv::Point(xx, originY),
               isMajor ? cv::Scalar(68, 72, 84) : cv::Scalar(42, 44, 54), 1);
    }

    // 原点标记与说明
    cv::circle(canvas, cv::Point(originX, originY), 4, cv::Scalar(120, 200, 255), -1,
               cv::LINE_AA);
    cv::putText(canvas, "O (ground x body-center)", cv::Point(originX + 8, originY - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(150, 200, 240), 1, cv::LINE_AA);
    char info[160];
    std::snprintf(info, sizeof(info),
                  "relative coords in pixels | +X right, +Y up | body height %.0f px",
                  static_cast<double>(ref->spanPx));
    cv::putText(canvas, info, cv::Point(8, canvas.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(130, 134, 150), 1, cv::LINE_AA);
  }

  // ---- 骨架本体（坐标视图不重复标注角度）----
  DrawStyle body = style;
  body.jointCoords = false;
  body.jointAngles = false;
  drawSkeletonBody(canvas, poses, {}, body, nullptr, nullptr);

  // ---- 每个关节标注相对坐标 (X, Y) ----
  for (size_t i = 0; i < poses.skeletons.size() && i < coords.size(); ++i) {
    const pose::Skeleton& sk = poses.skeletons[i];
    const auto& rc = coords[i];
    const float span = (i < calibs.size()) ? calibs[i].spanPx : 0.f;
    for (int j = 0; j < pose::kJointCount; ++j) {
      if (!rc[j].valid) continue;
      const pose::Vec2& p = sk.joints2d[j];
      if (!p.valid) continue;

      // 躯干与右侧肢体标在右方，左侧肢体标在左方，减少文字互相压盖
      const bool rightSide = (j == static_cast<int>(pose::JointId::Head) ||
                              j == static_cast<int>(pose::JointId::Neck) ||
                              j == static_cast<int>(pose::JointId::Spine) ||
                              j == static_cast<int>(pose::JointId::ShoulderR) ||
                              j == static_cast<int>(pose::JointId::ElbowR) ||
                              j == static_cast<int>(pose::JointId::WristR) ||
                              j == static_cast<int>(pose::JointId::HipR) ||
                              j == static_cast<int>(pose::JointId::KneeR) ||
                              j == static_cast<int>(pose::JointId::AnkleR) ||
                              j == static_cast<int>(pose::JointId::FootR));
      const std::string txt = pose::formatCoord(rc[j], style.coordNormalized, span);
      const int tw = static_cast<int>(txt.size()) * 8;
      int tx = rightSide ? static_cast<int>(p.x) + style.jointRadius + 4
                         : static_cast<int>(p.x) - style.jointRadius - 4 - tw;
      // 左右两侧文字的竖直位置错开，降低相邻关节标签互相压盖的概率
      const int vShift = rightSide ? -3 : 11;
      int ty = static_cast<int>(p.y) + style.jointRadius + vShift;
      tx = std::max(2, std::min(tx, canvas.cols - tw - 2));
      ty = std::max(12, std::min(ty, canvas.rows - 4));
      textOutlined(canvas, txt, cv::Point(tx, ty), 0.42, cv::Scalar(215, 225, 240));
    }
  }
}
void drawSkeletonOverlay(cv::Mat& bgr, const pose::PoseFrame& poses,
                         const std::vector<pose::PoseAngles>& angles,
                         const DrawStyle& style) {
  if (bgr.empty()) return;

  // 需要标注相对坐标时，先算好参考系与坐标
  std::vector<std::vector<pose::RelCoord>> relCoords;
  std::vector<pose::GroundCalib> calibs;
  if (style.jointCoords) {
    relCoords.reserve(poses.skeletons.size());
    calibs.reserve(poses.skeletons.size());
    for (const auto& sk : poses.skeletons) {
      pose::GroundCalib c;
      relCoords.push_back(pose::computeRelativeCoords(sk, &c));
      calibs.push_back(c);
    }
  }

  if (style.alpha >= 0.999f) {
    drawSkeletonBody(bgr, poses, angles, style, &relCoords, &calibs);
    return;
  }
  // 半透明：先画到独立图层再按 alpha 混合
  cv::Mat layer = cv::Mat::zeros(bgr.size(), CV_8UC3);
  drawSkeletonBody(layer, poses, angles, style, &relCoords, &calibs);
  cv::addWeighted(bgr, 1.0, layer, std::max(0.f, style.alpha), 0.0, bgr);
}

void drawSkeletonOnly(cv::Mat& canvas, const pose::PoseFrame& poses,
                      const std::vector<pose::PoseAngles>& angles, const DrawStyle& style,
                      const cv::Scalar& background) {
  DrawStyle s = style;
  s.alpha = 1.f;
  drawSkeletonBody(canvas, poses, angles, s);
  (void)background;
}

void colorizeDepth(const std::vector<uint16_t>& mm, int w, int h, DepthColorMap map,
                   int minMm, int maxMm, cv::Mat& bgrOut) {
  if (w <= 0 || h <= 0 || mm.size() < static_cast<size_t>(w) * h) {
    bgrOut = cv::Mat();
    return;
  }
  cv::Mat raw(h, w, CV_16UC1, const_cast<uint16_t*>(mm.data()));
  cv::Mat norm8(h, w, CV_8UC1);

  const int lo = std::max(1, minMm);
  const int hi = std::max(lo + 1, maxMm);
  const float scale = 255.0f / static_cast<float>(hi - lo);

  for (int y = 0; y < h; ++y) {
    const uint16_t* src = raw.ptr<uint16_t>(y);
    uint8_t* dst = norm8.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      const int v = src[x];
      if (v == 0 || v < lo) {
        dst[x] = 0;  // 无效/过近 -> 黑
      } else if (v >= hi) {
        dst[x] = 255;
      } else {
        dst[x] = static_cast<uint8_t>((v - lo) * scale);
      }
    }
  }

  switch (map) {
    case DepthColorMap::Turbo:
      cv::applyColorMap(norm8, bgrOut, cv::COLORMAP_TURBO);
      break;
    case DepthColorMap::Jet:
      cv::applyColorMap(norm8, bgrOut, cv::COLORMAP_JET);
      break;
    case DepthColorMap::Viridis:
      cv::applyColorMap(norm8, bgrOut, cv::COLORMAP_VIRIDIS);
      break;
    case DepthColorMap::Gray:
    default:
      cv::cvtColor(norm8, bgrOut, cv::COLOR_GRAY2BGR);
      break;
  }

  // 无效像素压暗，方便一眼看出空洞
  for (int y = 0; y < h; ++y) {
    const uint16_t* src = raw.ptr<uint16_t>(y);
    cv::Vec3b* dst = bgrOut.ptr<cv::Vec3b>(y);
    for (int x = 0; x < w; ++x) {
      if (src[x] == 0) dst[x] = cv::Vec3b(18, 18, 22);
    }
  }
}

}  // namespace ui
