// ============================================================================
//  src/renderer.cpp  —  在彩色图上叠加骨骼、角度、HUD
//
//  绘制内容：
//    1. 骨架连线：左侧青色、右侧绿色、躯干黄色；低置信度关节连线变暗
//    2. 关节点：绿色（高置信）/ 橙色（中）/ 红色（低），并标注关节名与角度
//    3. 附加角度：肩外展、髋外展（相对躯干竖直轴的夹角），用于姿态评估
//    4. HUD：FPS、数据源、骨骼后端、每个人的完整角度表（超阈值标红）
//    5. 深度小窗（可选）
// ============================================================================
#include "pose/renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "pose/geometry.hpp"

namespace pose {
namespace {

const cv::Scalar kColorLeft(255, 220, 0);     // BGR 青
const cv::Scalar kColorRight(0, 220, 60);     // BGR 绿
const cv::Scalar kColorTorso(0, 200, 255);    // BGR 黄
const cv::Scalar kColorWarn(0, 0, 255);       // 红
const cv::Scalar kColorText(255, 255, 255);
const cv::Scalar kColorDim(160, 160, 160);
const cv::Scalar kColorJointGood(0, 255, 0);
const cv::Scalar kColorJointMid(0, 200, 255);
const cv::Scalar kColorJointBad(0, 0, 255);

// 面板尺寸
constexpr int kPanelW = 300;
constexpr int kLineH = 18;

inline bool isLeftBone(int joint) {
  return joint == static_cast<int>(JointId::ShoulderL) ||
         joint == static_cast<int>(JointId::ElbowL) ||
         joint == static_cast<int>(JointId::WristL) ||
         joint == static_cast<int>(JointId::HipL) ||
         joint == static_cast<int>(JointId::KneeL) ||
         joint == static_cast<int>(JointId::AnkleL) ||
         joint == static_cast<int>(JointId::FootL);
}

inline bool isRightBone(int joint) {
  return joint == static_cast<int>(JointId::ShoulderR) ||
         joint == static_cast<int>(JointId::ElbowR) ||
         joint == static_cast<int>(JointId::WristR) ||
         joint == static_cast<int>(JointId::HipR) ||
         joint == static_cast<int>(JointId::KneeR) ||
         joint == static_cast<int>(JointId::AnkleR) ||
         joint == static_cast<int>(JointId::FootR);
}

inline bool valid(const Vec2& p) { return p.valid; }

// 关节名到角度列表：用于在关节点旁边标注对应角度
AngleId angleForJoint(int joint, bool* ok) {
  switch (static_cast<JointId>(joint)) {
    case JointId::ElbowL: *ok = true; return AngleId::ElbowL;
    case JointId::ElbowR: *ok = true; return AngleId::ElbowR;
    case JointId::ShoulderL: *ok = true; return AngleId::ShoulderL;
    case JointId::ShoulderR: *ok = true; return AngleId::ShoulderR;
    case JointId::HipL: *ok = true; return AngleId::HipL;
    case JointId::HipR: *ok = true; return AngleId::HipR;
    case JointId::KneeL: *ok = true; return AngleId::KneeL;
    case JointId::KneeR: *ok = true; return AngleId::KneeR;
    case JointId::Neck: *ok = true; return AngleId::Neck;
    default: *ok = false; return AngleId::ElbowL;
  }
}

std::string fmt1(float v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(v));
  return buf;
}

// 相对躯干竖直轴的横向外展/前伸角：
// 把「三点角（如 肘-肩-髋）」进一步拆成"肢体相对竖直方向偏离多少度"，
// 比单纯的三点角更贴近康复/健身评估习惯（例如"手臂抬离躯干 60°"）。
// 返回 -1 表示无法计算。
float lateralAbductionDeg(const Skeleton& sk, JointId moving, JointId pivot,
                          bool prefer3d) {
  if (prefer3d && sk.joints3d[static_cast<int>(moving)].valid &&
      sk.joints3d[static_cast<int>(pivot)].valid) {
    const Vec3 v =
        geom::sub(sk.joints3d[static_cast<int>(moving)], sk.joints3d[static_cast<int>(pivot)]);
    const float len = geom::length(v);
    if (len > 1e-3f) {
      const float vert = std::fabs(v.y);
      const float horiz = std::sqrt(v.x * v.x + v.z * v.z);
      return std::atan2(horiz, vert) * geom::kRad2Deg;
    }
    return -1.f;
  }
  const Vec2& m = sk.joints2d[static_cast<int>(moving)];
  const Vec2& p = sk.joints2d[static_cast<int>(pivot)];
  if (!m.valid || !p.valid) return -1.f;
  const Vec2 v2 = geom::sub2(m, p);
  const float n = geom::length2(v2);
  if (n < 1e-3f) return -1.f;
  return std::atan2(std::fabs(v2.x), std::fabs(v2.y)) * geom::kRad2Deg;
}

}  // namespace

bool OverlayRenderer::isWarn(AngleId id, float deg) const {
  if (!cfg_.enableThreshold) return false;
  switch (id) {
    case AngleId::ElbowL:
    case AngleId::ElbowR:
      return deg > cfg_.elbowStraightenThreshold;
    case AngleId::KneeL:
    case AngleId::KneeR:
      return deg > cfg_.kneeStraightenThreshold;
    case AngleId::Torso:
      return deg > cfg_.torsoLeanWarnDeg;
    case AngleId::Neck:
      return std::fabs(180.f - deg) > cfg_.neckTiltWarnDeg;
    default:
      return false;
  }
}

void OverlayRenderer::putText(cv::Mat& img, const std::string& s, cv::Point org,
                              double scale, cv::Scalar color, int thickness) const {
  cv::putText(img, s, org, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

void OverlayRenderer::drawSkeleton(cv::Mat& img, const Skeleton& sk,
                                   const PoseAngles& angles) const {
  (void)angles;
  for (const Bone& b : kBones) {
    const Vec2& pa = sk.joints2d[b.a];
    const Vec2& pb = sk.joints2d[b.b];
    if (!valid(pa) || !valid(pb)) continue;

    cv::Scalar color = kColorTorso;
    if (isLeftBone(b.a) || isLeftBone(b.b)) color = kColorLeft;
    if (isRightBone(b.a) || isRightBone(b.b)) color = kColorRight;

    // 置信度低 -> 画暗一点，避免"看起来检测到了其实没检测到"
    const float c = std::min(sk.conf[b.a] <= 0.f ? 1.f : sk.conf[b.a],
                             sk.conf[b.b] <= 0.f ? 1.f : sk.conf[b.b]);
    if (c < cfg_.minJointConfidence) color = kColorDim;

    cv::line(img, cv::Point(static_cast<int>(pa.x), static_cast<int>(pa.y)),
             cv::Point(static_cast<int>(pb.x), static_cast<int>(pb.y)), color,
             cfg_.boneThickness, cv::LINE_AA);
  }
}

void OverlayRenderer::drawJoints(cv::Mat& img, const Skeleton& sk) const {
  for (int i = 0; i < kJointCount; ++i) {
    const Vec2& p = sk.joints2d[i];
    if (!valid(p)) continue;
    const float c = sk.conf[i] <= 0.f ? 1.f : sk.conf[i];
    cv::Scalar color = c >= 0.6f ? kColorJointGood
                       : (c >= cfg_.minJointConfidence ? kColorJointMid : kColorJointBad);
    const int r = cfg_.jointRadius;
    cv::circle(img, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), r, color, -1,
               cv::LINE_AA);
    // 外圈描边，深色背景下更清晰
    cv::circle(img, cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), r + 1,
               cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
  }
}

void OverlayRenderer::drawAngleLabels(cv::Mat& img, const Skeleton& sk,
                                      const PoseAngles& angles) const {
  for (int i = 0; i < kJointCount; ++i) {
    const Vec2& p = sk.joints2d[i];
    if (!valid(p)) continue;
    bool ok = false;
    const AngleId aid = angleForJoint(i, &ok);
    if (!ok) continue;
    const AngleValue& v = angles.get(aid);
    if (!v.valid) continue;

    const std::string text = fmt1(v.deg);
    const cv::Scalar color = isWarn(aid, v.deg) ? kColorWarn : kColorText;
    const cv::Point org(static_cast<int>(p.x) + cfg_.jointRadius + 3,
                        static_cast<int>(p.y) - cfg_.jointRadius - 2);
    // 黑色描边保证在任意背景上可读
    putText(img, text, org, cfg_.fontScale, cv::Scalar(0, 0, 0), 3);
    putText(img, text, org, cfg_.fontScale, color, 1);
  }
}

void OverlayRenderer::drawDepthPeek(cv::Mat& img, const DepthFrame& depth) const {
  if (depth.empty()) return;
  cv::Mat viz;
  if (!depth.alignedBgr.empty()) {
    cv::Mat tmp(depth.height, depth.width, CV_8UC3,
                const_cast<uint8_t*>(depth.alignedBgr.data()));
    tmp.copyTo(viz);
  } else {
    // uint16 深度 -> 伪彩色
    cv::Mat raw(depth.height, depth.width, CV_16UC1,
                const_cast<uint16_t*>(depth.mm.data()));
    cv::Mat norm;
    raw.convertTo(norm, CV_8UC1, 255.0 / 5000.0);  // 假设 0..5000mm
    cv::applyColorMap(norm, viz, cv::COLORMAP_JET);
  }
  const int targetW = 240;
  const int targetH = std::max(1, static_cast<int>(viz.rows * (targetW / static_cast<double>(viz.cols))));
  cv::Mat small;
  cv::resize(viz, small, cv::Size(targetW, targetH));

  const int x0 = 10;
  const int y0 = img.rows - targetH - 10;
  if (x0 + small.cols >= img.cols || y0 < 0) return;
  cv::Mat roi = img(cv::Rect(x0, y0, small.cols, small.rows));
  cv::addWeighted(roi, 0.25, small, 0.75, 0.0, roi);
  cv::rectangle(img, cv::Rect(x0, y0, small.cols, small.rows), cv::Scalar(200, 200, 200), 1);
  putText(img, "depth", cv::Point(x0 + 4, y0 + 16), 0.45, cv::Scalar(255, 255, 255), 1);
}

void OverlayRenderer::drawHud(cv::Mat& img, const PoseFrame& poses,
                              const AngleList& angles, double fps,
                              const std::string& sourceName,
                              const std::string& backendName,
                              const std::string& statusLine) const {
  // 面板高度：标题行 + 每个人的块
  const int perPerson = 3 + kAngleCount;  // 姓名行 + 10 项角度 + 2 项附加角度 + 间距
  const int rows = 4 + static_cast<int>(poses.skeletons.size()) * perPerson + 2;
  const int panelH = std::min(img.rows - 20, rows * kLineH + 12);
  const cv::Rect panel(10, 10, std::min(kPanelW, img.cols - 20), panelH);

  if (panel.width > 10 && panel.height > 10) {
    cv::Mat roi = img(panel);
    cv::Mat black = cv::Mat::zeros(roi.size(), roi.type());
    cv::addWeighted(roi, 0.35, black, 0.65, 0.0, roi);
    cv::rectangle(img, panel, cv::Scalar(180, 180, 180), 1);
  }

  int y = panel.y + kLineH;
  const int x = panel.x + 8;
  char buf[256];

  if (cfg_.drawFps) {
    std::snprintf(buf, sizeof(buf), "FPS %.1f   people %d", fps,
                  static_cast<int>(poses.skeletons.size()));
    putText(img, buf, cv::Point(x, y), 0.5, cv::Scalar(0, 255, 255), 1);
    y += kLineH;
  }
  std::snprintf(buf, sizeof(buf), "src: %s | pose: %s", sourceName.c_str(),
                backendName.c_str());
  putText(img, buf, cv::Point(x, y), 0.45, kColorText, 1);
  y += kLineH;

  if (!statusLine.empty()) {
    putText(img, statusLine.substr(0, 40), cv::Point(x, y), 0.42, kColorDim, 1);
    y += kLineH;
  }
  y += 4;

  for (size_t pi = 0; pi < poses.skeletons.size(); ++pi) {
    const Skeleton& sk = poses.skeletons[pi];
    const PoseAngles emptyAngles{};
    const PoseAngles& ang = pi < angles.size() ? angles[pi] : emptyAngles;

    std::snprintf(buf, sizeof(buf), "ID %d  conf %.2f", sk.id,
                  static_cast<double>(sk.globalConfidence));
    putText(img, buf, cv::Point(x, y), 0.5, cv::Scalar(0, 255, 255), 1);
    y += kLineH;

    if (y > panel.y + panel.height - 4) break;

    for (int ai = 0; ai < kAngleCount; ++ai) {
      const AngleId id = static_cast<AngleId>(ai);
      const AngleValue& v = ang.values[ai];
      const char* nm = cfg_.showChinese ? angleNameZh(id) : angleName(id);
      if (v.valid) {
        std::snprintf(buf, sizeof(buf), "  %-10s %6.1f  %s", nm,
                      static_cast<double>(v.deg), v.from3d ? "3D" : "2D");
      } else {
        std::snprintf(buf, sizeof(buf), "  %-10s   --   ", nm);
      }
      const cv::Scalar color =
          (v.valid && isWarn(id, v.deg)) ? kColorWarn : kColorText;
      putText(img, buf, cv::Point(x, y), 0.42, color, 1);
      y += kLineH;
      if (y > panel.y + panel.height - 4) break;
    }

    // ---- 附加角度：相对躯干竖直轴的横向外展/前伸 ----
    if (y <= panel.y + panel.height - 4) {
      const bool p3 = cfg_.anglesFrom3d;
      const float armL = lateralAbductionDeg(sk, JointId::WristL, JointId::ShoulderL, p3);
      const float armR = lateralAbductionDeg(sk, JointId::WristR, JointId::ShoulderR, p3);
      const float legL = lateralAbductionDeg(sk, JointId::KneeL, JointId::HipL, p3);
      const float legR = lateralAbductionDeg(sk, JointId::KneeR, JointId::HipR, p3);
      const std::string sL = armL >= 0.f ? fmt1(armL) : "--";
      const std::string sR = armR >= 0.f ? fmt1(armR) : "--";
      const std::string lL = legL >= 0.f ? fmt1(legL) : "--";
      const std::string lR = legR >= 0.f ? fmt1(legR) : "--";
      char ebuf[192];
      std::snprintf(ebuf, sizeof(ebuf), "  arm.L/R %s/%s  leg.L/R %s/%s", sL.c_str(),
                    sR.c_str(), lL.c_str(), lR.c_str());
      putText(img, ebuf, cv::Point(x, y), 0.40, kColorDim, 1);
      y += kLineH;

      char tbuf[128];
      const AngleValue& tv = ang.get(AngleId::Torso);
      const std::string ts = tv.valid ? fmt1(tv.deg) : "--";
      std::snprintf(tbuf, sizeof(tbuf), "  torso lean %s deg", ts.c_str());
      putText(img, tbuf, cv::Point(x, y), 0.40, kColorDim, 1);
      y += kLineH;
    }
    y += 4;
  }
}

void OverlayRenderer::draw(cv::Mat& bgr, const PoseFrame& poses,
                           const AngleList& angles, const DepthFrame* depth, double fps,
                           const std::string& sourceName, const std::string& backendName,
                           const std::string& statusLine) {
  if (bgr.empty()) return;

  for (size_t i = 0; i < poses.skeletons.size(); ++i) {
    const Skeleton& sk = poses.skeletons[i];
    const PoseAngles emptyAngles{};
    const PoseAngles& ang = i < angles.size() ? angles[i] : emptyAngles;

    if (cfg_.drawSkeleton) drawSkeleton(bgr, sk, ang);
    drawJoints(bgr, sk);
    if (cfg_.drawJointAngles) drawAngleLabels(bgr, sk, ang);

    // 头顶画 ID，方便多人区分。
    // 若头部落在左上角 HUD 面板区域内，则把标签移到头下方，避免被面板遮住。
    const Vec2& head = sk.joints2d[static_cast<int>(JointId::Head)];
    if (valid(head)) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "ID %d", sk.id);
      int lx = static_cast<int>(head.x) - 20;
      int ly = static_cast<int>(head.y) - 18;
      if (cfg_.drawHudTable) {
        const cv::Rect panel(10, 10, std::min(kPanelW, bgr.cols - 20), 200);
        if (panel.contains(cv::Point(static_cast<int>(head.x), static_cast<int>(head.y)))) {
          ly = static_cast<int>(head.y) + cfg_.jointRadius + 20;
          lx = static_cast<int>(head.x) + cfg_.jointRadius + 4;
        }
      }
      lx = std::max(2, std::min(lx, bgr.cols - 70));
      ly = std::max(16, std::min(ly, bgr.rows - 6));
      putText(bgr, buf, cv::Point(lx, ly), 0.6, cv::Scalar(0, 0, 0), 3);
      putText(bgr, buf, cv::Point(lx, ly), 0.6, cv::Scalar(0, 255, 255), 1);
    }
  }

  if (cfg_.drawDepthPeek && depth) drawDepthPeek(bgr, *depth);
  if (cfg_.drawHudTable) {
    drawHud(bgr, poses, angles, fps, sourceName, backendName, statusLine);
  } else {
    // 即使不画表格，也把 FPS 显示出来
    char buf[128];
    std::snprintf(buf, sizeof(buf), "FPS %.1f  people %d", fps,
                  static_cast<int>(poses.skeletons.size()));
    putText(bgr, buf, cv::Point(12, 26), 0.6, cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace pose
