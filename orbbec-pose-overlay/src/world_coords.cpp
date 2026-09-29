// ============================================================================
//  pose/world_coords.cpp  —  骨骼点 -> 人体相对坐标系（纯平移，无缩放）
//
//    X = 关节像素 x - 人体中心像素 x
//    Y = 地面像素 y - 关节像素 y
// ============================================================================
#include "pose/world_coords.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace pose {
namespace {

inline int idx(JointId id) { return static_cast<int>(id); }

// 取一个关节的像素 y，无效时返回 false
inline bool hasY(const Skeleton& sk, JointId id, float* out) {
  const Vec2& p = sk.joints2d[idx(id)];
  if (!p.valid) return false;
  if (out) *out = p.y;
  return true;
}

}  // namespace

GroundCalib calibrateRelative(const Skeleton& sk) {
  GroundCalib c;

  // ---- 地面：身体最低点（左右踝的最大 y；脚部关节等于踝，故用踝即可）----
  float ankleL = 0.f, ankleR = 0.f;
  const bool okL = hasY(sk, JointId::AnkleL, &ankleL);
  const bool okR = hasY(sk, JointId::AnkleR, &ankleR);
  if (okL && okR) {
    c.groundYpx = std::max(ankleL, ankleR);
  } else if (okL) {
    c.groundYpx = ankleL;
  } else if (okR) {
    c.groundYpx = ankleR;
  } else {
    // 下半身不可见：退化为"脚部关节"或直接放弃（保持 valid=false，
    // 上层会显示"--"，避免给出一个靠猜的原点）
    float footL = 0.f, footR = 0.f;
    const bool fL = hasY(sk, JointId::FootL, &footL);
    const bool fR = hasY(sk, JointId::FootR, &footR);
    if (fL && fR) {
      c.groundYpx = std::max(footL, footR);
    } else if (fL) {
      c.groundYpx = footL;
    } else if (fR) {
      c.groundYpx = footR;
    } else {
      return c;
    }
  }

  // ---- 人体中心：左右髋中点 -> 左右肩中点 -> 头 ----
  const Vec2& hipL = sk.joints2d[idx(JointId::HipL)];
  const Vec2& hipR = sk.joints2d[idx(JointId::HipR)];
  const Vec2& shL = sk.joints2d[idx(JointId::ShoulderL)];
  const Vec2& shR = sk.joints2d[idx(JointId::ShoulderR)];
  const Vec2& head = sk.joints2d[idx(JointId::Head)];
  if (hipL.valid && hipR.valid) {
    c.centerXpx = (hipL.x + hipR.x) * 0.5f;
  } else if (shL.valid && shR.valid) {
    c.centerXpx = (shL.x + shR.x) * 0.5f;
  } else if (head.valid) {
    c.centerXpx = head.x;
  } else {
    return c;
  }

  // ---- 身体像素高度（仅作界面参考：地面到头/颈）----
  float headY = 0.f;
  if (hasY(sk, JointId::Head, &headY)) {
    c.spanPx = std::fabs(c.groundYpx - headY);
  } else {
    float neckY = 0.f;
    if (hasY(sk, JointId::Neck, &neckY)) c.spanPx = std::fabs(c.groundYpx - neckY);
  }

  c.valid = true;
  return c;
}

std::vector<RelCoord> toRelativeCoords(const Skeleton& sk, const GroundCalib& calib) {
  std::vector<RelCoord> out(kJointCount);
  if (!calib.valid) return out;

  for (int i = 0; i < kJointCount; ++i) {
    const Vec2& p = sk.joints2d[i];
    if (!p.valid) continue;
    RelCoord r;
    r.x = p.x - calib.centerXpx;
    r.y = calib.groundYpx - p.y;
    r.valid = true;
    out[i] = r;
  }
  return out;
}

std::vector<RelCoord> computeRelativeCoords(const Skeleton& sk, GroundCalib* calibOut) {
  const GroundCalib c = calibrateRelative(sk);
  if (calibOut) *calibOut = c;
  return toRelativeCoords(sk, c);
}

std::string formatCoord(const RelCoord& c, bool normalized, float bodyHeightPx) {
  if (!c.valid) return "--";
  char buf[64];
  if (normalized && bodyHeightPx > 1.f) {
    // 按身高折算成比例（便于跨距离比较）；地面=0，头顶≈1
    std::snprintf(buf, sizeof(buf), "(%.2f, %.2f)",
                  static_cast<double>(c.x / bodyHeightPx),
                  static_cast<double>(c.y / bodyHeightPx));
  } else {
    std::snprintf(buf, sizeof(buf), "(%.0f, %.0f)", static_cast<double>(c.x),
                  static_cast<double>(c.y));
  }
  return buf;
}

}  // namespace pose
