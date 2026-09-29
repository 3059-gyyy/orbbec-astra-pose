// ============================================================================
//  pose/geometry.hpp  —  与 SDK 无关的纯几何工具
// ============================================================================
#pragma once

#include <cmath>

#include "pose/types.hpp"

namespace pose {
namespace geom {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRad2Deg = 180.f / kPi;

inline float dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 sub(const Vec3& a, const Vec3& b) {
  return Vec3{a.x - b.x, a.y - b.y, a.z - b.z, a.valid && b.valid};
}
inline Vec3 add(const Vec3& a, const Vec3& b) {
  return Vec3{a.x + b.x, a.y + b.y, a.z + b.z, a.valid && b.valid};
}
inline Vec3 mul(const Vec3& a, float s) {
  return Vec3{a.x * s, a.y * s, a.z * s, a.valid};
}
inline float length(const Vec3& a) {
  return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
inline Vec3 normalize(const Vec3& a) {
  const float n = length(a);
  if (n <= 1e-6f) return Vec3{0.f, 0.f, 0.f, false};
  return Vec3{a.x / n, a.y / n, a.z / n, a.valid};
}

inline float dot2(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline Vec2 sub2(const Vec2& a, const Vec2& b) {
  return Vec2{a.x - b.x, a.y - b.y, a.valid && b.valid};
}
inline float length2(const Vec2& a) { return std::sqrt(a.x * a.x + a.y * a.y); }

// 三点夹角（顶点在 b）。返回 0..180 度；任何一点无效则返回 valid=false。
inline float angleDeg3(const Vec3& a, const Vec3& b, const Vec3& c, bool* ok) {
  if (!a.valid || !b.valid || !c.valid) {
    if (ok) *ok = false;
    return 0.f;
  }
  const Vec3 ba = sub(a, b);
  const Vec3 bc = sub(c, b);
  const float nba = length(ba);
  const float nbc = length(bc);
  if (nba <= 1e-6f || nbc <= 1e-6f) {
    if (ok) *ok = false;
    return 0.f;
  }
  float cosv = dot(ba, bc) / (nba * nbc);
  cosv = cosv > 1.f ? 1.f : (cosv < -1.f ? -1.f : cosv);
  if (ok) *ok = true;
  return std::acos(cosv) * kRad2Deg;
}

inline float angleDeg3(const Vec2& a, const Vec2& b, const Vec2& c, bool* ok) {
  if (!a.valid || !b.valid || !c.valid) {
    if (ok) *ok = false;
    return 0.f;
  }
  const Vec2 ba = sub2(a, b);
  const Vec2 bc = sub2(c, b);
  const float nba = length2(ba);
  const float nbc = length2(bc);
  if (nba <= 1e-6f || nbc <= 1e-6f) {
    if (ok) *ok = false;
    return 0.f;
  }
  float cosv = dot2(ba, bc) / (nba * nbc);
  cosv = cosv > 1.f ? 1.f : (cosv < -1.f ? -1.f : cosv);
  if (ok) *ok = true;
  return std::acos(cosv) * kRad2Deg;
}

// 向量与参考方向（down = +y 的图像/相机坐标）的夹角，用于躯干前倾测量。
inline float angleToDownDeg(const Vec2& a, const Vec2& b, bool* ok) {
  if (!a.valid || !b.valid) {
    if (ok) *ok = false;
    return 0.f;
  }
  const Vec2 v = sub2(b, a);          // a -> b
  const float n = length2(v);
  if (n <= 1e-6f) {
    if (ok) *ok = false;
    return 0.f;
  }
  float cosv = v.y / n;               // 与 (0,1) 的点积
  cosv = cosv > 1.f ? 1.f : (cosv < -1.f ? -1.f : cosv);
  if (ok) *ok = true;
  return std::acos(cosv) * kRad2Deg;
}

inline Vec2 midpoint(const Vec2& a, const Vec2& b) {
  return Vec2{(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, a.valid && b.valid};
}
inline Vec3 midpoint(const Vec3& a, const Vec3& b) {
  return Vec3{(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, (a.z + b.z) * 0.5f,
              a.valid && b.valid};
}

// 指数平滑（跨帧稳定角度读数，避免数字跳动）
inline float smooth(float previous, float current, float alpha) {
  if (alpha <= 0.f) return current;
  if (alpha >= 1.f) return previous;
  return previous * alpha + current * (1.f - alpha);
}

}  // namespace geom
}  // namespace pose
