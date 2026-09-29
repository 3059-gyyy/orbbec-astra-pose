// ============================================================================
//  pose/angles.cpp  —  姿态角度计算
// ============================================================================
#include "pose/angles.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "pose/geometry.hpp"

namespace pose {
namespace {

inline int idx(JointId id) { return static_cast<int>(id); }

// 关节置信度是否达标（没填置信度的数据源视为 1.0，即只要 valid 就用）
inline bool confident(const Skeleton& sk, JointId id, float minConf) {
  const float c = sk.conf[idx(id)];
  return c <= 0.f ? true : (c >= minConf);
}

inline bool usable(const Skeleton& sk, JointId id, float minConf) {
  return sk.joints3d[idx(id)].valid || sk.joints2d[idx(id)].valid
             ? confident(sk, id, minConf)
             : false;
}

struct Tri {
  bool ok = false;
  float deg = 0.f;
  bool from3d = false;
};

// 计算 a-b-c 三点角：优先 3D，失败退回 2D。
Tri tri(const Skeleton& sk, JointId a, JointId b, JointId c, bool prefer3d,
        float minConf) {
  Tri r;
  const bool all3d = sk.joints3d[idx(a)].valid && sk.joints3d[idx(b)].valid &&
                     sk.joints3d[idx(c)].valid &&
                     confident(sk, a, minConf) && confident(sk, b, minConf) &&
                     confident(sk, c, minConf);
  const bool all2d = sk.joints2d[idx(a)].valid && sk.joints2d[idx(b)].valid &&
                     sk.joints2d[idx(c)].valid &&
                     confident(sk, a, minConf) && confident(sk, b, minConf) &&
                     confident(sk, c, minConf);

  if (prefer3d && all3d) {
    bool ok = false;
    r.deg = geom::angleDeg3(sk.joints3d[idx(a)], sk.joints3d[idx(b)],
                            sk.joints3d[idx(c)], &ok);
    r.ok = ok;
    r.from3d = true;
    if (ok) return r;
  }
  if (all2d) {
    bool ok = false;
    r.deg = geom::angleDeg3(sk.joints2d[idx(a)], sk.joints2d[idx(b)],
                            sk.joints2d[idx(c)], &ok);
    r.ok = ok;
    r.from3d = false;
    if (ok) return r;
  }
  if (!prefer3d && all3d) {
    bool ok = false;
    r.deg = geom::angleDeg3(sk.joints3d[idx(a)], sk.joints3d[idx(b)],
                            sk.joints3d[idx(c)], &ok);
    r.ok = ok;
    r.from3d = true;
  }
  return r;
}

// 缺失关节的补全：脚 = 踝 + (踝-膝) 方向延伸 25%（Nuitrack 有时不给脚部）
void fillMissingJoints(Skeleton& sk) {
  const auto extrapolate = [&sk](JointId tip, JointId from, JointId to) {
    const int it = idx(tip), ifr = idx(from), ito = idx(to);
    if (sk.joints3d[it].valid || !sk.joints3d[ifr].valid || !sk.joints3d[ito].valid) return;
    const Vec3 dir = geom::sub(sk.joints3d[ifr], sk.joints3d[ito]);  // 膝->踝
    Vec3 p = geom::add(sk.joints3d[ifr], geom::mul(dir, 0.25f));
    p.valid = true;
    sk.joints3d[it] = p;
    if (sk.conf[it] <= 0.f) sk.conf[it] = sk.conf[ifr] * 0.8f;
  };
  extrapolate(JointId::FootL, JointId::AnkleL, JointId::KneeL);
  extrapolate(JointId::FootR, JointId::AnkleR, JointId::KneeR);
}

}  // namespace

void computeAngles(const Skeleton& sk, const AppConfig& cfg, PoseAngles& out) {
  const float mc = cfg.minJointConfidence;
  const bool prefer3d = cfg.anglesFrom3d;

  const auto set = [&out](AngleId id, const Tri& t) {
    AngleValue& v = out.get(id);
    v.deg = t.deg;
    v.valid = t.ok;
    v.from3d = t.from3d;
  };

  set(AngleId::ElbowL, tri(sk, JointId::ShoulderL, JointId::ElbowL, JointId::WristL, prefer3d, mc));
  set(AngleId::ElbowR, tri(sk, JointId::ShoulderR, JointId::ElbowR, JointId::WristR, prefer3d, mc));
  set(AngleId::ShoulderL, tri(sk, JointId::ElbowL, JointId::ShoulderL, JointId::HipL, prefer3d, mc));
  set(AngleId::ShoulderR, tri(sk, JointId::ElbowR, JointId::ShoulderR, JointId::HipR, prefer3d, mc));
  set(AngleId::HipL, tri(sk, JointId::ShoulderL, JointId::HipL, JointId::KneeL, prefer3d, mc));
  set(AngleId::HipR, tri(sk, JointId::ShoulderR, JointId::HipR, JointId::KneeR, prefer3d, mc));
  set(AngleId::KneeL, tri(sk, JointId::HipL, JointId::KneeL, JointId::AnkleL, prefer3d, mc));
  set(AngleId::KneeR, tri(sk, JointId::HipR, JointId::KneeR, JointId::AnkleR, prefer3d, mc));
  set(AngleId::Neck, tri(sk, JointId::Head, JointId::Neck, JointId::Spine, prefer3d, mc));

  // ---- 躯干倾斜：髋中心 -> 颈 向量与「竖直方向」的夹角 ---------------------
  // 定义：0° = 躯干完全竖直；角度越大表示前后/左右倾斜越明显。
  {
    AngleValue& v = out.get(AngleId::Torso);
    v.valid = false;
    v.from3d = false;

    const bool hips3dOk = sk.joints3d[idx(JointId::HipL)].valid &&
                          sk.joints3d[idx(JointId::HipR)].valid &&
                          sk.joints3d[idx(JointId::Neck)].valid;
    if (prefer3d && hips3dOk) {
      const Vec3 hips = geom::midpoint(sk.joints3d[idx(JointId::HipL)],
                                       sk.joints3d[idx(JointId::HipR)]);
      const Vec3 d = geom::sub(sk.joints3d[idx(JointId::Neck)], hips);
      // 相机坐标约定：+y 向下。竖直分量取 |dy|，水平分量取 sqrt(dx^2+dz^2)
      const float horiz = std::sqrt(d.x * d.x + d.z * d.z);
      if (std::fabs(d.y) > 1e-3f || horiz > 1e-3f) {
        v.deg = std::atan2(horiz, std::fabs(d.y)) * geom::kRad2Deg;
        v.valid = true;
        v.from3d = true;
      }
    }

    if (!v.valid) {
      const Vec2 hips2 = geom::midpoint(sk.joints2d[idx(JointId::HipL)],
                                        sk.joints2d[idx(JointId::HipR)]);
      const Vec2 neck2 = sk.joints2d[idx(JointId::Neck)];
      if (hips2.valid && neck2.valid) {
        // 图像坐标 y 向下：与 (0,-1) 的夹角即倾斜量
        const Vec2 d2 = geom::sub2(neck2, hips2);
        const float n = geom::length2(d2);
        if (n > 1e-3f) {
          float cosv = -d2.y / n;
          cosv = cosv > 1.f ? 1.f : (cosv < -1.f ? -1.f : cosv);
          v.deg = std::acos(cosv) * geom::kRad2Deg;
          v.valid = true;
          v.from3d = false;
        }
      }
    }
  }

  // ---- 肢体可见性（用于 HUD 标注可信度）-----------------------------------
  const auto limbVisible = [&](JointId a, JointId b, JointId c) {
    return usable(sk, a, mc) && usable(sk, b, mc) && usable(sk, c, mc);
  };
  out.leftArmVisible = limbVisible(JointId::ShoulderL, JointId::ElbowL, JointId::WristL);
  out.rightArmVisible = limbVisible(JointId::ShoulderR, JointId::ElbowR, JointId::WristR);
  out.leftLegVisible = limbVisible(JointId::HipL, JointId::KneeL, JointId::AnkleL);
  out.rightLegVisible = limbVisible(JointId::HipR, JointId::KneeR, JointId::AnkleR);
}

PoseAngles computeAngles(const Skeleton& sk, const AppConfig& cfg) {
  PoseAngles a;
  Skeleton copy = sk;
  fillMissingJoints(copy);
  computeAngles(copy, cfg, a);
  return a;
}

// ---------------------------------------------------------------------------
//  AngleSmoother
// ---------------------------------------------------------------------------
AngleSmoother::State& AngleSmoother::stateFor(int id, uint64_t nowUs) {
  for (auto& kv : states_) {
    if (kv.first == id) {
      kv.second.lastSeenUs = nowUs;
      return kv.second;
    }
  }
  states_.emplace_back(id, State{});
  states_.back().second.lastSeenUs = nowUs;
  return states_.back().second;
}

void AngleSmoother::evictStale(uint64_t nowUs) {
  const uint64_t kTimeoutUs = 1500000;  // 1.5 秒未出现则清理
  states_.erase(std::remove_if(states_.begin(), states_.end(),
                               [&](const std::pair<int, State>& kv) {
                                 return nowUs > kv.second.lastSeenUs &&
                                        nowUs - kv.second.lastSeenUs > kTimeoutUs;
                               }),
                states_.end());
}

void AngleSmoother::apply(int skeletonId, PoseAngles& angles, uint64_t timestampUs) {
  evictStale(timestampUs);
  State& st = stateFor(skeletonId, timestampUs);
  for (int i = 0; i < kAngleCount; ++i) {
    AngleValue& v = angles.values[i];
    if (!v.valid) {
      if (st.init[i]) {
        // 短暂丢失：保留上一次读数但置信度降低（避免数字闪烁成 0）
        v.deg = st.deg[i];
        v.valid = false;
      }
      continue;
    }
    if (alpha_ > 0.f && st.init[i]) {
      v.deg = geom::smooth(st.deg[i], v.deg, alpha_);
    }
    st.deg[i] = v.deg;
    st.init[i] = true;
  }
}

std::string formatAnglesText(const PoseAngles& angles, bool chinese) {
  std::string s;
  char buf[128];
  for (int i = 0; i < kAngleCount; ++i) {
    const AngleId id = static_cast<AngleId>(i);
    const AngleValue& v = angles.values[i];
    const char* nm = chinese ? angleNameZh(id) : angleName(id);
    if (v.valid) {
      std::snprintf(buf, sizeof(buf), "%-10s %6.1f deg  (%s)\n", nm, v.deg,
                    v.from3d ? "3D" : "2D");
    } else {
      std::snprintf(buf, sizeof(buf), "%-10s   --    \n", nm);
    }
    s += buf;
  }
  return s;
}

}  // namespace pose
