// ============================================================================
//  src/mock_source.cpp  —  无相机也能跑的模拟数据源
//
//  用途：
//    * 先把「骨骼叠加 + 角度计算 + HUD」整条链路跑通，验证渲染与算法；
//    * 没有 Astra+ / Nuitrack License 时也能演示与回归测试。
//  输出：一张合成彩色图 + 一副按正弦规律摆动的虚拟人体骨骼（含 3D 与投影后的 2D）。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "pose/color_source.hpp"
#include "pose/geometry.hpp"
#include "pose/pose_provider.hpp"

namespace pose {

std::unique_ptr<ColorSource> makeMockColorSource();
std::unique_ptr<PoseProvider> makeMockPoseProvider();

namespace {

// 虚拟相机（与 Mock 骨骼共用同一套参数，保证叠加对齐）
constexpr int kW = 1280;
constexpr int kH = 720;
constexpr float kFx = 1000.f;
constexpr float kFy = 1000.f;
constexpr float kCx = static_cast<float>(kW) * 0.5f;
constexpr float kCy = static_cast<float>(kH) * 0.5f;

// 虚拟人体在世界坐标下的姿态（y 向上，z 朝相机为正，单位毫米）
struct Rig {
  Vec3 head, neck, spine;
  Vec3 shoulderL, elbowL, wristL;
  Vec3 shoulderR, elbowR, wristR;
  Vec3 hipL, kneeL, ankleL, footL;
  Vec3 hipR, kneeR, ankleR, footR;
};

// 从根节点沿「与竖直向下成 angleDeg 度」的方向延伸 len，得到子节点。
// angleDeg = 0 表示竖直向下，正值向 +x（图像右侧）偏。
Vec3 limb(const Vec3& root, float len, float angleDeg) {
  const float rad = angleDeg / geom::kRad2Deg;
  return Vec3{root.x + std::sin(rad) * len, root.y - std::cos(rad) * len, root.z, true};
}

Rig buildRig(float t) {
  Rig r;
  // 站姿高度：踝 100mm、头 ~1500mm（相机在 3m 外、f=1000，正好整身入画）
  const float phase = std::sin(t * 1.1f);
  const float squat = 0.5f + 0.5f * std::sin(t * 0.7f);  // 0..1 下蹲程度

  const float hipY = 870.f - 200.f * squat;
  r.spine = Vec3{0.f, 1050.f, 0.f, true};
  r.neck = Vec3{0.f, 1270.f, 0.f, true};
  r.head = Vec3{0.f, 1420.f, 0.f, true};

  const float shY = 1250.f;
  const float shHalf = 170.f;
  r.shoulderL = Vec3{-shHalf, shY, 0.f, true};
  r.shoulderR = Vec3{shHalf, shY, 0.f, true};

  // 手臂：肩外展 20..70°，肘屈伸 20..115°
  const float armAbduct = 20.f + 50.f * (0.5f + 0.5f * std::sin(t * 1.3f));
  const float elbowFlex = 20.f + 95.f * (0.5f + 0.5f * std::sin(t * 1.3f + 1.2f));
  const float upperArm = 250.f, foreArm = 230.f;

  r.elbowL = limb(r.shoulderL, upperArm, 180.f - armAbduct);
  r.wristL = limb(r.elbowL, foreArm, 180.f - armAbduct - elbowFlex);
  r.elbowR = limb(r.shoulderR, upperArm, 180.f + armAbduct * 0.6f);
  r.wristR = limb(r.elbowR, foreArm, 180.f + armAbduct * 0.6f + elbowFlex * 0.5f);

  // 腿：下蹲时髋下沉、膝盖前移
  const float thigh = 380.f;
  r.hipL = Vec3{-95.f, hipY, 0.f, true};
  r.hipR = Vec3{95.f, hipY, 0.f, true};
  const float kneeFlex = 8.f + 62.f * squat;
  const float kneeDrop = thigh * std::cos(kneeFlex / geom::kRad2Deg);
  const float kneeFwd = thigh * std::sin(kneeFlex / geom::kRad2Deg);
  const float kneeY = hipY - kneeDrop;
  const float ankleY = 100.f;
  r.kneeL = Vec3{r.hipL.x, kneeY, kneeFwd, true};
  r.kneeR = Vec3{r.hipR.x, kneeY, kneeFwd, true};
  // 踝在膝下方，膝盖前移时脚相对回缩（简化的两段腿）
  const float ankleZ = kneeFwd * 0.35f;
  r.ankleL = Vec3{r.hipL.x, ankleY, ankleZ, true};
  r.ankleR = Vec3{r.hipR.x, ankleY, ankleZ, true};

  // 脚：踝 -> 前下方
  r.footL = Vec3{r.ankleL.x, ankleY - 40.f, ankleZ + 110.f, true};
  r.footR = Vec3{r.ankleR.x, ankleY - 40.f, ankleZ + 110.f, true};

  // 整个人左右轻微摆动，制造 2D/3D 差异，便于验证角度来源
  const float sway = 60.f * phase;
  for (Vec3* p : {&r.spine, &r.neck, &r.head, &r.shoulderL, &r.shoulderR, &r.elbowL,
                  &r.elbowR, &r.wristL, &r.wristR, &r.hipL, &r.hipR, &r.kneeL, &r.kneeR,
                  &r.ankleL, &r.ankleR, &r.footL, &r.footR}) {
    p->x += sway;
  }
  return r;
}

// 世界坐标(毫米) -> 彩色图像素（针孔模型，y 轴翻转）。
// 相机距离 3600mm、焦距 1000px、视轴高度 760mm（即人物身高 100..1420mm 的中点）：
// 于是人物在画面上从 y≈147（头顶）到 y≈531（脚底），竖直居中。
constexpr float kCameraDistanceMm = 3600.f;
constexpr float kCameraAxisHeightMm = 760.f;

Vec2 project(const Vec3& p) {
  Vec2 out;
  if (!p.valid) return out;
  const float z = kCameraDistanceMm + p.z;
  if (z <= 1.f) return out;
  const float yRel = p.y - kCameraAxisHeightMm;  // 相对视轴的高度
  out.x = kCx + p.x * kFx / z;
  out.y = kCy - yRel * kFy / z;
  out.valid = true;
  return out;
}

// ---------------------------------------------------------------------------
//  模拟彩色图：渐变背景 + 网格 + 与骨骼大致对应的色块，便于肉眼确认叠加位置
// ---------------------------------------------------------------------------
class MockColorSource final : public ColorSource {
 public:
  bool open(const AppConfig& cfg) override {
    cfg_ = cfg;
    frameNo_ = 0;
    return true;
  }

  bool grab(ColorFrame& color, DepthFrame& depth) override {
    (void)depth;  // mock 不提供真实深度
    const float t = static_cast<float>(frameNo_) / 30.f;
    ++frameNo_;

    const Rig rig = buildRig(t);

    color.width = kW;
    color.height = kH;
    color.stride = kW * 3;
    color.timestampUs = static_cast<uint64_t>(frameNo_) * 33333ull;
    color.bgr.assign(static_cast<size_t>(kW) * kH * 3, 0);

    // 背景：竖直渐变（上暗下亮，暗色便于叠加线与文字清晰可读）
    for (int y = 0; y < kH; ++y) {
      const uint8_t v = static_cast<uint8_t>(60 - 40.0 * y / kH);
      uint8_t* row = color.bgr.data() + static_cast<size_t>(y) * kW * 3;
      for (int x = 0; x < kW; ++x) {
        row[x * 3 + 0] = static_cast<uint8_t>(v + 30);  // B
        row[x * 3 + 1] = v;                             // G
        row[x * 3 + 2] = static_cast<uint8_t>(v * 0.7); // R
      }
    }
    // 网格（每 200mm 一条竖线，帮助判断投影比例；不依赖 OpenCV）
    for (int mm = -1200; mm <= 1200; mm += 200) {
      for (int ym = 0; ym <= 1500; ym += 10) {
        const Vec2 p = project(Vec3{static_cast<float>(mm), static_cast<float>(ym), 0.f, true});
        if (p.valid && p.x >= 0 && p.x < kW && p.y >= 0 && p.y < kH) {
          uint8_t* px = color.bgr.data() +
                        (static_cast<size_t>(p.y) * kW + static_cast<size_t>(p.x)) * 3;
          px[0] = 95; px[1] = 95; px[2] = 95;
        }
      }
    }

    // 人体色块：用关节位置画粗线段（模拟"看到一个人"）
    const auto blob = [&](const Vec3& a, const Vec3& b) {
      const Vec2 pa = project(a), pb = project(b);
      if (!pa.valid || !pb.valid) return;
      const int steps = static_cast<int>(geom::length2(geom::sub2(pb, pa))) + 1;
      for (int i = 0; i <= steps; ++i) {
        const float k = static_cast<float>(i) / steps;
        const int x = static_cast<int>(pa.x + (pb.x - pa.x) * k);
        const int y = static_cast<int>(pa.y + (pb.y - pa.y) * k);
        for (int dy = -12; dy <= 12; ++dy) {
          for (int dx = -12; dx <= 12; ++dx) {
            const int xx = x + dx, yy = y + dy;
            if (xx < 0 || yy < 0 || xx >= kW || yy >= kH) continue;
            if (dx * dx + dy * dy > 144) continue;
            uint8_t* px = color.bgr.data() + (static_cast<size_t>(yy) * kW + xx) * 3;
            px[0] = 120; px[1] = 150; px[2] = 190;  // 肤色近似（BGR）
          }
        }
      }
    };
    blob(rig.head, rig.neck);
    blob(rig.neck, rig.spine);
    blob(rig.neck, rig.shoulderL);
    blob(rig.neck, rig.shoulderR);
    blob(rig.shoulderL, rig.elbowL); blob(rig.elbowL, rig.wristL);
    blob(rig.shoulderR, rig.elbowR); blob(rig.elbowR, rig.wristR);
    blob(rig.spine, rig.hipL); blob(rig.spine, rig.hipR);
    blob(rig.hipL, rig.kneeL); blob(rig.kneeL, rig.ankleL);
    blob(rig.hipR, rig.kneeR); blob(rig.kneeR, rig.ankleR);

    // 深度小窗用数据：以人为中心的一个梯形深度场
    depth.width = 160;
    depth.height = 120;
    depth.timestampUs = color.timestampUs;
    depth.mm.assign(static_cast<size_t>(depth.width) * depth.height, 0);
    for (int y = 0; y < depth.height; ++y) {
      for (int x = 0; x < depth.width; ++x) {
        const float fx = (x - depth.width * 0.5f) / depth.width;
        const float fy = (y - depth.height * 0.5f) / depth.height;
        const float rad = std::sqrt(fx * fx + fy * fy);
        depth.mm[static_cast<size_t>(y) * depth.width + x] =
            static_cast<uint16_t>(rad < 0.25f ? 1500.f + fy * 200.f : 3500.f);
      }
    }
    depth.alignedBgr.clear();
    return true;
  }

  void close() override {}
  const char* name() const override { return "mock"; }
  std::string error() const override { return {}; }

  Intrinsics colorIntrinsics() const override {
    Intrinsics in;
    in.fx = kFx; in.fy = kFy; in.cx = kCx; in.cy = kCy; in.valid = true;
    return in;
  }

 private:
  AppConfig cfg_{};
  uint64_t frameNo_ = 0;
};

// ---------------------------------------------------------------------------
//  模拟骨骼：与上面的 Rig 完全同源，保证 2D 叠加严丝合缝
// ---------------------------------------------------------------------------
class MockPoseProvider final : public PoseProvider {
 public:
  bool start(const AppConfig& cfg) override {
    cfg_ = cfg;
    t_ = 0.f;
    return true;
  }
  void stop() override {}
  const char* name() const override { return "mock"; }
  std::string error() const override { return {}; }

  bool fetch(uint64_t colorTimestampUs, PoseFrame& out) override {
    t_ += 1.f / 30.f;
    const Rig rig = buildRig(t_);

    Skeleton s;
    s.id = 7;
    s.globalConfidence = 1.f;
    const auto put = [&s](JointId id, const Vec3& p) {
      const int i = static_cast<int>(id);
      s.joints3d[i] = p;
      s.joints2d[i] = project(p);
      s.conf[i] = 0.95f;
    };
    put(JointId::Head, rig.head);
    put(JointId::Neck, rig.neck);
    put(JointId::Spine, rig.spine);
    put(JointId::ShoulderL, rig.shoulderL);
    put(JointId::ElbowL, rig.elbowL);
    put(JointId::WristL, rig.wristL);
    put(JointId::ShoulderR, rig.shoulderR);
    put(JointId::ElbowR, rig.elbowR);
    put(JointId::WristR, rig.wristR);
    put(JointId::HipL, rig.hipL);
    put(JointId::KneeL, rig.kneeL);
    put(JointId::AnkleL, rig.ankleL);
    put(JointId::HipR, rig.hipR);
    put(JointId::KneeR, rig.kneeR);
    put(JointId::AnkleR, rig.ankleR);
    put(JointId::FootL, rig.footL);
    put(JointId::FootR, rig.footR);

    out.skeletons.clear();
    out.skeletons.push_back(std::move(s));
    out.timestampUs = colorTimestampUs;
    return true;
  }

  bool projectJoint(const Vec3& in, Vec2& out) const override {
    out = project(in);
    return out.valid;
  }
  bool projectionReady() const override { return true; }

 private:
  AppConfig cfg_{};
  float t_ = 0.f;
};

}  // namespace

std::unique_ptr<ColorSource> makeMockColorSource() {
  return std::unique_ptr<ColorSource>(new MockColorSource());
}

std::unique_ptr<PoseProvider> makeMockPoseProvider() {
  return std::unique_ptr<PoseProvider>(new MockPoseProvider());
}

}  // namespace pose
