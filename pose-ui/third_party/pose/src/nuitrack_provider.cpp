// ============================================================================
//  src/nuitrack_provider.cpp  —  Nuitrack 骨骼后端
//
//  ⚠️ 重要说明（务必先读）
//  ---------------------------------------------------------------------------
//  1. 本文件里所有 Nuitrack 调用都在 `HAVE_NUITRACK` 开关内。未定义该宏时本
//     文件只提供「未编译」的占位实现，整个工程依然能正常构建运行（会退回
//     mock 骨骼），因此你可以先把叠加与角度链路跑通，再接 SDK。
//  2. Nuitrack C++ 的类名稳定，但少数 API 名称/成员名随版本不同。
//     本文件里所有 `// VERIFY:` 注释都标出了需要你按本地头文件核对的点，
//     对照文件：`<NUITRACK_ROOT>/include/nuitrack/*.h`
//     参考示例：`<NUITRACK_ROOT>/examples/nuitrack_console_sample/main.cpp`
//  3. 设备占用：Nuitrack 打开相机会独占 Astra+，**不要**同时再让 Orbbec SDK
//     打开同一台相机。因此 `--source nuitrack` 时彩色底图也用 Nuitrack 的
//     ColorSensor；只有 `--source orbbec`（纯 Orbbec SDK）时才走 Orbbec 取流。
// ============================================================================
#include "pose/pose_provider.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#if defined(HAVE_NUITRACK)
#include <nuitrack/Nuitrack.h>
#endif

namespace pose {

namespace {

class NuitrackPoseProvider final : public PoseProvider {
 public:
  bool start(const AppConfig& cfg) override {
    cfg_ = cfg;
#if defined(HAVE_NUITRACK)
    return startReal();
#else
    error_ =
        "本程序编译时未启用 Nuitrack。\n"
        "请安装 Nuitrack SDK 后，用如下方式重新配置：\n"
        "  cmake -S . -B build -DHAVE_NUITRACK=ON -DNUITRACK_ROOT=\"C:/Nuitrack\"\n"
        "见见 README「3. 接入 Nuitrack」。";
    return false;
#endif
  }

  void stop() override {
#if defined(HAVE_NUITRACK)
    running_ = false;
    if (worker_.joinable()) worker_.join();
    colorSensor_.reset();
    tracker_.reset();
    try {
      nuitrack::release();  // VERIFY: 部分版本无需显式 release
    } catch (...) {
      // 退出阶段的异常忽略
    }
#endif
  }

  const char* name() const override { return "nuitrack"; }
  std::string error() const override { return error_; }

  bool fetch(uint64_t colorTimestampUs, PoseFrame& out) override {
    (void)colorTimestampUs;
#if defined(HAVE_NUITRACK)
    std::lock_guard<std::mutex> lk(mtx_);
    out = latest_;
    return true;
#else
    out = PoseFrame{};
    return false;
#endif
  }

  bool projectJoint(const Vec3& in, Vec2& out) const override {
    if (!in.valid) return false;
#if defined(HAVE_NUITRACK)
    std::lock_guard<std::mutex> lk(projMtx_);
    if (proj_.fx > 1e-3f && proj_.fy > 1e-3f && std::fabs(in.z) > 1e-3f) {
      out.x = proj_.cx + in.x * proj_.fx / in.z;
      out.y = proj_.cy + in.y * proj_.fy / in.z;
      out.valid = true;
      return true;
    }
#endif
    (void)out;
    return false;
  }

  bool projectionReady() const override {
#if defined(HAVE_NUITRACK)
    return proj_.fx > 1e-3f;
#else
    return false;
#endif
  }

  // 注意：cfg_ / error_ 必须在编译开关之外声明，
  // 否则未启用 Nuitrack 时上面的 start()/stop() 会出现「未声明的标识符」。
 private:
  AppConfig cfg_{};
  std::string error_;

#if defined(HAVE_NUITRACK)
 public:
  // 供 NuitrackColorSource 复用同一个 ColorSensor，避免二次打开设备
  bool copyLatestColor(std::vector<uint8_t>& bgr, int& w, int& h) const {
    std::lock_guard<std::mutex> lk(colorMtx_);
    if (colorW_ <= 0 || colorH_ <= 0 || colorBgr_.empty()) return false;
    bgr = colorBgr_;
    w = colorW_;
    h = colorH_;
    return true;
  }

 private:
  bool startReal();
  void workerLoop();
  void onSkeletonUpdate(nuitrack::SkeletonData::Ptr data);

  std::atomic<bool> running_{false};
  std::thread worker_;

  mutable std::mutex mtx_;
  PoseFrame latest_;

  mutable std::mutex colorMtx_;
  std::vector<uint8_t> colorBgr_;
  int colorW_ = 0;
  int colorH_ = 0;

  mutable std::mutex projMtx_;
  struct ManualProj {
    float fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;
  } proj_{};

  nuitrack::SkeletonTracker::Ptr tracker_;
  nuitrack::ColorSensor::Ptr colorSensor_;
#endif
};

}  // namespace

std::unique_ptr<PoseProvider> makeNuitrackPoseProvider() {
  return std::unique_ptr<PoseProvider>(new NuitrackPoseProvider());
}

// 供 NuitrackColorSource 复用同一个后端内部缓存的彩色帧（避免二次打开相机）
bool nuitrackProviderFetchColor(PoseProvider* provider, std::vector<uint8_t>& bgr, int& w,
                                int& h) {
  if (!provider) return false;
#if defined(HAVE_NUITRACK)
  auto* p = dynamic_cast<NuitrackPoseProvider*>(provider);
  if (!p) return false;
  return p->copyLatestColor(bgr, w, h);
#else
  (void)bgr;
  (void)w;
  (void)h;
  return false;
#endif
}

bool nuitrackCompiledIn() {
#if defined(HAVE_NUITRACK)
  return true;
#else
  return false;
#endif
}

#if defined(HAVE_NUITRACK)
// ===========================================================================
//  真实 Nuitrack 实现
// ===========================================================================
namespace {

// Nuitrack 关节枚举 -> 本工程 JointId。
// VERIFY: 逐项对照你本地 <nuitrack/SkeletonTracker.h> 的 JointType 枚举；
// 若某关节不存在（例如没有独立脚部关节），保持 -1，程序会自动外推补全。
struct MapRow {
  JointId dst;
  int src;
};

const MapRow kMap[] = {
    {JointId::Head, static_cast<int>(nuitrack::JOINT_HEAD)},
    {JointId::Neck, static_cast<int>(nuitrack::JOINT_NECK)},
    {JointId::ShoulderL, static_cast<int>(nuitrack::JOINT_LEFT_SHOULDER)},
    {JointId::ElbowL, static_cast<int>(nuitrack::JOINT_LEFT_ELBOW)},
    {JointId::WristL, static_cast<int>(nuitrack::JOINT_LEFT_WRIST)},
    {JointId::ShoulderR, static_cast<int>(nuitrack::JOINT_RIGHT_SHOULDER)},
    {JointId::ElbowR, static_cast<int>(nuitrack::JOINT_RIGHT_ELBOW)},
    {JointId::WristR, static_cast<int>(nuitrack::JOINT_RIGHT_WRIST)},
    {JointId::Spine, static_cast<int>(nuitrack::JOINT_TORSO)},
    {JointId::HipL, static_cast<int>(nuitrack::JOINT_LEFT_HIP)},
    {JointId::KneeL, static_cast<int>(nuitrack::JOINT_LEFT_KNEE)},
    {JointId::AnkleL, static_cast<int>(nuitrack::JOINT_LEFT_ANKLE)},
    {JointId::HipR, static_cast<int>(nuitrack::JOINT_RIGHT_HIP)},
    {JointId::KneeR, static_cast<int>(nuitrack::JOINT_RIGHT_KNEE)},
    {JointId::AnkleR, static_cast<int>(nuitrack::JOINT_RIGHT_ANKLE)},
    {JointId::FootL, -1},
    {JointId::FootR, -1},
};
constexpr int kMapSize = static_cast<int>(sizeof(kMap) / sizeof(kMap[0]));

}  // namespace

bool NuitrackPoseProvider::startReal() {
  try {
    nuitrack::init();  // VERIFY: 少数版本为 nuitrack::Nuitrack::init()
  } catch (const std::exception& e) {
    error_ = std::string("nuitrack::init() 失败: ") + e.what() +
             "\n常见原因：\n"
             "  1) 未设置 NUITRACK_HOME 环境变量；\n"
             "  2) Nuitrack License 未激活或已过期（首次运行会弹激活窗口）；\n"
             "  3) 相机被 OrbbecViewer / 其他程序独占。";
    return false;
  }

  // ---- 彩色内参（把 3D 关节投影到彩色图）--------------------------------
  // VERIFY: 若你的 SDK 没有 nuitrack::getRgbIntrinsics()，删除本段即可；
  //         程序会优先使用 Nuitrack 自己给出的 2D 投影坐标（skel.joints[].projective）。
  try {
    const auto intr = nuitrack::getRgbIntrinsics();
    std::lock_guard<std::mutex> lk(projMtx_);
    proj_.fx = static_cast<float>(intr.fx);
    proj_.fy = static_cast<float>(intr.fy);
    proj_.cx = static_cast<float>(intr.cx);
    proj_.cy = static_cast<float>(intr.cy);
  } catch (...) {
    // 拿不到内参不致命
  }

  // ---- 彩色流 ------------------------------------------------------------
  if (cfg_.preferColorFromNuitrack) {
    try {
      colorSensor_ = nuitrack::ColorSensor::create();
      colorSensor_->setOutputMode(nuitrack::ColorSensor::OutputMode::RGB);
      colorSensor_->onNewFrame([this](nuitrack::ColorFrame::Ptr frame) {
        if (!frame) return;
        const int w = frame->getCols();
        const int h = frame->getRows();
        const uint8_t* src = frame->getData();
        if (!src || w <= 0 || h <= 0) return;
        std::lock_guard<std::mutex> lk(colorMtx_);
        colorW_ = w;
        colorH_ = h;
        colorBgr_.resize(static_cast<size_t>(w) * h * 3);
        // Nuitrack RGB 帧为 RGB 排列，这里转成 BGR 供 OpenCV 直接显示
        for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
          colorBgr_[i * 3 + 0] = src[i * 3 + 2];  // B
          colorBgr_[i * 3 + 1] = src[i * 3 + 1];  // G
          colorBgr_[i * 3 + 2] = src[i * 3 + 0];  // R
        }
      });
    } catch (const std::exception& e) {
      error_ = std::string("ColorSensor 初始化失败（将只显示骨骼叠加所需的最小背景）: ") + e.what();
    }
  }

  // ---- 骨骼流 ------------------------------------------------------------
  try {
    tracker_ = nuitrack::SkeletonTracker::create();
    tracker_->onSkeletonUpdate(
        [this](nuitrack::SkeletonData::Ptr data) { onSkeletonUpdate(std::move(data)); });
  } catch (const std::exception& e) {
    error_ = std::string("SkeletonTracker 初始化失败: ") + e.what();
    stop();
    return false;
  }

  running_ = true;
  worker_ = std::thread([this] { workerLoop(); });
  return true;
}

void NuitrackPoseProvider::workerLoop() {
  while (running_) {
    try {
      nuitrack::update();  // VERIFY: 少数版本为 nuitrack::Nuitrack::update()
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lk(mtx_);
      error_ = std::string("nuitrack::update() 异常: ") + e.what();
      running_ = false;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void NuitrackPoseProvider::onSkeletonUpdate(nuitrack::SkeletonData::Ptr data) {
  PoseFrame frame;
  frame.timestampUs = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());

  if (data) {
    for (const auto& skel : data->getSkeletons()) {
      Skeleton s;
      s.id = skel.id;
      s.globalConfidence = 1.f;

      const auto& joints = skel.joints;
      for (int i = 0; i < kMapSize; ++i) {
        const int src = kMap[i].src;
        if (src < 0 || src >= static_cast<int>(joints.size())) continue;
        const auto& j = joints[static_cast<size_t>(src)];
        const int dst = static_cast<int>(kMap[i].dst);

        // --- 3D：相机坐标系，单位毫米 ---
        // VERIFY: Joint 成员名。多数版本为 j.real（3D，相机坐标），
        //         j.projective（2D，已投影到彩色图）。
        Vec3 p3;
        p3.x = static_cast<float>(j.real.x);
        p3.y = static_cast<float>(j.real.y);
        p3.z = static_cast<float>(j.real.z);
        p3.valid = (p3.z > 1.f);
        s.joints3d[dst] = p3;

        // --- 2D：Nuitrack 已按深度-彩色标定投影好 ---
        Vec2 p2;
        p2.x = static_cast<float>(j.projective.x);
        p2.y = static_cast<float>(j.projective.y);
        p2.valid = (p2.x > 0.f && p2.y > 0.f);
        s.joints2d[dst] = p2;

        s.conf[dst] = 1.f;
      }

      // 缺失脚部关节：用「踝 + (踝-膝)*0.25」外推，保证骨架连线不断
      const auto fillFoot = [&s](JointId foot, JointId ankle, JointId knee) {
        const int f = static_cast<int>(foot);
        const int a = static_cast<int>(ankle);
        const int k = static_cast<int>(knee);
        if (s.joints3d[f].valid || !s.joints3d[a].valid || !s.joints3d[k].valid) return;
        const float dx = s.joints3d[a].x - s.joints3d[k].x;
        const float dy = s.joints3d[a].y - s.joints3d[k].y;
        const float dz = s.joints3d[a].z - s.joints3d[k].z;
        s.joints3d[f] = Vec3{s.joints3d[a].x + dx * 0.25f, s.joints3d[a].y + dy * 0.25f,
                             s.joints3d[a].z + dz * 0.25f, true};
        s.conf[f] = 0.8f;
        if (s.joints2d[a].valid && s.joints2d[k].valid) {
          s.joints2d[f] =
              Vec2{s.joints2d[a].x + (s.joints2d[a].x - s.joints2d[k].x) * 0.25f,
                   s.joints2d[a].y + (s.joints2d[a].y - s.joints2d[k].y) * 0.25f, true};
        }
      };
      fillFoot(JointId::FootL, JointId::AnkleL, JointId::KneeL);
      fillFoot(JointId::FootR, JointId::AnkleR, JointId::KneeR);

      frame.skeletons.push_back(std::move(s));
    }
  }

  std::lock_guard<std::mutex> lk(mtx_);
  latest_ = std::move(frame);
}
#endif  // HAVE_NUITRACK

}  // namespace pose
