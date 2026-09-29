// ============================================================================
//  src/orbbec_pose_provider.cpp  —  奥比中光官方 Body Tracking 骨骼后端
//
//  适用：奥比中光 **Astra SDK 2.x + Body Tracking 算法模块**
//        （官方说明兼容 Astra / Persee 系列；Astra+ 是否在内需向奥比中光确认）
//
//  ⚠️ 三条前置条件，缺一不可：
//    1. 安装 Astra SDK 2.x（含 astra.lib / astra.dll 与 include/astra/）
//    2. 机器上存在 Body Tracking 算法模块与**有效 License**
//       —— 试用版会过期，常见报错 "bodytracking trial expired"。
//          申请/咨询入口见 README 第 6 节（3dclub 论坛 Body tracking license 帖）。
//    3. 编译时打开 -DHAVE_ASTRA_SDK=ON 并给出 ASTRA_SDK_ROOT
//
//  ⚠️ 本文件所有 astra:: 调用都标了 `// VERIFY:`。
//     请对照 <ASTRA_SDK_ROOT>/include/astra/ 下的头文件与
//     <ASTRA_SDK_ROOT>/samples/（如 bodytracking / BodyTracker 示例）逐条核对。
//
//  数据流：
//    astra::StreamSet ──create_reader<BodyStream>──▶ BodyFrame ──▶ Body ──▶ Joint[]
//        │                                                                  │
//        └──create_reader<ColorStream>──▶ ColorFrame（彩色底图，可选）      │
//                                                                           ▼
//                              3D(深度坐标, mm) + color_position(已投影到彩色图)
// ============================================================================
#include "pose/pose_provider.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(HAVE_ASTRA_SDK)
#include <astra/astra.hpp>
#endif

namespace pose {

std::unique_ptr<PoseProvider> makeOrbbecAstraPoseProvider();

bool astraCompiledIn() {
#if defined(HAVE_ASTRA_SDK)
  return true;
#else
  return false;
#endif
}

namespace {

#if defined(HAVE_ASTRA_SDK)
// 只引入确实需要的名字，避免与 pose 命名空间里的同名字段/类型冲突。
// 注意：不能引入 astra::ColorFrame —— 它与 pose::ColorFrame 同名会造成歧义。
using astra::BodyFrame;
using astra::BodyStream;
using astra::ColorStream;
using astra::JointType;
using astra::Vector3f;
#endif

class OrbbecAstraPoseProvider final : public PoseProvider {
 public:
  ~OrbbecAstraPoseProvider() override { stop(); }

  bool start(const AppConfig& cfg) override {
    cfg_ = cfg;
#if defined(HAVE_ASTRA_SDK)
    return startReal();
#else
    error_ =
        "本程序编译时未启用奥比中光 Astra SDK Body Tracking。\n"
        "启用方式：\n"
        "  1) 安装 Astra SDK 2.x，并确认 Body Tracking 算法模块与 License 有效；\n"
        "  2) cmake -S . -B build -DHAVE_ASTRA_SDK=ON "
        "-DASTRA_SDK_ROOT=\"C:/AstraSDK\"\n"
        "见见 README「6. 接入奥比中光 Body Tracking SDK」。\n"
        "当前可先用 --pose nuitrack 或 --pose mock。";
    return false;
#endif
  }

  void stop() override {
#if defined(HAVE_ASTRA_SDK)
    running_ = false;
    if (worker_.joinable()) worker_.join();
    colorReader_ = nullptr;
    bodyReader_ = nullptr;
    streamSet_ = nullptr;
    if (astraInited_) {
      astra::terminate();  // VERIFY: 与 initialize() 配对
      astraInited_ = false;
    }
#endif
  }

  const char* name() const override { return "orbbec-astra"; }
  std::string error() const override { return error_; }

  bool fetch(uint64_t colorTimestampUs, PoseFrame& out) override {
    (void)colorTimestampUs;
#if defined(HAVE_ASTRA_SDK)
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
#if defined(HAVE_ASTRA_SDK)
    if (intrValid_ && std::fabs(in.z) > 1e-3f) {
      // 注意：这里假设关节已在彩色相机坐标系下（Astra SDK 的 color_position
      // 就是这种语义）。若你用的是 depth_position，请先用
      // astra::get_camera_params() 的 depthToColor 外参做刚体变换。
      out.x = intrCx_ + in.x * intrFx_ / in.z;
      out.y = intrCy_ + in.y * intrFy_ / in.z;
      out.valid = true;
      return true;
    }
#endif
    (void)out;
    return false;
  }

  bool projectionReady() const override {
#if defined(HAVE_ASTRA_SDK)
    return intrValid_;
#else
    return false;
#endif
  }

#if defined(HAVE_ASTRA_SDK)
  // 供彩色底图复用（Astra SDK 已打开相机，不能被第二个 SDK 抢占）
  bool copyLatestColor(std::vector<uint8_t>& bgr, int& w, int& h) const {
    std::lock_guard<std::mutex> lk(colorMtx_);
    if (colorW_ <= 0 || colorH_ <= 0 || colorBgr_.empty()) return false;
    bgr = colorBgr_;
    w = colorW_;
    h = colorH_;
    return true;
  }
#endif

  // 注意：cfg_ / error_ 及相关状态必须声明在编译开关之外，
  // 否则未启用 Astra SDK 时上面的成员函数会报「未声明的标识符」。
 private:
  AppConfig cfg_{};
  std::string error_;

#if defined(HAVE_ASTRA_SDK)
  bool startReal();
  void workerLoop();
  void onBodyFrame(BodyFrame& frame);

  std::atomic<bool> running_{false};
  std::thread worker_;
  bool astraInited_ = false;

  mutable std::mutex mtx_;
  PoseFrame latest_;

  mutable std::mutex colorMtx_;
  std::vector<uint8_t> colorBgr_;
  int colorW_ = 0;
  int colorH_ = 0;

  bool intrValid_ = false;
  float intrFx_ = 0.f, intrFy_ = 0.f, intrCx_ = 0.f, intrCy_ = 0.f;

  std::unique_ptr<astra::StreamSet> streamSet_;      // VERIFY: 类型名
  astra::BodyStream* bodyReader_ = nullptr;          // 生命周期由 StreamSet 持有
  astra::ColorStream* colorReader_ = nullptr;
#endif
};

#if defined(HAVE_ASTRA_SDK)
// ---------------------------------------------------------------------------
//  Astra 关节 -> 本工程 JointId
//  官方骨骼含 19 个关节，本工程用其中 18 个（脚部由踝外推补全）。
//  VERIFY: 逐项核对 <astra/JointType.hpp>；缺项写 -1，程序会自动外推。
// ---------------------------------------------------------------------------
struct MapRow {
  JointId dst;
  int src;  // JointType 的整数值，-1 表示 SDK 无此关节
};

const MapRow kMap[] = {
    {JointId::Head, static_cast<int>(JointType::Head)},
    {JointId::Neck, static_cast<int>(JointType::Neck)},
    {JointId::ShoulderL, static_cast<int>(JointType::LeftShoulder)},
    {JointId::ElbowL, static_cast<int>(JointType::LeftElbow)},
    {JointId::WristL, static_cast<int>(JointType::LeftHand)},
    {JointId::ShoulderR, static_cast<int>(JointType::RightShoulder)},
    {JointId::ElbowR, static_cast<int>(JointType::RightElbow)},
    {JointId::WristR, static_cast<int>(JointType::RightHand)},
    {JointId::Spine, static_cast<int>(JointType::Torso)},
    {JointId::HipL, static_cast<int>(JointType::LeftHip)},
    {JointId::KneeL, static_cast<int>(JointType::LeftKnee)},
    {JointId::AnkleL, static_cast<int>(JointType::LeftFoot)},
    {JointId::HipR, static_cast<int>(JointType::RightHip)},
    {JointId::KneeR, static_cast<int>(JointType::RightKnee)},
    {JointId::AnkleR, static_cast<int>(JointType::RightFoot)},
    {JointId::FootL, -1},
    {JointId::FootR, -1},
};
constexpr int kMapSize = static_cast<int>(sizeof(kMap) / sizeof(kMap[0]));

bool OrbbecAstraPoseProvider::startReal() {
  try {
    astra::initialize();  // VERIFY: 返回类型/是否需要参数
    astraInited_ = true;

    streamSet_ = std::make_unique<astra::StreamSet>();  // VERIFY: 类型名与构造方式

    bodyReader_ = &streamSet_->create_reader<BodyStream>();  // VERIFY
    bodyReader_->on_frame_ready([this](BodyStream&, BodyFrame& frame) {
      onBodyFrame(frame);
    });

    if (cfg_.bodyTrackingColorStream) {
      try {
        colorReader_ = &streamSet_->create_reader<ColorStream>();  // VERIFY
        // 直接把 astra::ColorFrame 转成 pose 用的紧凑 BGR 缓冲，
        // 避免在类里再声明一个与 pose::ColorFrame 同名的转换函数。
        colorReader_->on_frame_ready(
            [this](ColorStream&, astra::ColorFrame& frame) {  // VERIFY: 回调签名
              try {
                // VERIFY: 取分辨率与像素指针的接口名
                const auto res = frame.resolution();
                const int w = static_cast<int>(res.width);
                const int h = static_cast<int>(res.height);
                const astra::RgbPixel* px = frame.data();
                if (!px || w <= 0 || h <= 0) return;
                std::lock_guard<std::mutex> lk(colorMtx_);
                colorW_ = w;
                colorH_ = h;
                colorBgr_.resize(static_cast<size_t>(w) * h * 3);
                for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
                  colorBgr_[i * 3 + 0] = px[i].b;
                  colorBgr_[i * 3 + 1] = px[i].g;
                  colorBgr_[i * 3 + 2] = px[i].r;
                }
              } catch (...) {
                // 单帧异常忽略
              }
            });
      } catch (const std::exception&) {
        colorReader_ = nullptr;  // 没有彩色流不致命
      }
    }

    // 彩色内参：优先取 color 流的 intrinsic，取不到则由上层用 2D 坐标兜底
    try {
      // VERIFY: 名称可能是 get_camera_params() / get_color_intrinsics()；
      //         返回结构字段名也可能是 fx/fy/cx/cy 之外的写法。
      const auto params = astra::get_camera_params();  // VERIFY
      intrFx_ = params.color.fx;                       // VERIFY
      intrFy_ = params.color.fy;
      intrCx_ = params.color.cx;
      intrCy_ = params.color.cy;
      intrValid_ = intrFx_ > 1e-3f && intrFy_ > 1e-3f;
    } catch (...) {
      intrValid_ = false;
    }

    running_ = true;
    worker_ = std::thread([this] { workerLoop(); });
    return true;
  } catch (const std::exception& e) {
    error_ = std::string("Astra SDK 启动失败: ") + e.what() +
             "\n排查：\n"
             "  1) Body Tracking 算法模块是否随 SDK 一起安装；\n"
             "  2) License 是否有效（试用版过期会启动失败）；\n"
             "  3) 相机是否被其他程序（OrbbecViewer / Nuitrack）占用；\n"
             "  4) Astra+ 是否在 Body Tracking 支持列表内（需向奥比中光确认）。";
    stop();
    return false;
  }
}

void OrbbecAstraPoseProvider::workerLoop() {
  while (running_) {
    try {
      astra::update();  // VERIFY: 触发回调的轮询函数名
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lk(mtx_);
      error_ = std::string("astra::update() 异常: ") + e.what();
      running_ = false;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// 彩色帧的转换逻辑已内联到 startReal() 的 on_frame_ready 回调里（见上）。

void OrbbecAstraPoseProvider::onBodyFrame(BodyFrame& frame) {
  PoseFrame out;
  out.timestampUs = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());

  try {
    frame.get_bodies([&out](astra::Body& body) {  // VERIFY: 遍历接口名
      Skeleton s;
      s.id = static_cast<int>(body.id());  // VERIFY: id() 的返回类型（unsigned int）
      s.globalConfidence = 1.f;

      const auto& joints = body.joints();  // VERIFY: joints() 返回容器/数组

      for (int i = 0; i < kMapSize; ++i) {
        const int src = kMap[i].src;
        if (src < 0 || src >= static_cast<int>(joints.size())) continue;
        // VERIFY: 遍历方式（下标 / 查找 JointType）。部分版本为
        //   body.joints()[static_cast<size_t>(JointType::Head)]
        const astra::Joint& j = joints[static_cast<size_t>(src)];
        const int dst = static_cast<int>(kMap[i].dst);

        // --- 3D：深度相机坐标系，单位毫米 ---
        // VERIFY: 成员名 depth_position / world_position；若为后者请先做外参变换
        const Vector3f& p = j.depth_position;  // VERIFY
        Vec3 p3;
        p3.x = p.x;
        p3.y = p.y;
        p3.z = p.z;
        p3.valid = (p3.z > 1.f);
        s.joints3d[dst] = p3;

        // --- 2D：Astra SDK 已给出投影到彩色图的坐标 ---
        // VERIFY: 若你的版本没有 color_position，可置为无效，
        //         上层会用 projectJoint() 或内参兜底投影。
        const Vector3f& c = j.color_position;  // VERIFY
        Vec2 p2;
        p2.x = c.x;
        p2.y = c.y;
        p2.valid = (p2.x > 0.f && p2.y > 0.f);
        s.joints2d[dst] = p2;

        // VERIFY: 部分版本提供 j.depth_confidence / j.world_confidence
        s.conf[dst] = 1.f;
      }

      // 脚部关节外推：踝 + (踝-膝)*0.25
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

      out.skeletons.push_back(std::move(s));
    });
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lk(mtx_);
    error_ = std::string("解析 BodyFrame 失败: ") + e.what();
  }

  std::lock_guard<std::mutex> lk(mtx_);
  latest_ = std::move(out);
}
#endif  // HAVE_ASTRA_SDK

}  // namespace

std::unique_ptr<PoseProvider> makeOrbbecAstraPoseProvider() {
  return std::unique_ptr<PoseProvider>(new OrbbecAstraPoseProvider());
}

}  // namespace pose
