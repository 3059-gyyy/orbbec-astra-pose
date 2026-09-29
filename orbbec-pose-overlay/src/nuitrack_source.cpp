// ============================================================================
//  src/nuitrack_source.cpp  —  使用 Nuitrack ColorSensor 的彩色底图
//
//  为什么需要它：Nuitrack 打开相机会独占 Astra+，所以当骨骼来自 Nuitrack 时，
//  彩色底图也必须来自 Nuitrack，才能保证骨骼的 2D 坐标与底图严丝合缝对齐。
//
//  实现方式：复用同一个 NuitrackPoseProvider 内部缓存的彩色帧
//  （见 nuitrack_provider.cpp 的 onNewFrame 回调与
//   nuitrackProviderFetchColor 桥接函数），不会二次打开设备。
// ============================================================================
#include <string>
#include <vector>

#include "pose/color_source.hpp"
#include "pose/pose_provider.hpp"

namespace pose {

// 由 nuitrack_provider.cpp 提供的桥接函数
bool nuitrackProviderFetchColor(PoseProvider* provider, std::vector<uint8_t>& bgr, int& w,
                                int& h);

namespace {

class NuitrackColorSource final : public ColorSource {
 public:
  explicit NuitrackColorSource(PoseProvider* provider) : provider_(provider) {}
  ~NuitrackColorSource() override { close(); }

  bool open(const AppConfig& cfg) override {
    cfg_ = cfg;
#if !defined(HAVE_NUITRACK)
    error_ =
        "本程序编译时未启用 Nuitrack，无法使用 --source nuitrack。\n"
        "可先用 --source opencv 或 --source mock 验证叠加链路，"
        "或按 README 启用 Nuitrack 后重新编译。";
    return false;
#else
    if (!provider_) {
      error_ = "内部错误：Nuitrack 彩色源需要先创建 Nuitrack 骨骼后端。";
      return false;
    }
    return true;
#endif
  }

  bool grab(ColorFrame& color, DepthFrame& depth) override {
#if !defined(HAVE_NUITRACK)
    (void)color;
    (void)depth;
    return false;
#else
    std::vector<uint8_t> bgr;
    int w = 0;
    int h = 0;
    if (!nuitrackProviderFetchColor(provider_, bgr, w, h)) {
      // 首帧可能尚未到达：返回 true（不视为失败），由主循环继续尝试
      ++waitingFrames_;
      if (waitingFrames_ > 300) {
        error_ = "等待 Nuitrack 彩色帧超时：请检查相机连接、License 是否有效，"
                 "以及 Nuitrack 是否已正确识别到设备。";
      }
      color = ColorFrame{};
      depth = DepthFrame{};
      return true;
    }
    waitingFrames_ = 0;

    color.width = w;
    color.height = h;
    color.stride = w * 3;
    color.timestampUs = ++frameIndex_;
    color.bgr = std::move(bgr);

    depth = DepthFrame{};  // Nuitrack 模式下暂不提供深度小窗
    (void)cfg_;
    return true;
#endif
  }

  void close() override {}

  const char* name() const override { return "nuitrack-color"; }
  std::string error() const override { return error_; }

 private:
  PoseProvider* provider_ = nullptr;
  AppConfig cfg_{};
  std::string error_;
  uint64_t frameIndex_ = 0;
  int waitingFrames_ = 0;
};

}  // namespace

std::unique_ptr<ColorSource> makeNuitrackColorSource(PoseProvider* provider) {
  return std::unique_ptr<ColorSource>(new NuitrackColorSource(provider));
}

}  // namespace pose
