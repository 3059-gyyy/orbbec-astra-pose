// ============================================================================
//  src/dnn_pose.cpp  —  基于 ONNX 姿态模型的人体骨骼后端（YOLOv8-pose）
//
//  为什么是这个：奥比中光官方 Body Tracking 与 Nuitrack 都需要申请 License，
//  而 ONNX 姿态模型跑在彩色图上即可出 17 个 COCO 关键点，**无需任何授权**，
//  且与相机型号无关（Astra+ / 普通摄像头 / 视频文件都能用）。
//
//  数据流：
//    彩色帧(BGR) ──letterbox──▶ 640x640 ──cv::dnn──▶ [1,56,N]
//        │                                              │
//        │                                   框(4) + 置信度(1) + 17×(x,y,conf)
//        ▼                                              ▼
//   反 letterbox 到原图坐标 ──────────────────────▶ COCO-17 关键点
//        │
//        └── 映射到本工程 18 关节（Neck/Spine 由双肩/双髋推算；脚用踝近似）
//
//  模型获取（12.9 MB，已验证可下载）：
//    https://huggingface.co/Xenova/yolov8n-pose/resolve/main/onnx/model.onnx
//
//  ⚠️ 若你换成别的姿态模型（BlazePose / MoveNet / RTMPose 等），
//     只需重写 infer() 里的「输出解码」部分，其余（映射、角度、渲染）都不用动。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "pose/geometry.hpp"
#include "pose/pose_provider.hpp"

namespace pose {

std::unique_ptr<PoseProvider> makeDnnPoseProvider();

namespace {

// COCO-17 关键点顺序
enum CocoKpt {
  kCocoNose = 0,
  kCocoEyeL,
  kCocoEyeR,
  kCocoEarL,
  kCocoEarR,
  kCocoShoulderL,
  kCocoShoulderR,
  kCocoElbowL,
  kCocoElbowR,
  kCocoWristL,
  kCocoWristR,
  kCocoHipL,
  kCocoHipR,
  kCocoKneeL,
  kCocoKneeR,
  kCocoAnkleL,
  kCocoAnkleR,
};
constexpr int kCocoCount = 17;

struct Detection {
  float score = 0.f;
  cv::Rect2f box;
  std::array<float, kCocoCount * 3> kpt{};  // x, y, conf
};

// 让检测框按置信度降序排列
bool byScoreDesc(const Detection& a, const Detection& b) { return a.score > b.score; }

}  // namespace

class DnnPoseProvider final : public PoseProvider {
 public:
  bool start(const AppConfig& cfg) override {
    cfg_ = cfg;
    error_.clear();
    if (cfg.poseModelPath.empty()) {
      error_ =
          "未指定 ONNX 姿态模型。请加参数 --pose-model <path>\n"
          "例如：--pose dnn --pose-model models\\yolov8n-pose.onnx\n"
          "模型下载地址（12.9MB）：\n"
          "  https://huggingface.co/Xenova/yolov8n-pose/resolve/main/onnx/model.onnx";
      return false;
    }
    try {
      net_ = cv::dnn::readNetFromONNX(cfg.poseModelPath);
      if (net_.empty()) {
        error_ = "模型加载失败（cv::dnn::readNetFromONNX 返回空）: " + cfg.poseModelPath;
        return false;
      }
      // 单线程即可：姿态模型在 640x640 上很快，多线程反而抢占采集线程
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      ready_ = true;
      std::printf("[dnn] 已加载姿态模型: %s (输入 %dx%d)\n", cfg.poseModelPath.c_str(),
                  cfg.poseInputSize, cfg.poseInputSize);
      return true;
    } catch (const std::exception& e) {
      error_ = std::string("模型加载异常: ") + e.what() + "\n路径: " + cfg.poseModelPath;
      return false;
    }
  }

  void stop() override { ready_ = false; }

  const char* name() const override { return "dnn-yolov8-pose"; }
  std::string error() const override { return error_; }
  bool projectionReady() const override { return false; }  // 2D 已在图像坐标系，无需投影

  bool projectJoint(const Vec3& in, Vec2& out) const override {
    (void)in;
    (void)out;
    return false;
  }

  bool fetch(uint64_t colorTimestampUs, PoseFrame& out) override {
    out.skeletons.clear();
    out.timestampUs = colorTimestampUs;
    if (!ready_) return false;
    // 真正的推理在 infer() 里由主循环传入当前帧调用（见 runOnFrame）
    return true;
  }

  // 主循环每帧调用：直接在彩色帧上跑模型，产出骨骼（避免复制整帧的开销）
  bool runOnFrame(const ColorFrame& color, PoseFrame& out) {
    out.skeletons.clear();
    out.timestampUs = color.timestampUs;
    if (!ready_ || color.empty()) return false;

    cv::Mat bgr(color.height, color.width, CV_8UC3,
                const_cast<uint8_t*>(color.bgr.data()));
    std::vector<Detection> dets;
    if (!infer(bgr, dets)) return false;
    decodeDetections(dets, out);
    return true;
  }

  // 界面工程调用：直接在裸 BGR 缓冲上推理，并允许实时更新阈值类参数
  bool runOnFrameRaw(const AppConfig& cfg, const uint8_t* bgr, int width, int height,
                     PoseFrame& out) {
    out.skeletons.clear();
    if (!ready_ || !bgr || width <= 0 || height <= 0) return false;
    // 只更新不需要重载模型的参数
    cfg_.poseScoreThreshold = cfg.poseScoreThreshold;
    cfg_.poseKptThreshold = cfg.poseKptThreshold;
    cfg_.poseMaxPersons = cfg.poseMaxPersons;

    cv::Mat img(height, width, CV_8UC3, const_cast<uint8_t*>(bgr));
    std::vector<Detection> dets;
    if (!infer(img, dets)) return false;
    decodeDetections(dets, out);
    return true;
  }

 private:
  // 把检测结果转换成统一的 18 关节骨架（COCO-17 -> 本工程关节集）
  void decodeDetections(const std::vector<Detection>& dets, PoseFrame& out) {
    for (const Detection& d : dets) {
      Skeleton s;
      s.id = nextId_++;
      s.globalConfidence = d.score;

      const float kptThr = cfg_.poseKptThreshold;
      const auto put2d = [&s, kptThr](JointId id, float x, float y, float conf) {
        const int i = static_cast<int>(id);
        s.joints2d[i] = Vec2{x, y, conf >= kptThr};
        s.conf[i] = conf;
      };
      const auto kx = [&d](int k) { return d.kpt[k * 3 + 0]; };
      const auto ky = [&d](int k) { return d.kpt[k * 3 + 1]; };
      const auto kc = [&d](int k) { return d.kpt[k * 3 + 2]; };

      // 头部：用置信度最高的五官加权平均，比单取 Nose 稳
      {
        float sx = 0.f, sy = 0.f, sw = 0.f;
        const int face[5] = {kCocoNose, kCocoEyeL, kCocoEyeR, kCocoEarL, kCocoEarR};
        for (int k : face) {
          const float wgt = kc(k);
          if (wgt < kptThr) continue;
          sx += kx(k) * wgt;
          sy += ky(k) * wgt;
          sw += wgt;
        }
        if (sw > 1e-3f) {
          put2d(JointId::Head, sx / sw, sy / sw, sw / 5.f);
        } else {
          put2d(JointId::Head, 0.f, 0.f, 0.f);
        }
      }

      // 四肢直接映射
      put2d(JointId::ShoulderL, kx(kCocoShoulderL), ky(kCocoShoulderL), kc(kCocoShoulderL));
      put2d(JointId::ElbowL, kx(kCocoElbowL), ky(kCocoElbowL), kc(kCocoElbowL));
      put2d(JointId::WristL, kx(kCocoWristL), ky(kCocoWristL), kc(kCocoWristL));
      put2d(JointId::ShoulderR, kx(kCocoShoulderR), ky(kCocoShoulderR), kc(kCocoShoulderR));
      put2d(JointId::ElbowR, kx(kCocoElbowR), ky(kCocoElbowR), kc(kCocoElbowR));
      put2d(JointId::WristR, kx(kCocoWristR), ky(kCocoWristR), kc(kCocoWristR));
      put2d(JointId::HipL, kx(kCocoHipL), ky(kCocoHipL), kc(kCocoHipL));
      put2d(JointId::KneeL, kx(kCocoKneeL), ky(kCocoKneeL), kc(kCocoKneeL));
      put2d(JointId::AnkleL, kx(kCocoAnkleL), ky(kCocoAnkleL), kc(kCocoAnkleL));
      put2d(JointId::HipR, kx(kCocoHipR), ky(kCocoHipR), kc(kCocoHipR));
      put2d(JointId::KneeR, kx(kCocoKneeR), ky(kCocoKneeR), kc(kCocoKneeR));
      put2d(JointId::AnkleR, kx(kCocoAnkleR), ky(kCocoAnkleR), kc(kCocoAnkleR));

      // 推算：颈 = 双肩中点；脊柱 = 双髋中点；脚 = 踝（COCO 无脚点）
      const Vec2 shL = s.joints2d[static_cast<int>(JointId::ShoulderL)];
      const Vec2 shR = s.joints2d[static_cast<int>(JointId::ShoulderR)];
      const Vec2 hpL = s.joints2d[static_cast<int>(JointId::HipL)];
      const Vec2 hpR = s.joints2d[static_cast<int>(JointId::HipR)];

      const Vec2 neck = geom::midpoint(shL, shR);
      s.joints2d[static_cast<int>(JointId::Neck)] = neck;
      s.conf[static_cast<int>(JointId::Neck)] =
          std::min(s.conf[static_cast<int>(JointId::ShoulderL)],
                   s.conf[static_cast<int>(JointId::ShoulderR)]);

      const Vec2 spine = geom::midpoint(hpL, hpR);
      s.joints2d[static_cast<int>(JointId::Spine)] = spine;
      s.conf[static_cast<int>(JointId::Spine)] =
          std::min(s.conf[static_cast<int>(JointId::HipL)],
                   s.conf[static_cast<int>(JointId::HipR)]);

      s.joints2d[static_cast<int>(JointId::FootL)] =
          s.joints2d[static_cast<int>(JointId::AnkleL)];
      s.joints2d[static_cast<int>(JointId::FootR)] =
          s.joints2d[static_cast<int>(JointId::AnkleR)];
      s.conf[static_cast<int>(JointId::FootL)] = s.conf[static_cast<int>(JointId::AnkleL)];
      s.conf[static_cast<int>(JointId::FootR)] = s.conf[static_cast<int>(JointId::AnkleR)];

      // 双肩/双髋都不可信时，这一帧整体丢弃（避免画出一堆错位骨架）
      if (!neck.valid && !spine.valid) continue;
      out.skeletons.push_back(std::move(s));
      if (static_cast<int>(out.skeletons.size()) >= cfg_.poseMaxPersons) break;
    }
  }

  // 预处理：letterbox 到正方形，保持长宽比，填充 114
  cv::Mat letterbox(const cv::Mat& src, float& scale, int& padX, int& padY) const {
    const int in = cfg_.poseInputSize;
    scale = std::min(static_cast<float>(in) / src.cols, static_cast<float>(in) / src.rows);
    const int newW = static_cast<int>(std::round(src.cols * scale));
    const int newH = static_cast<int>(std::round(src.rows * scale));
    padX = (in - newW) / 2;
    padY = (in - newH) / 2;

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(newW, newH), 0, 0, cv::INTER_LINEAR);
    cv::Mat canvas(in, in, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(padX, padY, newW, newH)));
    return canvas;
  }

  bool infer(const cv::Mat& bgr, std::vector<Detection>& out) {
    float scale = 1.f;
    int padX = 0, padY = 0;
    cv::Mat canvas = letterbox(bgr, scale, padX, padY);

    cv::Mat blob;
    // YOLOv8 需要 RGB、0..1 归一化（swapRB=true 把 BGR 转 RGB）
    cv::dnn::blobFromImage(canvas, blob, 1.0 / 255.0, cv::Size(cfg_.poseInputSize, cfg_.poseInputSize),
                           cv::Scalar(), true, false, CV_32F);
    net_.setInput(blob);
    cv::Mat out0;
    try {
      out0 = net_.forward();
    } catch (const std::exception& e) {
      error_ = std::string("模型前向计算失败: ") + e.what();
      return false;
    }

    // YOLOv8-pose 输出：[1, 56, N]（4 框 + 1 置信度 + 17×3 关键点）
    cv::Mat o = out0;
    if (o.dims == 3 && o.size[1] != 56 && o.size[2] == 56) {
      // 少数导出形式为 [1, N, 56]，转置过来
      cv::Mat tmp;
      cv::transpose(o.reshape(1, o.size[1]), tmp);
      o = tmp;
    }
    if (o.dims != 3 || o.size[1] < 56) {
      error_ = "模型输出维度不符合 YOLOv8-pose 约定（期望 [1,56,N]）";
      return false;
    }

    const int channels = o.size[1];
    const int anchors = o.size[2];
    const float* data = reinterpret_cast<const float*>(o.data);
    // 内存布局：[1, channels, anchors]，元素 (c, a) 位于 data[c * anchors + a]
    const auto at = [&](int c, int a) { return data[static_cast<size_t>(c) * anchors + a]; };

    const float invScale = 1.f / (scale > 1e-6f ? scale : 1.f);
    std::vector<Detection> raw;
    raw.reserve(64);

    // 诊断：打印维度与最佳锚点，便于与 Python 参考实现逐项对比
    if (std::getenv("POSE_DNN_DEBUG") != nullptr) {
      std::printf("[dnn] out dims=%d size=[%d,%d,%d] scale=%.4f pad=(%d,%d)\n", o.dims,
                  o.size[0], o.size[1], o.size[2], scale, padX, padY);
      int best = -1;
      float bestScore = -1.f;
      for (int a = 0; a < anchors; ++a) {
        const float sc = at(4, a);
        if (sc > bestScore) { bestScore = sc; best = a; }
      }
      if (best >= 0) {
        std::printf("[dnn] best anchor=%d conf=%.4f box=(%.1f,%.1f,%.1f,%.1f)\n", best,
                    bestScore, at(0, best), at(1, best), at(2, best), at(3, best));
        for (int k = 0; k < kCocoCount; ++k) {
          std::printf("[dnn]   kpt %2d  raw=(%7.1f,%7.1f) conf=%.3f\n", k,
                      at(5 + k * 3 + 0, best), at(5 + k * 3 + 1, best),
                      at(5 + k * 3 + 2, best));
        }
      }
    }

    for (int a = 0; a < anchors; ++a) {
      const float score = at(4, a);
      if (score < cfg_.poseScoreThreshold) continue;

      const float cx = at(0, a), cy = at(1, a), w = at(2, a), h = at(3, a);
      Detection d;
      d.score = score;
      // 反 letterbox：先去填充，再除以缩放比
      const float x0 = (cx - w * 0.5f - padX) * invScale;
      const float y0 = (cy - h * 0.5f - padY) * invScale;
      d.box = cv::Rect2f(x0, y0, w * invScale, h * invScale);

      for (int k = 0; k < kCocoCount; ++k) {
        const float kxr = at(5 + k * 3 + 0, a);
        const float kyr = at(5 + k * 3 + 1, a);
        const float ksc = at(5 + k * 3 + 2, a);
        d.kpt[k * 3 + 0] = (kxr - padX) * invScale;
        d.kpt[k * 3 + 1] = (kyr - padY) * invScale;
        d.kpt[k * 3 + 2] = ksc;
      }
      raw.push_back(d);
    }

    // 按置信度排序 + 贪心 NMS（同一人只留一个框）
    std::sort(raw.begin(), raw.end(), byScoreDesc);
    std::vector<Detection> keep;
    std::vector<bool> removed(raw.size(), false);
    const float nmsThr = 0.45f;
    for (size_t i = 0; i < raw.size(); ++i) {
      if (removed[i]) continue;
      keep.push_back(raw[i]);
      if (static_cast<int>(keep.size()) >= cfg_.poseMaxPersons) break;
      for (size_t j = i + 1; j < raw.size(); ++j) {
        if (removed[j]) continue;
        const float inter = (raw[i].box & raw[j].box).area();
        const float uni = raw[i].box.area() + raw[j].box.area() - inter;
        const float iou = uni > 1e-6f ? inter / uni : 0.f;
        if (iou > nmsThr) removed[j] = true;
      }
    }
    out = std::move(keep);
    return true;
  }

  AppConfig cfg_{};
  cv::dnn::Net net_;
  bool ready_ = false;
  int nextId_ = 1;
  std::string error_;
};

std::unique_ptr<PoseProvider> makeDnnPoseProvider() {
  return std::unique_ptr<PoseProvider>(new DnnPoseProvider());
}

// 供主循环调用：直接在当前彩色帧上推理。
// 返回 false 表示该后端不是 DNN（或未就绪），主循环会退回 provider->fetch()。
bool dnnRunOnFrame(PoseProvider* provider, const ColorFrame& color, PoseFrame& out) {
  if (!provider) return false;
  auto* p = dynamic_cast<DnnPoseProvider*>(provider);
  if (!p) return false;
  return p->runOnFrame(color, out);
}

// 界面工程用：在裸 BGR 缓冲上推理一条路径，避免构造 ColorFrame 时的整帧拷贝。
bool dnnInferOnFrame(PoseProvider* provider, const AppConfig& cfg, const uint8_t* bgr,
                     int width, int height, PoseFrame& out) {
  if (!provider || !bgr || width <= 0 || height <= 0) return false;
  auto* p = dynamic_cast<DnnPoseProvider*>(provider);
  if (!p) return false;
  // 阈值等参数可能被界面实时调整，这里同步一次（不影响已加载的模型）
  return p->runOnFrameRaw(cfg, bgr, width, height, out);
}

bool dnnBackendAvailable() { return true; }

}  // namespace pose
