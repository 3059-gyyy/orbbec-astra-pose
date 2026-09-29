// ============================================================================
//  src/opencv_source.cpp  —  USB 摄像头 / 视频文件 / 图片 数据源（回退与离线调试）
//
//  作用：在没有 Astra+ 或不想接 SDK 时，用普通摄像头把整条「叠加 + 角度」链路跑通。
//  另外提供一条后台录像线程，避免 VideoWriter 阻塞主循环。
// ============================================================================
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "pose/color_source.hpp"

namespace pose {

std::unique_ptr<ColorSource> makeOpenCvColorSource();

namespace {

inline bool hasImageExt(const std::string& p) {
  const size_t dot = p.find_last_of('.');
  if (dot == std::string::npos) return false;
  std::string e = p.substr(dot + 1);
  std::transform(e.begin(), e.end(), e.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return e == "png" || e == "jpg" || e == "jpeg" || e == "bmp" || e == "webp" ||
         e == "tif" || e == "tiff";
}

class OpenCvColorSource final : public ColorSource {
 public:
  ~OpenCvColorSource() override { close(); }

  bool open(const AppConfig& cfg) override {
    cfg_ = cfg;
    error_.clear();

    if (!cfg.videoPath.empty() && hasImageExt(cfg.videoPath)) {
      still_ = cv::imread(cfg.videoPath, cv::IMREAD_COLOR);
      if (still_.empty()) {
        error_ = "无法读取图片: " + cfg.videoPath;
        return false;
      }
      isImage_ = true;
      return true;
    }

    const bool fromFile = !cfg.videoPath.empty();
    const int idx = fromFile ? cfg.videoPath.size() ? 0 : cfg.cameraIndex : cfg.cameraIndex;
    if (fromFile) {
      cap_.open(cfg.videoPath);
    } else {
      // 后端选择：auto 时先 DSHOW（普通 UVC 最稳），再 OBSENSOR，最后默认。
      // 也可以显式指定：--backend dshow|obsensor|msmf|any
      std::string want = cfg.captureBackend;
      std::transform(want.begin(), want.end(), want.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

      struct Cand {
        int api;
        const char* tag;
      };
      std::vector<Cand> cands;
      if (want == "obsensor") {
        cands = {{cv::CAP_OBSENSOR, "OBSENSOR"}};
      } else if (want == "dshow") {
        cands = {{cv::CAP_DSHOW, "DSHOW"}};
      } else if (want == "msmf") {
        cands = {{cv::CAP_MSMF, "MSMF"}};
      } else if (want == "any" || want == "default") {
        cands = {{cv::CAP_ANY, "ANY"}};
      } else {  // auto
        cands = {{cv::CAP_DSHOW, "DSHOW"}, {cv::CAP_OBSENSOR, "OBSENSOR"},
                 {cv::CAP_ANY, "ANY"}};
      }

      for (const Cand& c : cands) {
        try {
          if (cap_.open(idx, c.api)) {
            openedVia_ = c.tag;
            break;
          }
        } catch (const std::exception& e) {
          std::fprintf(stderr, "[source] 后端 %s 打开失败: %s\n", c.tag, e.what());
        } catch (...) {
          std::fprintf(stderr, "[source] 后端 %s 打开异常\n", c.tag);
        }
        if (cap_.isOpened()) break;
      }
      if (cap_.isOpened()) {
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, cfg.colorWidth);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.colorHeight);
        cap_.set(cv::CAP_PROP_FPS, cfg.fps);
        std::printf("[source] 已用 %s 打开 camera %d\n", openedVia_.c_str(), idx);
      }
    }
    if (!cap_.isOpened()) {
      error_ = fromFile ? ("无法打开视频文件: " + cfg.videoPath)
                        : ("无法打开摄像头索引 " + std::to_string(idx) +
                           "（检查是否被其他程序占用；可用 --list 查看哪些索引可用）");
      return false;
    }

    fromFile_ = fromFile;
    frameIndex_ = 0;
    return true;
  }

  bool grab(ColorFrame& color, DepthFrame& depth) override {
    cv::Mat bgr;
    if (isImage_) {
      if (!still_.empty()) {
        still_.copyTo(bgr);
      } else {
        return false;
      }
    } else {
      if (!cap_.read(bgr) || bgr.empty()) {
        if (fromFile_ && cfg_.loopVideo) {
          cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
          if (!cap_.read(bgr) || bgr.empty()) return false;
        } else {
          return false;
        }
      }
    }

    if (bgr.type() != CV_8UC3) cv::cvtColor(bgr, bgr, cv::COLOR_GRAY2BGR);

    color.width = bgr.cols;
    color.height = bgr.rows;
    color.stride = bgr.cols * 3;
    color.timestampUs = static_cast<uint64_t>(++frameIndex_) * 33333ull;
    color.bgr.resize(static_cast<size_t>(color.stride) * color.height);
    // 逐行拷贝，兼容 OpenCV 的 padding
    for (int y = 0; y < bgr.rows; ++y) {
      std::memcpy(color.bgr.data() + static_cast<size_t>(y) * color.stride,
                  bgr.ptr(y), static_cast<size_t>(color.stride));
    }

    if (cfg_.drawDepthPeek) {
      // 没有真实深度：用模糊灰度图冒充深度小窗，保持 UI 一致
      cv::Mat small, gray;
      cv::resize(bgr, small, cv::Size(160, 120));
      cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
      cv::GaussianBlur(gray, gray, cv::Size(15, 15), 0);
      depth.width = gray.cols;
      depth.height = gray.rows;
      depth.timestampUs = color.timestampUs;
      depth.mm.assign(static_cast<size_t>(gray.cols) * gray.rows, 0);
      depth.alignedBgr.assign(static_cast<size_t>(gray.cols) * gray.rows * 3, 0);
      cv::Mat viz(gray.rows, gray.cols, CV_8UC3, depth.alignedBgr.data());
      cv::cvtColor(gray, viz, cv::COLOR_GRAY2BGR);
    } else {
      depth = DepthFrame{};
    }
    return true;
  }

  void close() override {
    stopWriter();
    if (cap_.isOpened()) cap_.release();
    still_.release();
  }

  const char* name() const override {
    if (isImage_) return "image";
    if (fromFile_) return "video";
    if (!openedVia_.empty()) return openedVia_.c_str();
    return "camera";
  }
  std::string error() const override { return error_; }

  Intrinsics colorIntrinsics() const override {
    // 普通摄像头的内参未知：按 60° 水平 FOV 粗略估计，仅用于兜底投影
    Intrinsics in;
    in.fx = 0.5f * cfg_.colorWidth / std::tan(30.f * 3.14159265f / 180.f);
    in.fy = in.fx;
    in.cx = cfg_.colorWidth * 0.5f;
    in.cy = cfg_.colorHeight * 0.5f;
    in.valid = true;
    return in;
  }

  // 可选：后台保存带叠加的结果视频（避免阻塞主循环）
  void pushFrame(const cv::Mat& bgr) {
    if (!cfg_.saveAnnotatedVideo) return;
    std::lock_guard<std::mutex> lk(writerMtx_);
    if (!writerOpen_) {
      writer_.open(cfg_.outputVideoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                   cfg_.fps > 0 ? cfg_.fps : 25,
                   cv::Size(bgr.cols, bgr.rows));
      writerOpen_ = writer_.isOpened();
      if (!writerOpen_) return;
    }
    writer_.write(bgr);
  }

 private:
  void stopWriter() {
    std::lock_guard<std::mutex> lk(writerMtx_);
    if (writerOpen_) {
      writer_.release();
      writerOpen_ = false;
    }
  }

  AppConfig cfg_{};
  cv::VideoCapture cap_;
  cv::Mat still_;
  bool isImage_ = false;
  bool fromFile_ = false;
  uint64_t frameIndex_ = 0;
  std::string error_;
  std::string openedVia_;

  std::mutex writerMtx_;
  cv::VideoWriter writer_;
  bool writerOpen_ = false;
};

}  // namespace

std::unique_ptr<ColorSource> makeOpenCvColorSource() {
  return std::unique_ptr<ColorSource>(new OpenCvColorSource());
}

}  // namespace pose
