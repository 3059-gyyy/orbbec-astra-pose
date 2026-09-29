// ============================================================================
//  src/camera_pipeline.cpp  —  采集 + 姿态推理线程实现
// ============================================================================
#include "camera_pipeline.hpp"
#include "video_device.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "pose/angles.hpp"
#include "pose/pose_provider.hpp"

namespace ui {

const char* depthColorMapName(DepthColorMap m) {
  switch (m) {
    case DepthColorMap::Turbo: return "Turbo";
    case DepthColorMap::Jet: return "Jet";
    case DepthColorMap::Viridis: return "Viridis";
    case DepthColorMap::Gray: return "Gray";
    default: return "?";
  }
}

const char* viewModeName(ViewMode m) {
  switch (m) {
    case ViewMode::Color: return "彩色图像";
    case ViewMode::Depth: return "深度图";
    case ViewMode::PointCloud: return "点云三维";
    case ViewMode::SkeletonOnly: return "骨骼单独输出";
    case ViewMode::SkeletonCoords: return "骨骼坐标 (X,Y)";
    default: return "?";
  }
}

// ---------------------------------------------------------------------------
//  OpenCV / 模型句柄
// ---------------------------------------------------------------------------
struct CameraPipeline::Impl {
  cv::VideoCapture cap;
  bool usingObsensor = false;                   // 当前是否走 obsensor（可同时拿深度）
  bool hasIntrinsics = false;                   // 是否取到相机内参（点云必需）
  double fx = 0, fy = 0, icx = 0, icy = 0;
  std::unique_ptr<pose::PoseProvider> pose;      // DNN 姿态后端
  std::string loadedModel;
  bool poseReady = false;

  // 复用缓冲，避免每帧分配
  cv::Mat bgr;
  cv::Mat depthRaw;
};

CameraPipeline::CameraPipeline() : impl_(new Impl()) {
  opt_.modelPath = "models/yolov8n-pose.onnx";
}
CameraPipeline::~CameraPipeline() { stop(); }

void CameraPipeline::start() {
  if (running_.exchange(true)) return;
  worker_ = std::thread([this] { threadMain(); });
}

void CameraPipeline::stop() {
  if (!running_.exchange(false)) return;
  if (worker_.joinable()) worker_.join();
}

void CameraPipeline::updateOptions(const PipelineOptions& o) {
  std::lock_guard<std::mutex> lk(optMtx_);
  const bool camChanged = (o.cameraIndex != opt_.cameraIndex) ||
                          (o.width != opt_.width) || (o.height != opt_.height) ||
                          (o.fps != opt_.fps) || (o.captureBackend != opt_.captureBackend);
  opt_ = o;
  if (camChanged) reopenRequested_.store(true);
  cameraWanted_.store(o.cameraOn);
}

PipelineOptions CameraPipeline::options() const {
  std::lock_guard<std::mutex> lk(optMtx_);
  return opt_;
}

void CameraPipeline::requestCamera(bool on, int index) {
  {
    std::lock_guard<std::mutex> lk(optMtx_);
    opt_.cameraOn = on;
    opt_.cameraIndex = index;
  }
  cameraWanted_.store(on);
  reopenRequested_.store(true);
}

std::string CameraPipeline::cameraError() const {
  std::lock_guard<std::mutex> lk(errMtx_);
  return cameraError_;
}

bool CameraPipeline::waitUntilPaused(int timeoutMs) const {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(std::max(0, timeoutMs));
  // 采集线程进入暂停分支后会关闭相机；这里以相机已关闭作为"已让出"的标志
  while (std::chrono::steady_clock::now() < deadline) {
    if (!cameraOpen_.load()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return !cameraOpen_.load();
}

bool CameraPipeline::fetchLatest(Sample& out) const {
  std::lock_guard<std::mutex> lk(sampleMtx_);
  if (!hasLatest_) return false;
  out = latest_;
  return true;
}

bool CameraPipeline::openCameraLocked() {
  PipelineOptions o;
  {
    std::lock_guard<std::mutex> lk(optMtx_);
    o = opt_;
  }

  closeCameraLocked();

  // 打开策略（本机实测结论）：
  //  * obsensor 后端：能同时取彩色 + 深度，并提供相机内参（点云必需），
  //    但它把整台设备当成一个索引（深度/彩色用 retrieve 的数据类型区分）；
  //    Astra+ 明确在 OpenCV 的支持列表内。
  //  * dshow 后端：延迟低、最稳，但拿不到深度与内参。
  //  因此：需要深度时优先 obsensor，否则用 dshow。
  const bool wantDepth = o.depthEnabled;
  int firstApi = cv::CAP_DSHOW;
  if (o.captureBackend == "obsensor") firstApi = cv::CAP_OBSENSOR;
  else if (o.captureBackend == "msmf") firstApi = cv::CAP_MSMF;
  else if (o.captureBackend == "any") firstApi = cv::CAP_ANY;
  // auto 一律先用 dshow（彩色流畅，实测 ~10 FPS + 推理）；
  // 需要深度/点云时由用户显式选 obsensor（深度取流较慢，约 1-2 FPS）。
  else if (o.captureBackend == "auto") firstApi = cv::CAP_DSHOW;

  impl_->usingObsensor = false;

  const auto tryOpen = [&](int api) -> bool {
    try {
      if (impl_->cap.open(o.cameraIndex, api)) return true;
    } catch (const std::exception&) {
    }
    if (impl_->cap.isOpened()) impl_->cap.release();
    return false;
  };

  bool opened = tryOpen(firstApi);
  if (opened && firstApi == cv::CAP_OBSENSOR) {
    impl_->usingObsensor = true;
    // 读内参（失败也无所谓：点云会用备用内参或禁用）
    impl_->fx = impl_->cap.get(cv::CAP_PROP_OBSENSOR_INTRINSIC_FX);
    impl_->fy = impl_->cap.get(cv::CAP_PROP_OBSENSOR_INTRINSIC_FY);
    impl_->icx = impl_->cap.get(cv::CAP_PROP_OBSENSOR_INTRINSIC_CX);
    impl_->icy = impl_->cap.get(cv::CAP_PROP_OBSENSOR_INTRINSIC_CY);
    impl_->hasIntrinsics = impl_->fx > 1.0 && impl_->fy > 1.0;
  }

  if (!opened && wantDepth && firstApi != cv::CAP_DSHOW) {
    // 深度打不开就退回普通彩色，保证界面至少能用
    opened = tryOpen(cv::CAP_DSHOW);
  }
  if (!opened) {
    opened = tryOpen(cv::CAP_ANY);
  }

  if (!opened) {
    std::lock_guard<std::mutex> lk(errMtx_);
    cameraError_ = "无法打开相机索引 " + std::to_string(o.cameraIndex) +
                   "（检查是否被 OrbbecViewer 等程序占用，或换索引/后端）";
    return false;
  }

  if (!impl_->usingObsensor) {
    impl_->cap.set(cv::CAP_PROP_FRAME_WIDTH, o.width);
    impl_->cap.set(cv::CAP_PROP_FRAME_HEIGHT, o.height);
    impl_->cap.set(cv::CAP_PROP_FPS, o.fps);
  }
  impl_->cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

  // 日志：明确记录实际使用的后端与解析出的分辨率，便于排查性能问题
  std::printf("[pipe] camera opened: index=%d backend=%s (%dx%d @ %.0f fps reported)\n",
              o.cameraIndex, impl_->usingObsensor ? "obsensor" : (firstApi == cv::CAP_DSHOW ? "dshow" : "other"),
              static_cast<int>(impl_->cap.get(cv::CAP_PROP_FRAME_WIDTH)),
              static_cast<int>(impl_->cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
              impl_->cap.get(cv::CAP_PROP_FPS));
  std::fflush(stdout);

  // 预热：DSHOW 打开后前几帧往往要等 300~500ms（USB 相机握手 + 自动曝光收敛），
  // 这里先丢弃若干帧，避免界面刚启动时显示成"0.3 FPS"的假象。
  {
    cv::Mat warm;
    for (int i = 0; i < 8; ++i) {
      if (!impl_->cap.grab()) break;
      if (i == 7) impl_->cap.retrieve(warm, impl_->usingObsensor ? cv::CAP_OBSENSOR_BGR_IMAGE
                                                                 : 0);
    }
  }
  {
    std::lock_guard<std::mutex> lk(errMtx_);
    cameraError_.clear();
  }
  return true;
}

void CameraPipeline::closeCameraLocked() {
  if (impl_->cap.isOpened()) {
    impl_->cap.release();
    // 释放占用登记，设备扫描随之可以重新探测这些索引（保险起见清一遍常用范围）
    for (int i = 0; i < 12; ++i) ui::markDeviceInUse(i, false);
  }
}

void CameraPipeline::threadMain() {
  using clock = std::chrono::steady_clock;

  long long readOk = 0, readFails = 0, emptyFrames = 0;
  int perfFrames = 0;
  double perfGrabMs = 0.0;
  auto lastPerfTp = clock::now();
  auto lastFpsTp = clock::now();
  int framesSinceFps = 0;
  double smoothedFps = 0.0;

  // 上一帧的参数快照，用于检测需要热更新的变化
  PipelineOptions last;

  while (running_.load()) {
    // 暂停请求：设备扫描期间让出相机，避免与扫描线程抢同一台设备
    if (paused_.load()) {
      if (impl_->cap.isOpened()) {
      closeCameraLocked();
        cameraOpen_.store(false);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }

    PipelineOptions o;
    {
      std::lock_guard<std::mutex> lk(optMtx_);
      o = opt_;
    }

    // ---- 相机开关 / 热重开 -------------------------------------------------
    const bool want = cameraWanted_.load() && o.cameraOn;
    if (!want) {
      if (impl_->cap.isOpened()) {
      closeCameraLocked();
        cameraOpen_.store(false);
      }
      {
        std::lock_guard<std::mutex> lk(sampleMtx_);
        latest_.note = "相机已关闭";
        latest_.valid = false;
        hasLatest_ = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      last = o;
      continue;
    }

    if (!impl_->cap.isOpened() || reopenRequested_.exchange(false)) {
      if (openCameraLocked()) {
        cameraOpen_.store(true);
        // 登记占用：设备扫描会跳过这个索引，避免两个消费者抢同一台相机
        ui::markDeviceInUse(o.cameraIndex, true);   // 登记占用：扫描会跳过该索引
      } else {
        cameraOpen_.store(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        last = o;
        continue;
      }
    }

    // ---- 模型热加载（路径或阈值变化时重新加载/更新）-------------------------
    const bool modelChanged = (o.modelPath != impl_->loadedModel) ||
                              (o.inputSize != last.inputSize);
    if (o.poseEnabled && (modelChanged || !impl_->pose)) {
      pose::AppConfig pc;
      pc.poseModelPath = o.modelPath;
      pc.poseInputSize = o.inputSize;
      pc.poseScoreThreshold = o.scoreThreshold;
      pc.poseKptThreshold = o.kptThreshold;
      pc.poseMaxPersons = o.maxPersons;
      // 前置自检模型文件：OpenCV 的 DNN 在文件缺失/损坏时会直接触发访问冲突
      // （不是可捕获的 C++ 异常），会把整个进程打崩，必须先拦截。
      bool fileOk = false;
      {
        std::FILE* f = std::fopen(o.modelPath.c_str(), "rb");
        if (f) {
          std::fseek(f, 0, SEEK_END);
          const long sz = std::ftell(f);
          std::fclose(f);
          fileOk = sz > 4096;   // 正常姿态模型都是 MB 级
        }
      }
      if (!fileOk) {
        {
          std::lock_guard<std::mutex> lk(errMtx_);
          cameraError_ = "姿态模型不存在或过小：" + o.modelPath +
                         "\n请保持 models\\yolov8n-pose.onnx 与程序同目录，或在左侧关闭骨骼检测。";
        }
        impl_->poseReady = false;
        impl_->pose.reset();
        impl_->loadedModel = o.modelPath;
        last = o;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;   // 跳过本帧，等文件就绪后自动重试
      }

      impl_->pose = pose::makePoseProviderWithBackend(pc, pose::AppConfig::PoseBackend::Dnn);
      impl_->poseReady = impl_->pose && impl_->pose->start(pc);
      impl_->loadedModel = o.modelPath;
      if (!impl_->poseReady) {
        std::lock_guard<std::mutex> lk(errMtx_);
        cameraError_ = impl_->pose ? impl_->pose->error() : "姿态模型加载失败";
      } else {
        std::lock_guard<std::mutex> lk(errMtx_);
        cameraError_.clear();
      }
    }

    // ---- 取流 -------------------------------------------------------------
    // obsensor：一次 grab，然后分别 retrieve 彩色与深度（同一台设备，天然同源）
    // 其他后端：普通 read 取彩色，没有深度
    const auto t0 = clock::now();
    cv::Mat frame;
    cv::Mat depth;
    bool got = false;
    try {
      if (impl_->usingObsensor) {
        if (impl_->cap.grab()) {
          if (impl_->cap.retrieve(frame, cv::CAP_OBSENSOR_BGR_IMAGE) && !frame.empty()) {
            got = true;
          }
          if (o.depthEnabled) {
            cv::Mat d;
            if (impl_->cap.retrieve(d, cv::CAP_OBSENSOR_DEPTH_MAP) && !d.empty()) {
              if (d.type() == CV_16UC1) {
                depth = d;
              } else {
                d.convertTo(depth, CV_16UC1);
              }
            }
          }
        }
      } else {
        got = impl_->cap.read(frame) && !frame.empty();
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lk(errMtx_);
      cameraError_ = std::string("读取帧异常: ") + e.what();
      got = false;
    }
    if (!got) {
      // 连续读失败：重开一次相机（USB 相机偶发断流很常见）
      ++readFails;
      if (readFails % 10 == 1) {
        std::printf("[pipe] read failed (%d times) backend=%s usingObsensor=%d\n", readFails,
                    impl_->usingObsensor ? "obsensor" : "dshow",
                    impl_->usingObsensor ? 1 : 0);
        std::fflush(stdout);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      closeCameraLocked();
      cameraOpen_.store(false);
      last = o;
      continue;
    }
    ++readOk;
    // 防御：某些后端（尤其 obsensor 被其他进程/扫描线程占用时）会返回空帧，
    // 此时调用 cvtColor 会触发 OpenCV 断言失败并直接终止进程。
    if (frame.empty()) {
      ++emptyFrames;
      if (emptyFrames % 30 == 1) {
        std::printf("[pipe] 空帧（第 %lld 次），跳过本帧\n",
                    static_cast<long long>(emptyFrames));
        std::fflush(stdout);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    if (frame.type() != CV_8UC3) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

    // obsensor 后端的帧行序是"自下而上"的（与 DShow 相反），必须上下翻转，
    // 否则画面上下颠倒，骨骼坐标 Y 也会出现负值。
    if (impl_->usingObsensor && o.flipVertical) {
      cv::flip(frame, frame, 0);
      if (!depth.empty()) cv::flip(depth, depth, 0);
    }

    Sample s;
    s.valid = true;
    s.colorW = frame.cols;
    s.colorH = frame.rows;
    s.bgr.resize(static_cast<size_t>(frame.cols) * frame.rows * 3);
    for (int y = 0; y < frame.rows; ++y) {
      std::memcpy(s.bgr.data() + static_cast<size_t>(y) * frame.cols * 3, frame.ptr(y),
                  static_cast<size_t>(frame.cols) * 3);
    }

    // 深度
    if (!depth.empty()) {
      s.depthW = depth.cols;
      s.depthH = depth.rows;
      s.depthMm.resize(static_cast<size_t>(depth.cols) * depth.rows);
      for (int y = 0; y < depth.rows; ++y) {
        std::memcpy(s.depthMm.data() + static_cast<size_t>(y) * depth.cols,
                    depth.ptr<uint16_t>(y), static_cast<size_t>(depth.cols) * sizeof(uint16_t));
      }
    }

    // 内参

    s.fx = impl_->fx;
    s.fy = impl_->fy;
    s.icx = impl_->icx;
    s.icy = impl_->icy;
    s.hasIntrinsics = impl_->hasIntrinsics;
    s.usingObsensor = impl_->usingObsensor;

    const auto t1 = clock::now();

    // ---- 骨骼推理 ---------------------------------------------------------
    if (o.poseEnabled && impl_->poseReady && impl_->pose) {
      // 更新阈值（这些参数不需要重载模型）
      pose::AppConfig pc;
      pc.poseModelPath = o.modelPath;
      pc.poseInputSize = o.inputSize;
      pc.poseScoreThreshold = o.scoreThreshold;
      pc.poseKptThreshold = o.kptThreshold;
      pc.poseMaxPersons = o.maxPersons;

      pose::PoseFrame pf;
      pf.timestampUs = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count());
      pose::dnnInferOnFrame(impl_->pose.get(), pc, s.bgr.data(), s.colorW, s.colorH, pf);

      // 角度计算
      pose::AppConfig ac;
      ac.anglesFrom3d = o.anglesFrom3d;
      ac.minJointConfidence = o.minJointConfidence;
      s.angles.clear();
      s.angles.reserve(pf.skeletons.size());
      for (const auto& sk : pf.skeletons) {
        s.angles.push_back(pose::computeAngles(sk, ac));
      }
      s.poses = std::move(pf);
    }

    const auto t2 = clock::now();

    s.captureMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    s.inferMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    s.timestampUs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t2.time_since_epoch()).count());

    // ---- FPS 统计（1 秒窗口平滑）-----------------------------------------
    ++framesSinceFps;
    const double winSec = std::chrono::duration<double>(t2 - lastFpsTp).count();
    if (winSec >= 1.0 && framesSinceFps >= 8) {
      const double inst = framesSinceFps / winSec;
      smoothedFps = smoothedFps <= 0.0 ? inst : smoothedFps * 0.6 + inst * 0.4;
      fps_.store(smoothedFps);
      framesSinceFps = 0;
      lastFpsTp = t2;
    }
    s.frameFps = smoothedFps;

    // ---- 每秒打印一次采集统计（便于定位性能问题）------------------------
    ++perfFrames;
    perfGrabMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (perfFrames == 1) lastPerfTp = t1;   // 从第一帧开始计时，跳过相机预热期
    if (perfFrames >= 8) {
      const double sec = std::chrono::duration<double>(t1 - lastPerfTp).count();
      if (sec >= 2.0) {
        std::printf("[pipe] %d frames in %.1fs (%.1f fps) | grab avg %.1f ms | readOk=%lld readFail=%lld | obsensor=%d\n",
                    perfFrames, sec, perfFrames / sec, perfGrabMs / std::max(1, perfFrames),
                    readOk, readFails, impl_->usingObsensor ? 1 : 0);
        std::fflush(stdout);
        perfFrames = 0;
        perfGrabMs = 0.0;
        lastPerfTp = t1;
      }
    }

    // ---- 发布 -------------------------------------------------------------
    {
      std::lock_guard<std::mutex> lk(sampleMtx_);
      latest_ = std::move(s);
      hasLatest_ = true;
    }

    last = o;
    // 轻微让出 CPU，避免占满一个核心影响界面刷新
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

      closeCameraLocked();
  cameraOpen_.store(false);
  if (impl_->pose) impl_->pose->stop();
}

}  // namespace ui
