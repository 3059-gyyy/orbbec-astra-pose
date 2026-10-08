// ============================================================================
//  src/capture.cpp  —  截图与录像实现
// ============================================================================
#include "capture.hpp"

#include <cstdio>
#include <ctime>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#if defined(_WIN32)
#include <direct.h>
#include <sys/stat.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

namespace ui {
namespace {

// 录制期间的内存缓存上限（约 60 秒 @ 25fps）。超出后丢弃后续帧，
// 以免长录制把内存吃满；正常使用时远达不到这个量。
constexpr int kMaxBufferedFrames = 1500;

int fourcc(char a, char b, char c, char d) {
  return cv::VideoWriter::fourcc(a, b, c, d);
}

}  // namespace

bool ensureDirectory(const std::string& dir) {
  if (dir.empty()) return false;
  std::string cur;
  for (size_t i = 0; i < dir.size(); ++i) {
    const char c = dir[i];
    cur.push_back(c);
    const bool last = (i + 1 == dir.size());
    if (c == '/' || c == '\\' || last) {
      if (cur == "/" || cur == "\\" || cur.size() <= 1) continue;
      std::string d = cur;
      while (!d.empty() && (d.back() == '/' || d.back() == '\\')) d.pop_back();
      if (d.empty() || d.size() == 2 && d[1] == ':') continue;
#if defined(_WIN32)
      _mkdir(d.c_str());
#else
      mkdir(d.c_str(), 0755);
#endif
    }
  }
  return true;
}

std::string timestampedName(const std::string& prefix, const std::string& ext) {
  const std::time_t now = std::time(nullptr);
  std::tm tmv{};
#if defined(_WIN32)
  localtime_s(&tmv, &now);
#else
  localtime_r(&now, &tmv);
#endif
  char buf[128];
  std::snprintf(buf, sizeof(buf), "%s_%04d%02d%02d_%02d%02d%02d%s", prefix.c_str(),
                tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min,
                tmv.tm_sec, ext.c_str());
  return buf;
}

SnapshotResult saveSnapshot(const cv::Mat& frame, const CaptureConfig& cfg) {
  SnapshotResult r;
  if (frame.empty()) {
    r.message = "截图失败：当前没有可用画面";
    return r;
  }
  const std::string dir = cfg.rootDir + "/" + cfg.snapshotSub;
  ensureDirectory(dir);

  const std::string name = timestampedName("snapshot", cfg.imageExt);
  const std::string full = dir + "/" + name;

  // 统一成 BGR 三通道，避免灰度/深度图存出来异常
  cv::Mat out = frame;
  if (out.type() != CV_8UC3) {
    if (out.channels() == 1) {
      cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
    } else if (out.type() == CV_8UC4) {
      cv::cvtColor(out, out, cv::COLOR_BGRA2BGR);
    }
  }

  std::vector<int> params;
  if (cfg.imageExt == ".jpg" || cfg.imageExt == ".jpeg") {
    params = {cv::IMWRITE_JPEG_QUALITY, cfg.jpegQuality};
  }
  bool ok = false;
  try {
    ok = cv::imwrite(full, out, params);
  } catch (const std::exception& e) {
    r.message = std::string("截图异常：") + e.what();
    return r;
  } catch (...) {
    r.message = "截图异常：未知错误";
    return r;
  }

  r.ok = ok;
  r.path = full;
  r.message = ok ? ("已截图 -> " + full) : ("截图失败（无法写入 " + full + "）");
  return r;
}

long long VideoRecorder::writtenBytes() const {
  if (path_.empty()) return 0;
#if defined(_WIN32)
  struct _stat64 st;
  if (_stat64(path_.c_str(), &st) == 0) return static_cast<long long>(st.st_size);
#else
  struct stat st;
  if (stat(path_.c_str(), &st) == 0) return static_cast<long long>(st.st_size);
#endif
  return 0;
}

double VideoRecorder::seconds() const {
  // 录制中用真实经过时间；停止后返回本次录制总时长（供界面显示）
  if (!recording_) return lastSeconds_;
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - startTp_)
      .count();
}

bool VideoRecorder::start(const cv::Mat& firstFrame, const CaptureConfig& cfg,
                          std::string* message) {
  if (recording_) stop();
  error_.clear();
  skipped_ = 0;
  frames_ = 0;
  path_.clear();
  pendingFrames_.clear();
  measuredSrcFps_ = 0.0;

  if (firstFrame.empty()) {
    error_ = "当前没有可用画面";
    if (message) *message = "录制失败：" + error_;
    return false;
  }

  const std::string dir = cfg.rootDir + "/" + cfg.videoSub;
  ensureDirectory(dir);
  path_ = dir + "/" + timestampedName("recording", ".avi");

  // 目标帧率只作为上限：真正的文件帧率由"实测采集帧率"决定，
  // 这样文件里保留的是原始帧（允许跳帧），而不是靠复制帧撑出来的时长。
  fps_ = cfg.videoFps > 1.0 ? cfg.videoFps : 25.0;
  fileFps_ = fps_;

  // 计时基准
  startTp_ = std::chrono::steady_clock::now();
  lastFrameTp_ = startTp_;

  cv::Mat frame = firstFrame;
  if (frame.type() != CV_8UC3) {
    if (frame.channels() == 1) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    } else if (frame.type() == CV_8UC4) {
      cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
    }
  }
  width_ = frame.cols;
  height_ = frame.rows;

  // 先不建文件：用约 1.2 秒测量真实采集帧率，再按该帧率建立视频。
  // 早先直接按目标 25fps 建文件、再用重复帧补齐，会出现"原始帧很少、
  // 重复帧很多"的情况；现在改为按实测帧率写入，保留原始帧。
  pendingFrames_.push_back(frame.clone());
  recording_ = true;
  if (message) {
    *message = "开始录制（正在测量实际帧率）-> " + path_;
  }
  return true;
}

// 打开视频文件（测量结束后调用），用实测帧率作为文件帧率
void VideoRecorder::addFrame(const cv::Mat& frame) {
  if (!recording_ || frame.empty()) return;
  cv::Mat f = frame;
  if (f.type() != CV_8UC3) {
    if (f.channels() == 1) {
      cv::cvtColor(f, f, cv::COLOR_GRAY2BGR);
    } else if (f.type() == CV_8UC4) {
      cv::cvtColor(f, f, cv::COLOR_BGRA2BGR);
    }
  }

  // 录制期间只缓存原始帧，不写盘：
  //   * MJPEG 编码与落盘会拖慢主循环（实测写入阶段帧率从 46fps 掉到 29fps），
  //     所以"一边测一边写"无法得到可信的帧率——测出来的值总比实际写入时高。
  //   * 改为结束时按"总帧数 / 真实时长"一次性写盘，帧率与实际写入速率天然一致，
  //     回放时长因此与真实时长吻合（跳帧可接受，不做重复帧填充）。
  if (width_ <= 0 || height_ <= 0) {
    width_ = f.cols;
    height_ = f.rows;
  }
  if (f.cols != width_ || f.rows != height_) {
    ++skipped_;   // 中途改了视图/分辨率，跳过以保证文件一致
    return;
  }

  if (static_cast<int>(pendingFrames_.size()) < kMaxBufferedFrames) {
    pendingFrames_.push_back(f.clone());
    ++frames_;
  } else {
    ++skipped_;   // 超出缓存上限（超长录制）后不再追加
  }
  lastFrameTp_ = std::chrono::steady_clock::now();
}
std::string VideoRecorder::stop() {
  if (!recording_) return path_;
  const auto now = std::chrono::steady_clock::now();
  lastSeconds_ = std::chrono::duration<double>(now - startTp_).count();
  recording_ = false;

  if (pendingFrames_.empty()) {
    return path_;
  }

  // 文件帧率 = 总帧数 / 真实录制时长：与"实际写入速率"天然一致，
  // 回放时长因此与真实时长吻合。上限不超过用户设定的目标帧率。
  const double realFps = lastSeconds_ > 0.05
                             ? static_cast<double>(pendingFrames_.size()) / lastSeconds_
                             : fps_;
  measuredSrcFps_ = realFps;

  // 文件帧率目标：不超过用户设定值。若真实内容速率高于目标，
  // 就"均匀抽帧"降到目标帧率（跳帧可接受），这样时长仍然对齐；
  // 否则直接按真实速率写，时长同样对齐。
  fileFps_ = std::min(fps_, std::max(1.0, realFps));
  const int want = std::max(1, static_cast<int>(lastSeconds_ * fileFps_ + 0.5));
  if (static_cast<int>(pendingFrames_.size()) > want) {
    std::vector<cv::Mat> picked;
    picked.reserve(static_cast<size_t>(want));
    const size_t total = pendingFrames_.size();
    for (int i = 0; i < want; ++i) {
      const size_t src =
          std::min(total - 1, static_cast<size_t>(static_cast<double>(i) * total / want));
      picked.push_back(pendingFrames_[src]);
    }
    std::printf("[rec] 内容速率 %.1f fps 高于目标 %.1f fps：均匀抽帧 %zu -> %d 帧\n",
                realFps, fps_, total, want);
    std::fflush(stdout);
    pendingFrames_.swap(picked);
  }

  bool ok = false;
  try {
    ok = writer_.open(path_, fourcc('M', 'J', 'P', 'G'), fileFps_,
                      cv::Size(width_, height_), true);
    if (!ok) ok = writer_.open(path_, 0, fileFps_, cv::Size(width_, height_), true);
  } catch (const std::exception& e) {
    error_ = e.what();
  } catch (...) {
    error_ = "未知异常";
  }
  if (!ok) {
    error_ = error_.empty() ? "无法创建视频文件（检查目录权限/磁盘空间）" : error_;
    pendingFrames_.clear();
    return path_;
  }

  int written = 0;
  for (const auto& f : pendingFrames_) {
    try {
      writer_.write(f);
      ++written;
    } catch (...) {
      ++skipped_;
    }
  }
  try {
    writer_.release();
  } catch (...) {
  }
  pendingFrames_.clear();
  frames_ = written;

  std::printf("[rec] 录制 %.2f 秒 / %d 帧 -> 文件帧率 %.2f fps（真实速率，无重复填充）\n",
              lastSeconds_, written, fileFps_);
  std::fflush(stdout);
  return path_;
}
}  // namespace ui
