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
  return fps_ > 0.0 ? static_cast<double>(frames_) / fps_ : 0.0;
}

bool VideoRecorder::start(const cv::Mat& firstFrame, const CaptureConfig& cfg,
                          std::string* message) {
  if (recording_) stop();
  error_.clear();
  skipped_ = 0;
  frames_ = 0;
  path_.clear();

  if (firstFrame.empty()) {
    error_ = "当前没有可用画面";
    if (message) *message = "录制失败：" + error_;
    return false;
  }

  const std::string dir = cfg.rootDir + "/" + cfg.videoSub;
  ensureDirectory(dir);
  path_ = dir + "/" + timestampedName("recording", ".avi");
  fps_ = cfg.videoFps > 1.0 ? cfg.videoFps : 25.0;

  cv::Mat frame = firstFrame;
  if (frame.type() != CV_8UC3) {
    if (frame.channels() == 1) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    } else if (frame.type() == CV_8UC4) {
      cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
    }
  }

  // MJPG：Windows 自带解码器，文件大但绝对能播；打不开时退回默认编码
  bool ok = false;
  try {
    ok = writer_.open(path_, fourcc('M', 'J', 'P', 'G'), fps_,
                      cv::Size(frame.cols, frame.rows), true);
    if (!ok) ok = writer_.open(path_, 0, fps_, cv::Size(frame.cols, frame.rows), true);
  } catch (const std::exception& e) {
    error_ = e.what();
  } catch (...) {
    error_ = "未知异常";
  }

  if (!ok) {
    error_ = error_.empty() ? "无法创建视频文件（检查目录权限/磁盘空间）" : error_;
    if (message) *message = "录制失败：" + error_;
    return false;
  }

  recording_ = true;
  width_ = frame.cols;      // 自己记录尺寸，不要依赖 writer_.get()（写入端返回 0）
  height_ = frame.rows;
  writer_.write(frame);
  ++frames_;
  if (message) *message = "开始录制 -> " + path_;
  return true;
}

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
  // 尺寸与录制开始时不一致（例如切换视图/分辨率）则跳过，避免写出损坏文件。
  // 注意：这里比较的是自己记录的尺寸，不能用 writer_.get()——写入端会返回 0，
  // 那样除首帧外所有帧都会被误判为"尺寸不符"而丢弃。
  if (!writer_.isOpened() || f.cols != width_ || f.rows != height_) {
    ++skipped_;
    return;
  }
  try {
    // 按目标帧率补帧：界面帧率通常只有 9-13fps，而录像目标 25fps，
    // 不补帧的话回放会比真实速度快一倍以上。
    const double ratio = fps_ / std::max(1.0, assumedSourceFps_);
    const int dup = std::max(1, static_cast<int>(ratio + 0.5));
    for (int i = 0; i < dup; ++i) {
      writer_.write(f);
      ++frames_;
    }
  } catch (...) {
    ++skipped_;
  }
}

std::string VideoRecorder::stop() {
  if (!recording_) return path_;
  recording_ = false;
  try {
    writer_.release();
  } catch (...) {
  }
  return path_;
}

}  // namespace ui
