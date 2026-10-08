// ============================================================================
//  src/capture.hpp  —  截图与录像
//
//  功能：
//    * 截图：把当前画面（含骨骼叠加等所有已绘制内容）存为 PNG
//    * 录像：把当前画面逐帧写入 AVI（MJPG 编码，免额外编解码器即可播放）
//
//  文件命名：按时间戳自动命名，避免覆盖，例如
//      截图  snapshots\snapshot_20260929_181530.png
//      录像  videos\recording_20260929_181530.avi
//
//  目录结构（相对于 pose_ui.exe）：
//      captures\
//        snapshots\   <- 截图
//        videos\      <- 录像
// ============================================================================
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace ui {

struct CaptureConfig {
  std::string rootDir = "captures";   // 总目录
  std::string snapshotSub = "snapshots";
  std::string videoSub = "videos";
  std::string imageExt = ".png";
  double videoFps = 25.0;             // 录像帧率（界面显示帧率通常 9-13，25 更通用）
  int jpegQuality = 95;               // 若扩展名为 .jpg 时的质量
};

// 截图结果
struct SnapshotResult {
  bool ok = false;
  std::string path;      // 实际保存路径
  std::string message;   // 给界面显示的一句话
};

// 一张当前画面存成图片（frame 为 BGR，允许为空 -> 返回失败说明）
SnapshotResult saveSnapshot(const cv::Mat& frame, const CaptureConfig& cfg);

// ---- 录像器 ---------------------------------------------------------------
class VideoRecorder {
 public:
  // 开始录制（用首帧决定分辨率）；已在录制中会先停止
  bool start(const cv::Mat& firstFrame, const CaptureConfig& cfg, std::string* message);
  // 追加一帧；尺寸变化时自动忽略并计数
  void addFrame(const cv::Mat& frame);
  // 停止并收尾（写入索引）；返回输出文件路径
  std::string stop();

  bool recording() const { return recording_; }
  int frameCount() const { return frames_; }
  double seconds() const;        // 录制中=已录真实时长；结束后=本次录制总时长
  const std::string& path() const { return path_; }
  const std::string& lastError() const { return error_; }
  int skipped() const { return skipped_; }
  double targetFps() const { return fps_; }        // 目标帧率上限
  double fileFps() const { return fileFps_; }      // 实际写入文件的帧率
  double measuredSrcFps() const { return measuredSrcFps_; }   // 实测采集帧率
  int pendingCount() const { return static_cast<int>(pendingFrames_.size()); }
  // 已写入文件的真实字节数（用于避免 AVI 触到 2GB 上限）。
  // 注意：不能按"字节/像素"估算——实测同一分辨率下帧大小可能相差千倍
  // （合成画面约 5KB/帧，实拍约 1.4MB/帧），估算会导致误判提前停止。
  long long writtenBytes() const;

 private:
  cv::VideoWriter writer_;
  int width_ = 0;    // 录制尺寸：自己记录，VideoWriter::get 在写入端返回 0
  int height_ = 0;
  bool recording_ = false;
  int frames_ = 0;
  int skipped_ = 0;
  double fps_ = 25.0;
  double fileFps_ = 25.0;   // 实际写入文件的帧率（结束时按真实速率确定）
  double lastSeconds_ = 0.0; // 上一次录制的总时长（停止后仍可读取）
  // 时间轴策略（避免"6 秒录成 24 秒"这类问题）：
  //   录制期间只把原始帧缓存进内存（不写盘），结束时一次性写盘，并把文件帧率
  //   设为"总帧数 / 真实录制时长"——文件帧率与真实节奏天然一致，回放时长因此
  //   与真实时长吻合。若内容速率高于目标帧率则均匀抽帧（跳帧可接受）。
  //   之所以不"边测边写"：MJPEG 编码与落盘会拖慢主循环，写盘阶段的帧率
  //   明显低于空转阶段（实测 46fps -> 29fps），先测后写必然高估、回放被拉长。
  std::chrono::steady_clock::time_point startTp_{};
  std::chrono::steady_clock::time_point lastFrameTp_{};   // 最近一帧到达时刻
  double measuredSrcFps_ = 0.0;                         // 实测内容速率
  std::vector<cv::Mat> pendingFrames_;                  // 录制期间缓存的原始帧
  std::string path_;
  std::string error_;
};

// 生成带时间戳的文件名（不含目录），例如 snapshot_20260929_181530.png
std::string timestampedName(const std::string& prefix, const std::string& ext);

// 保证目录存在（支持多级）
bool ensureDirectory(const std::string& dir);

}  // namespace ui
