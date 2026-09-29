// ============================================================================
//  src/camera_pipeline.hpp  —  采集 + 推理线程
//
//  职责：
//    * 用 OpenCV 打开相机（可开关、可切换设备与分辨率）
//    * 取彩色帧；若开启深度选项，用 OpenCV 的 obsensor 通道尝试取深度
//    * 若开启骨骼检测，对彩色帧跑 ONNX 姿态模型，得到 18 关节骨架 + 角度
//    * 结果放在双缓冲里，界面线程 lock 后取最新一帧
//
//  线程安全：options 用互斥量保护；Sample 用双缓冲 + 互斥量交换。
// ============================================================================
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "sample.hpp"

namespace ui {

class CameraPipeline {
 public:
  CameraPipeline();
  ~CameraPipeline();

  // 启动/停止后台线程
  void start();
  void stop();

  // ---- 界面侧接口 --------------------------------------------------------
  // 修改参数（线程安全）。返回时参数已生效（相机重开等重活由后台线程处理）
  void updateOptions(const PipelineOptions& o);
  PipelineOptions options() const;

  // 相机开关与设备热切换：置为 true/false 后后台线程会重开或关闭相机
  void requestCamera(bool on, int index);
  bool cameraOpen() const { return cameraOpen_.load(); }
  std::string cameraError() const;

  // 取最新一帧：拷贝到 out。返回 false 表示还没有可用帧。
  bool fetchLatest(Sample& out) const;

  // 临时暂停/恢复采集：设备扫描要独占相机逐个探测，若与采集线程同时访问
  // 同一台设备会互相抢帧（实测会造成空帧，甚至触发 OpenCV 断言终止进程）
  void setPaused(bool p) { paused_.store(p); }
  bool paused() const { return paused_.load(); }
  // 等待采集线程真正让出相机（最多等 timeoutMs 毫秒）
  bool waitUntilPaused(int timeoutMs = 3000) const;

  // 让后台线程重新打开相机（例如换了分辨率/后端）
  void requestReopen() { reopenRequested_.store(true); }

  // 统计：采集线程平均帧率
  double fps() const { return fps_.load(); }

 private:
  void threadMain();
  bool openCameraLocked();
  void closeCameraLocked();

  mutable std::mutex optMtx_;
  PipelineOptions opt_;

  std::atomic<bool> running_{false};
  std::atomic<bool> cameraWanted_{false};
  std::atomic<bool> cameraOpen_{false};
  std::atomic<bool> reopenRequested_{false};
  std::atomic<double> fps_{0.0};
  std::atomic<bool> paused_{false};

  mutable std::mutex sampleMtx_;
  Sample latest_;
  bool hasLatest_ = false;

  mutable std::mutex errMtx_;
  std::string cameraError_;

  std::thread worker_;

  struct Impl;                  // OpenCV 与模型句柄（避免头文件里包含 OpenCV）
  std::unique_ptr<Impl> impl_;
};

}  // namespace ui
