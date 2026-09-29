// ============================================================================
//  pose/color_source.hpp  —  彩色/深度数据源抽象
//
//  实现：
//    * OrbbecColorSource     —— Orbbec SDK v2（ob::Pipeline），Astra+ 原生彩色/深度
//    * NuitrackColorSource   —— Nuitrack 的 ColorSensor（与骨骼内外参天然对齐）
//    * OpenCvColorSource     —— USB 摄像头 / 视频文件 / 图片（回退与离线调试）
//    * MockColorSource       —— 程序生成的测试画面（无相机也能验证整条叠加链路）
//
//  创建顺序：先 makePoseProvider()，再 makeColorSource(cfg, provider.get())。
//  原因见 color_source.cpp 顶部注释（设备独占）。
// ============================================================================
#pragma once

#include <memory>
#include <string>

#include "pose/types.hpp"

namespace pose {

class PoseProvider;  // 前置声明：Nuitrack 数据源需要复用骨骼后端内部的彩色帧

class ColorSource {
 public:
  virtual ~ColorSource() = default;

  // 打开设备/文件。失败时返回 false，并通过 error() 给出可读原因。
  virtual bool open(const AppConfig& cfg) = 0;

  // 读取一帧。返回 false 表示流结束（视频播完且不循环）或发生不可恢复错误。
  // 实现必须把彩色帧写成紧凑 BGR8（stride == width*3）。
  virtual bool grab(ColorFrame& color, DepthFrame& depth) = 0;

  virtual void close() = 0;

  virtual const char* name() const = 0;
  virtual std::string error() const = 0;

  // 相机内参（能拿到就返回）。用于把 3D 关节手工投影到彩色图；
  // 拿不到时上层直接使用骨骼后端给出的 2D 坐标。
  struct Intrinsics {
    float fx = 0.f;
    float fy = 0.f;
    float cx = 0.f;
    float cy = 0.f;
    bool valid = false;
  };
  virtual Intrinsics colorIntrinsics() const { return Intrinsics{}; }
};

// 工厂：按 cfg.source 构造数据源。provider 可为 nullptr（mock/opencv/orbbec 不需要）。
std::unique_ptr<ColorSource> makeColorSource(const AppConfig& cfg,
                                             PoseProvider* provider = nullptr);

const char* sourceKindName(AppConfig::SourceKind k);
bool parseSourceKind(const std::string& s, AppConfig::SourceKind& out);

}  // namespace pose
