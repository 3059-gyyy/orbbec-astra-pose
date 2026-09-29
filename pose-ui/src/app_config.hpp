// ============================================================================
//  src/app_config.hpp  —  运行配置（config.json）
//
//  作用：让"要不要显示命令行窗口""用哪个设备/视图"等启动项可以不改代码地调整。
//
//  文件位置：与 pose_ui.exe 同目录的 config.json（缺失时使用内置默认值，
//            程序不会因此报错，只会打印一行提示）。
//
//  示例 config.json：
//  {
//    "developerMode": false,       // false=隐藏命令行窗口（默认）；true=显示窗口看日志
//    "cameraIndex": 0,             // 默认相机索引
//    "view": "color",              // 启动视图: color/depth/cloud/skeleton/coords
//    "mirror": true,               // 是否镜像显示
//    "depthStream": false,         // 启动时是否开启深度流
//    "poseEnabled": true,          // 启动时是否开启骨骼检测
//    "poseModel": "models/yolov8n-pose.onnx",
//    "logToFile": false,           // 是否把日志同时写入 pose_ui.log
//    "captureDir": "captures",     // 截图/录像保存目录
//    "videoFps": 25                // 录像帧率
//  }
//
//  说明：为保持零依赖，这里只实现了一个"够用"的 JSON 读取器：
//        支持对象、字符串、数字、true/false，忽略注释与未知字段。
// ============================================================================
#pragma once

#include <string>

namespace ui {

struct AppConfigFile {
  bool developerMode = false;    // 默认关闭：不显示命令行窗口（开发者可置 true 打开）
  bool logToFile = true;         // 默认把运行日志写入 pose_ui.log（隐藏控制台时便于排查）
  int cameraIndex = 0;
  std::string view = "color";    // color / depth / cloud / skeleton / coords
  bool mirror = true;
  bool depthStream = false;
  bool poseEnabled = true;
  std::string poseModel = "models/yolov8n-pose.onnx";
  std::string captureBackend = "auto";
  // 截图/录像保存位置（相对 exe）：captures\snapshots 与 captures\videos
  std::string captureDir = "captures";
  double videoFps = 25.0;        // 录像帧率

  // 加载结果说明：成功 / 未找到文件 / 解析失败
  std::string status = "使用内置默认值";
};

// 读取同目录下的 config.json；文件不存在时返回默认值并说明
AppConfigFile loadAppConfigFile(const std::string& path = "config.json");

// 启动时决定是否保留命令行窗口：
//   wantVisible = true  -> 保留（并确保它不再被隐藏）
//   wantVisible = false -> 分离控制台（窗口关闭，进程继续运行）
void applyConsoleVisibility(bool wantVisible);

}  // namespace ui
