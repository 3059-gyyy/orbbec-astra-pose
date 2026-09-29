// ============================================================================
//  src/video_device.hpp  —  摄像头设备枚举（按唯一序列号聚合）
//
//  为什么不是"一个索引一个设备"：
//    同一台奥比中光相机在 Windows 上会注册成多条独立的 USB 链路，例如
//      USB\VID_2BC5&PID_0536\BY1N73300F8   -> RGB 彩色相机
//      USB\VID_2BC5&PID_0636\BY1N73300F8   -> 深度 + IR（同属一个复合设备）
//    DirectShow 会把它们各自列成"一个视频输入设备"，因此直接按索引展示会出现
//    "同一台相机有多个设备号"的困扰。
//
//  处理办法：
//    * 从 moniker 的 DevicePath 里解析出**序列号（SN）**
//    * 以 SN 为唯一标识聚合成一台物理设备
//    * 每台设备下挂若干"通道"（彩色 / 深度 / 红外），每个通道对应一个 OpenCV 索引
//  这样下拉框里一台设备只出现一次，通道在设备内部选择。
// ============================================================================
#pragma once

#include <string>
#include <vector>

namespace ui {

// 设备的一个可用通道（对应一个 OpenCV 采集索引）
struct DeviceChannel {
  int index = -1;              // OpenCV 索引
  std::string friendlyName;    // DirectShow 友好名
  std::string kind;            // rgb / depth / ir / control / unknown
  bool openable = false;       // 能否打开
  bool deliversFrames = false; // 能否取到帧
  int width = 0;
  int height = 0;
  double firstFrameMean = 0.0; // 首帧亮度均值（黑帧多为深度/IR）
  std::string note;
};

// 一台物理设备（按 SN 聚合）
struct PhysicalDevice {
  std::string serial;          // 唯一标识（VID&PID[+SN]，用于聚合与去重）
  std::string realSerial;      // 硬件真序列号（内部使用，界面不展示）
  std::string displayName;     // 界面展示的设备名称（如 "Orbbec Astra+"）
  std::string pid;             // 设备 PID（如 "0536"）：仅在名称重复时用于区分
  std::string vendor;          // 厂商（从名称推断）
  bool isOrbbec = false;
  std::vector<DeviceChannel> channels;   // 该设备的所有通道

  // 便捷查询：找到首选彩色通道（优先 rgb 且能取帧）
  const DeviceChannel* preferredColorChannel() const;
  const DeviceChannel* findChannel(const std::string& kind) const;
};

// 只枚举名称与 SN（不打开设备），用于快速构建设备树
std::vector<PhysicalDevice> enumeratePhysicalDevices();

// 对给定设备树逐个通道做打开/取帧实测（较慢，每个通道约 1-3 秒）
void probePhysicalDevices(std::vector<PhysicalDevice>& devices, int maxIndex = 8);

// ---------------------------------------------------------------------------
//  设备占用标记（避免"自己抢自己的相机"）
//
//  问题背景：采集线程正打开着某台相机时，设备扫描又去逐个索引 open()，
//  两个消费者同时访问同一设备会互相抢帧 —— 表现为画面反复出现设备刚开机
//  时的暖机帧（看起来像在播放一段固定画面），甚至产生空帧。
//  因此：采集线程打开相机时登记占用，扫描时跳过被占用的索引。
// ---------------------------------------------------------------------------
void markDeviceInUse(int index, bool inUse);
bool isDeviceInUse(int index);

// 把名称拆成"基础名 + 通道类型"，例如
//   "Orbbec Astra+ RGB Camera"   -> base="Orbbec Astra+", kind="rgb"
//   "Orbbec Astra+ Depth Camera" -> base="Orbbec Astra+", kind="depth"
void splitChannelName(const std::string& friendlyName, std::string* base,
                      std::string* kind);

}  // namespace ui
