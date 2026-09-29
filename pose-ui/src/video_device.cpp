// ============================================================================
//  src/video_device.cpp  —  设备枚举：解析 SN、按物理设备聚合、逐通道实测
// ============================================================================
#include "video_device.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <map>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#if defined(_WIN32)
#include <windows.h>
#include <cfgmgr32.h>
#include <dshow.h>
#include <initguid.h>
#endif

namespace ui {
namespace {

// ---------------------------------------------------------------------------
//  沿设备树向上找到 USB 父节点，取其真序列号
//
//  为什么必须这样做：同一台奥比中光相机在 Windows 上是两条独立链路
//      RGB  ：USB\VID_2BC5&PID_0536\BY1N73300F8     （独立 USB 设备，带 SN）
//      深度 ：USB\VID_2BC5&PID_0636\BY1N73300F8     （复合设备 MI_00 接口）
//      红外 ：USB\VID_2BC5&PID_0636\BY1N73300F8     （同一复合设备的 MI_02）
//  接口节点（"...&MI_00" 以及 "7&5034BFC&0&0000" 这类实例串）里**没有** SN，
//  只有其父 USB 设备节点才有。因此：接口实例 -> 父节点 -> USB 设备节点 -> 取 SN。
//  取到 SN 后即可把三条链路归并成"一台设备"，SN 本身也能区分同型号的多台相机。
// ---------------------------------------------------------------------------
namespace {
inline std::string toLowerCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}
}  // namespace

std::string usbSerialFromInterfacePath(const std::string& devicePath) {
  if (devicePath.empty()) return {};

  // 1) 接口路径 -> 设备实例 ID：去掉 \\?\ 前缀与结尾的 {GUID}
  std::string inst = devicePath;
  if (inst.rfind("\\\\?\\", 0) == 0) inst = inst.substr(4);
  const size_t brace = inst.find('{');
  if (brace != std::string::npos) inst = inst.substr(0, brace);
  while (!inst.empty() && (inst.back() == '#' || inst.back() == '\\')) inst.pop_back();
  if (inst.empty()) return {};

  // CM_* 需要反斜杠形式的实例 ID（接口路径里是 '#'）
  std::replace(inst.begin(), inst.end(), '#', '\\');
  std::wstring winst(inst.begin(), inst.end());

  DEVINST dev = 0;
  if (CM_Locate_DevNodeW(&dev, const_cast<DEVINSTID_W>(winst.c_str()),
                         CM_LOCATE_DEVNODE_NORMAL) != CR_SUCCESS) {
    return {};
  }

  // 2) 最多向上 6 层，找形如 "USB\VID_xxxx&PID_xxxx\<SN>" 的节点
  for (int depth = 0; depth < 6; ++depth) {
    wchar_t idBuf[MAX_DEVICE_ID_LEN] = {0};
    if (CM_Get_Device_IDW(dev, idBuf, MAX_DEVICE_ID_LEN, 0) != CR_SUCCESS) break;
    const std::wstring wid(idBuf);
    const std::string id(wid.begin(), wid.end());

    if (toLowerCopy(id).rfind("usb\\vid_", 0) == 0) {
      const size_t lastSlash = id.rfind('\\');
      if (lastSlash != std::string::npos && lastSlash + 1 < id.size()) {
        std::string sn = id.substr(lastSlash + 1);
        // "7&5034BFC&0&0000" 这类实例串不是真 SN：取第一个 '&' 之前
        const size_t amp = sn.find('&');
        if (amp != std::string::npos) sn = sn.substr(0, amp);
        bool allDigits = !sn.empty();
        for (char c : sn) {
          if (!std::isdigit(static_cast<unsigned char>(c))) { allDigits = false; break; }
        }
        // 纯数字且极短的是集线器端口号，不足以唯一标识一台设备
        if (!sn.empty() && !(allDigits && sn.size() <= 2)) {
          std::transform(sn.begin(), sn.end(), sn.begin(),
                         [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
          return sn;
        }
      }
    }

    DEVINST parent = 0;
    if (CM_Get_Parent(&parent, dev, 0) != CR_SUCCESS || parent == 0) break;
    dev = parent;
  }
  return {};
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool containsAny(const std::string& hay, const std::vector<std::string>& needles) {
  for (const auto& n : needles) {
    if (hay.find(n) != std::string::npos) return true;
  }
  return false;
}

std::string narrow(const wchar_t* w) {
  if (!w) return {};
#if defined(_WIN32)
  const int need = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
  if (need <= 1) return {};
  std::string out(static_cast<size_t>(need - 1), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w, -1, out.data(), need, nullptr, nullptr);
  return out;
#else
  std::string out;
  for (const wchar_t* p = w; *p; ++p) out.push_back(static_cast<char>(*p));
  return out;
#endif
}

// 从 DevicePath 解析"物理设备唯一标识"。
//
// 实测的三种典型形式（同一台 Astra+ 会出现三种）：
//   \\?\usb#vid_2bc5&pid_0536&mi_00#7&292abf16&0&0000#{...}   彩色（独立 USB 设备）
//   \\?\usb#vid_2bc5&pid_0636&mi_00#7&5034bfc&0&0000#{...}    深度（复合设备接口 0）
//   \\?\usb#vid_2bc5&pid_0636&mi_02#7&5034bfc&0&0002#{...}    红外（复合设备接口 2）
//
// 规则：
//   * 标识 = VID&PID  [+ "/" + 真序列号]
//   * VID&PID 相同 -> 视为**同一台物理设备**（复合设备的不同接口自动归并）
//   * 若路径里带真正的序列号（例如 RGB 那条的 BY1N73300F8），附加进去，
//     这样同型号多台设备也能区分开
//   * "7&5034BFC&0&0000" 这类是 Windows 生成的接口实例串，不含真序列号，忽略
std::string deviceKeyFromPath(const std::string& path, std::string* realSerialOut,
                             std::string* pidOut) {
  const std::string p = lower(path);
  if (realSerialOut) realSerialOut->clear();

  const size_t usb = p.find("usb#");
  if (usb == std::string::npos) {
    size_t h = 1469598103934665603ull;
    for (unsigned char c : path) { h ^= c; h *= 1099511628211ull; }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "PATH-%016llX", static_cast<unsigned long long>(h));
    return buf;
  }

  // --- 1) 取 VID&PID（含 MI_xx 之前的完整 vid/pid 段）---
  std::string vidpid;
  {
    const size_t v = p.find("vid_", usb);
    if (v != std::string::npos) {
      const size_t e = p.find('#', v);
      vidpid = path.substr(v, (e == std::string::npos ? path.size() : e) - v);
      vidpid.erase(std::remove_if(vidpid.begin(), vidpid.end(),
                                  [](unsigned char c) { return c < 0x20 || c == 0x7f; }),
                   vidpid.end());
      std::transform(vidpid.begin(), vidpid.end(), vidpid.begin(),
                     [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
      // 去掉 &MI_xx 后缀，使不同接口归并到同一个 VID&PID
      const size_t mi = vidpid.find("&MI_");
      if (mi != std::string::npos) vidpid = vidpid.substr(0, mi);
    }
  }

  // --- 2) 看接口实例串里有没有"真序列号"---
  //     判据：由 '&' 分隔的各段中，取最长的、非纯数字、长度 >= 6 的那段。
  //     "7&292ABF16&0&0000" -> 各段 7 / 292ABF16(纯hex,8位) / 0 / 0000
  //       其中 292ABF16 是 Windows 生成的地址，不是 SN —— 长度为 8 但含字母，
  //       需要排除：真正的 SN 一般同时含字母和数字且长度 >= 8，且不形如纯十六进制。
  std::string sn;
  {
    const size_t v = p.find("vid_", usb);
    size_t start = std::string::npos;
    if (v != std::string::npos) {
      const size_t h = p.find('#', v);
      if (h != std::string::npos) start = h + 1;
    }
    if (start != std::string::npos) {
      const size_t e = p.find('#', start);
      std::string seg =
          path.substr(start, (e == std::string::npos ? path.size() : e) - start);
      seg.erase(std::remove_if(seg.begin(), seg.end(),
                               [](unsigned char c) { return c < 0x20 || c == 0x7f; }),
                seg.end());
      // 按 '&' 切分，挑选最像序列号的一段
      std::vector<std::string> parts;
      std::string cur;
      for (char c : seg) {
        if (c == '&') { parts.push_back(cur); cur.clear(); }
        else cur.push_back(c);
      }
      if (!cur.empty()) parts.push_back(cur);

      size_t bestLen = 0;
      for (const auto& part : parts) {
        if (part.size() < 6) continue;                 // 端口号/0 这类忽略
        bool hasAlpha = false, hasDigit = false, allHex = true;
        for (char c : part) {
          const unsigned char u = static_cast<unsigned char>(c);
          if (std::isalpha(u)) hasAlpha = true;
          if (std::isdigit(u)) hasDigit = true;
          if (!std::isxdigit(u)) allHex = false;
        }
        if (!hasAlpha || !hasDigit) continue;
        // 纯十六进制（如 292ABF16）通常是总线地址，不是 SN
        if (allHex && part.size() == 8) continue;
        if (part.size() > bestLen) { bestLen = part.size(); sn = part; }
      }
      std::transform(sn.begin(), sn.end(), sn.begin(),
                     [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    }
  }

  if (realSerialOut) *realSerialOut = sn;

  // 顺便取出 PID（形如 "0536"），供界面在设备重名时区分
  if (pidOut) {
    pidOut->clear();
    const size_t pidPos = lower(path).find("pid_");
    if (pidPos != std::string::npos) {
      const std::string raw = path.substr(pidPos + 4, 4);
      for (char c : raw) {
        if (std::isxdigit(static_cast<unsigned char>(c))) {
          pidOut->push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        }
      }
    }
  }
  if (vidpid.empty()) {
    size_t h = 1469598103934665603ull;
    for (unsigned char c : path) { h ^= c; h *= 1099511628211ull; }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "PATH-%016llX", static_cast<unsigned long long>(h));
    return buf;
  }
  return sn.empty() ? vidpid : (vidpid + "/" + sn);
}

}  // namespace

void splitChannelName(const std::string& friendlyName, std::string* base,
                      std::string* kind) {
  const std::string n = lower(friendlyName);
  std::string k = "unknown";
  std::string suffix;

  if (n.find("depth") != std::string::npos) {
    k = "depth";
    suffix = "Depth";
  }
  if (n.find(" ir ") != std::string::npos || n.find("ir camera") != std::string::npos ||
      n.find("infrared") != std::string::npos) {
    k = "ir";
    suffix = "IR";
  }
  if (n.find("rgb") != std::string::npos) {
    k = "rgb";
    suffix = "RGB";
  }
  if (n.find("color") != std::string::npos || n.find("colour") != std::string::npos) {
    if (k == "unknown") { k = "rgb"; suffix = "Color"; }
  }
  if (n.find("data channel") != std::string::npos) {
    k = "control";
    suffix = "Data Channel";
  }

  std::string b = friendlyName;
  if (!suffix.empty()) {
    // 去掉结尾的 "<suffix> Camera" / "<suffix>"
    const std::vector<std::string> tails = {" " + suffix + " Camera", " " + suffix,
                                            "(" + suffix + ")"};
    for (const auto& t : tails) {
      if (b.size() > t.size()) {
        const std::string tail = lower(b.substr(b.size() - t.size()));
        if (tail == lower(t)) {
          b = b.substr(0, b.size() - t.size());
          break;
        }
      }
    }
  }
  // 清理多余空格与末尾连字符
  while (!b.empty() && (b.back() == ' ' || b.back() == '-')) b.pop_back();
  if (base) *base = b;
  if (kind) *kind = k;
}

namespace {
// 占用标记：用位图记录哪些 OpenCV 索引正被采集线程使用
std::mutex g_useMtx;
std::vector<bool> g_inUse;
}  // namespace

void markDeviceInUse(int index, bool inUse) {
  if (index < 0) return;
  std::lock_guard<std::mutex> lk(g_useMtx);
  if (static_cast<size_t>(index) >= g_inUse.size()) {
    g_inUse.resize(static_cast<size_t>(index) + 1, false);
  }
  g_inUse[static_cast<size_t>(index)] = inUse;
}

bool isDeviceInUse(int index) {
  if (index < 0) return false;
  std::lock_guard<std::mutex> lk(g_useMtx);
  return static_cast<size_t>(index) < g_inUse.size() && g_inUse[static_cast<size_t>(index)];
}

std::vector<PhysicalDevice> enumeratePhysicalDevices() {
  std::vector<PhysicalDevice> devices;
#if defined(_WIN32)
  const HRESULT hrInit = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
  const bool needUninit = SUCCEEDED(hrInit);

  ICreateDevEnum* devEnum = nullptr;
  if (SUCCEEDED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                 IID_PPV_ARGS(&devEnum))) &&
      devEnum) {
    IEnumMoniker* enumMoniker = nullptr;
    if (devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumMoniker, 0) == S_OK &&
        enumMoniker) {
      IMoniker* moniker = nullptr;
      int order = 0;   // DirectShow 枚举顺序 == OpenCV 的 CAP_DSHOW 索引
      while (enumMoniker->Next(1, &moniker, nullptr) == S_OK && moniker) {
        std::string friendly;
        std::string devicePath;
        IPropertyBag* bag = nullptr;
        if (SUCCEEDED(moniker->BindToStorage(nullptr, nullptr, IID_PPV_ARGS(&bag))) && bag) {
          VARIANT v;
          VariantInit(&v);
          if (SUCCEEDED(bag->Read(L"FriendlyName", &v, nullptr)) && v.vt == VT_BSTR) {
            friendly = narrow(v.bstrVal);
          }
          VariantClear(&v);
          VariantInit(&v);
          if (SUCCEEDED(bag->Read(L"DevicePath", &v, nullptr)) && v.vt == VT_BSTR) {
            devicePath = narrow(v.bstrVal);
          }
          VariantClear(&v);
          bag->Release();
        }
        moniker->Release();

        std::string base, kind;
        splitChannelName(friendly.empty() ? "(未命名设备)" : friendly, &base, &kind);

        // 过滤掉"数据通道"（Orbbec Data Channel）：它不是视频流，
        // 只是相机的控制接口，无法用于取图；且通道选择已由视图区负责，
        // 在这里列出会造成重复与误选。索引仍然递增，保证与 OpenCV 索引一致。
        if (kind == "control") {
          ++order;
          continue;
        }

        // 唯一标识：优先用"父 USB 设备序列号"，这样同一台相机的
        // RGB / 深度 / IR 三条链路会归并成一台设备
        // （它们的 VID&PID 不同：0536 与 0636，但父设备 SN 相同）。
        std::string pidStr;
        std::string dummySerial;
        if (!devicePath.empty()) deviceKeyFromPath(devicePath, &dummySerial, &pidStr);

        const std::string realSn = usbSerialFromInterfacePath(devicePath);
        std::string sn;
        if (!realSn.empty()) {
          sn = "SN:" + realSn;
        } else if (!devicePath.empty()) {
          sn = deviceKeyFromPath(devicePath, &dummySerial, &pidStr);
        } else {
          sn = "IDX" + std::to_string(order);
        }

        // 按唯一标识聚合
        PhysicalDevice* dev = nullptr;
        for (auto& d : devices) {
          if (d.serial == sn) { dev = &d; break; }
        }
        if (!dev) {
          PhysicalDevice nd;
          nd.serial = sn;
          nd.realSerial = realSn;   // 内部保留：真序列号
          nd.pid = pidStr;          // 界面在重名时用它区分
          nd.displayName = base.empty() ? friendly : base;
          const std::string ln = lower(nd.displayName);
          nd.isOrbbec = containsAny(ln, {"orbbec", "astra", "femto", "gemini", "obsensor"});
          nd.vendor = nd.isOrbbec ? "Orbbec" : "";
          devices.push_back(std::move(nd));
          dev = &devices.back();
        } else if (dev->displayName.size() > base.size() && !base.empty()) {
          // 取更短的那个作为公共基础名（"Orbbec Astra+" 优于 "Orbbec Astra+ RGB"）
          dev->displayName = base;
        }

        DeviceChannel ch;
        ch.index = order;
        ch.friendlyName = friendly;
        ch.kind = kind;
        dev->channels.push_back(std::move(ch));
        ++order;
      }
      enumMoniker->Release();
    }
    devEnum->Release();
  }
  if (needUninit) CoUninitialize();
#endif
  // 清掉没有任何可用通道的设备（只暴露 Data Channel 的条目）
  devices.erase(std::remove_if(devices.begin(), devices.end(),
                               [](const PhysicalDevice& d) { return d.channels.empty(); }),
                devices.end());
  return devices;
}

void probePhysicalDevices(std::vector<PhysicalDevice>& devices, int maxIndex) {
  // 记录已被 DirectShow 名称覆盖的索引，其余索引单独探测后补成"未知设备"。
  // 注意：容量必须同时覆盖 maxIndex 与所有通道索引，否则 used[ch.index] 会越界
  // （这正是之前 -1073741819 访问冲突的原因）。
  int upper = std::max(1, maxIndex);
  for (const auto& dev : devices) {
    for (const auto& ch : dev.channels) {
      if (ch.index + 1 > upper) upper = ch.index + 1;
    }
  }
  std::vector<bool> used(static_cast<size_t>(upper), false);

  for (auto& dev : devices) {
    for (auto& ch : dev.channels) {
      if (ch.index >= 0 && ch.index < static_cast<int>(used.size())) {
        used[static_cast<size_t>(ch.index)] = true;

      // 正在被采集线程使用的设备不重复打开：两边同时访问会互相抢帧，
      // 表现为画面反复出现设备刚开机的暖机帧（看着像在播一段固定画面）。
      if (isDeviceInUse(ch.index)) {
        ch.openable = true;
        ch.deliversFrames = true;
        ch.width = 0;
        ch.height = 0;
        ch.note = "正在使用中（跳过实测）";
        continue;
      }
      }
      cv::VideoCapture cap;
      bool opened = false;
      try {
        opened = cap.open(ch.index, cv::CAP_DSHOW);
        if (!opened) opened = cap.open(ch.index);
      } catch (...) {
        opened = false;
      }
      ch.openable = opened;
      if (!opened) {
        ch.note = "打不开（可能被其他程序占用）";
        continue;
      }
      cv::Mat frame;
      for (int attempt = 0; attempt < 6 && frame.empty(); ++attempt) {
        try {
          cap.read(frame);
        } catch (...) {
          break;
        }
        if (frame.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
      if (!frame.empty()) {
        ch.deliversFrames = true;
        ch.width = frame.cols;
        ch.height = frame.rows;
        ch.firstFrameMean = cv::mean(frame)[0];
        // 名称没给出类型时，用亮度判断：全黑多为深度/IR
        if (ch.kind == "unknown") {
          ch.kind = (ch.firstFrameMean < 3.0) ? "depth/ir" : "rgb";
        }
      } else {
        ch.note = "能打开但抓不到帧";
      }
      cap.release();
    }
  }

  // 名称缺失的索引：补成独立条目，避免用户看不到
  for (int i = 0; i < static_cast<int>(used.size()); ++i) {
    if (used[static_cast<size_t>(i)]) continue;
    if (isDeviceInUse(i)) continue;   // 正在使用中，不重复打开
    cv::VideoCapture cap;
    bool opened = false;
    try {
      opened = cap.open(i, cv::CAP_DSHOW);
      if (!opened) opened = cap.open(i);
    } catch (...) {
      opened = false;
    }
    if (!opened) continue;   // 完全打不开的空白索引就不列出来了

    PhysicalDevice dev;
    dev.serial = "IDX-" + std::to_string(i);
    dev.displayName = "(未命名设备)";
    DeviceChannel ch;
    ch.index = i;
    ch.friendlyName = dev.displayName;
    ch.openable = true;
    cv::Mat frame;
    for (int attempt = 0; attempt < 6 && frame.empty(); ++attempt) {
      try { cap.read(frame); } catch (...) { break; }
      if (frame.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    if (!frame.empty()) {
      ch.deliversFrames = true;
      ch.width = frame.cols;
      ch.height = frame.rows;
      ch.firstFrameMean = cv::mean(frame)[0];
      ch.kind = (ch.firstFrameMean < 3.0) ? "depth/ir" : "rgb";
    }
    cap.release();
    dev.channels.push_back(std::move(ch));
    devices.push_back(std::move(dev));
  }
}

const DeviceChannel* PhysicalDevice::preferredColorChannel() const {
  // 优先：彩色且能取帧 > 能取帧 > 任意彩色
  const DeviceChannel* best = nullptr;
  for (const auto& c : channels) {
    const bool isColor = (c.kind == "rgb");
    if (isColor && c.deliversFrames) return &c;
    if (isColor && !best) best = &c;
    if (c.deliversFrames && !best) best = &c;
  }
  if (best) return best;
  return channels.empty() ? nullptr : &channels.front();
}

const DeviceChannel* PhysicalDevice::findChannel(const std::string& kind) const {
  for (const auto& c : channels) {
    if (c.kind == kind) return &c;
  }
  return nullptr;
}

}  // namespace ui
