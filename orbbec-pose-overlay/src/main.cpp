// ============================================================================
//  src/main.cpp  —  程序入口 / 主循环 / 命令行参数
//
//  典型用法：
//    pose_overlay --source opencv --pose mock --camera 0
//    pose_overlay --source nuitrack --pose nuitrack
//    pose_overlay --source orbbec  --pose nuitrack --no-mirror
//    pose_overlay --source opencv --video demo.mp4 --headless --exit-after 300
//
//  运行时快捷键：
//    q / ESC  退出          s  保存当前帧 PNG
//    1 骨骼开关   2 关节角度标注   3 HUD 表格   4 深度小窗
//    5 3D/2D 角度切换（3D 不可用时自动退回 2D）
//    m 镜像开关   t 阈值告警开关
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "pose/angles.hpp"
#include "pose/color_source.hpp"
#include "pose/pose_provider.hpp"
#include "pose/renderer.hpp"
#include "pose/types.hpp"

namespace {

using namespace pose;

struct CliOptions {
  AppConfig cfg{};
  bool showHelp = false;
  bool noMirror = false;
  bool disableThreshold = false;
};

void printUsage() {
  std::printf(
      "pose_overlay —— 在彩色图像上叠加骨骼与关节角度（Astra+ / Nuitrack / Orbbec SDK v2）\n"
      "\n"
      "数据源（--source）：\n"
      "  mock        合成画面 + 合成骨骼（无相机，用于验证叠加与角度链路，默认）\n"
      "  opencv      USB 摄像头 / 视频文件 / 图片（--camera / --video）\n"
      "  nuitrack    Nuitrack 彩色帧（与 Nuitrack 骨骼天然对齐，需 -DHAVE_NUITRACK=ON）\n"
      "  orbbec      Orbbec SDK v2 彩色/深度（需 -DHAVE_ORBBEC_SDK=ON）\n"
      "\n"
      "骨骼后端（--pose）：\n"
      "  dnn              ONNX 姿态模型跑在彩色图上（需 --pose-model，无需 License）\n"
      "  orbbec-astra     奥比中光官方 Astra SDK Body Tracking（需 SDK + License）\n"
      "  nuitrack         Nuitrack SkeletonTracker（第三方，需 License）\n"
      "  mock             合成骨骼（默认；依赖缺失时自动降级到此）\n"
      "\n"
      "DNN 后端参数：\n"
      "  --pose-model PATH   ONNX 模型路径（YOLOv8-pose，输入 640x640）\n"
      "  --pose-score F      人体框置信度阈值（默认 0.35）\n"
      "  --pose-kpt F        关键点置信度阈值（默认 0.25）\n"
      "  --pose-size N       模型输入边长（默认 640）\n"
      "  --no-depth3d        不用深度图补 3D（只用 2D 算角度）\n"
      "\n"
      "常用参数：\n"
      "  --camera N              摄像头索引（默认 0）\n"
      "  --backend NAME          OpenCV 采集后端：auto/dshow/obsensor/msmf/any\n"
      "  --video PATH            视频文件路径\n"
      "  --loop / --no-loop      视频是否循环（默认循环）\n"
      "  --width N --height N    期望彩色分辨率（默认 1280x720）\n"
      "  --fps N                 期望帧率（默认 30）\n"
      "  --angles-2d             角度改用 2D 计算（默认优先 3D）\n"
      "  --min-conf F            关节置信度阈值（默认 0.30）\n"
      "  --no-skeleton           不画骨架连线\n"
      "  --no-joint-angles       不在关节旁标注角度数值\n"
      "  --no-hud                不画右上/左上 HUD 表格\n"
      "  --depth-peek            显示深度小窗\n"
      "  --thickness N           骨骼线宽（默认 4）\n"
      "  --font F                文字缩放（默认 0.55）\n"
      "  --chinese               尽量使用中文标签（需系统支持）\n"
      "  --threshold             启用阈值告警（超限角度标红）\n"
      "  --elbow-th F --knee-th F --torso-th F --neck-th F   告警阈值\n"
      "  --no-mirror             不做镜像显示（默认镜像，自拍视角更自然）\n"
      "  --save-video PATH       把带叠加的结果保存为 MJPEG AVI\n"
      "  --csv PATH              把每帧每人的角度写成 CSV（含 3D/2D 来源标记）\n"
      "  --snapshot-every N      每 N 帧落一张叠加结果 PNG（无窗口也能留证据）\n"
      "  --headless              不开窗口（配合 --save-video / --csv / --exit-after）\n"
      "  --exit-after N          处理 N 帧后退出（自动化验证用）\n"
      "  --list                  打印本次编译启用的能力，并探测可用摄像头\n"
      "  --no-probe              配合 --list 时不枚举摄像头（更快）\n"
      "  -h, --help              显示本帮助\n");
}

bool needValue(int i, int argc, const char* flag) {
  if (i + 1 >= argc) {
    std::fprintf(stderr, "参数 %s 缺少取值\n", flag);
    return false;
  }
  return true;
}

bool parseCli(int argc, char** argv, CliOptions& out) {
  AppConfig& c = out.cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&](std::string& dst) {
      if (!needValue(i, argc, a.c_str())) return false;
      dst = argv[++i];
      return true;
    };
    const auto nextInt = [&](int& dst) {
      if (!needValue(i, argc, a.c_str())) return false;
      dst = std::atoi(argv[++i]);
      return true;
    };
    const auto nextFloat = [&](float& dst) {
      if (!needValue(i, argc, a.c_str())) return false;
      dst = static_cast<float>(std::atof(argv[++i]));
      return true;
    };

    if (a == "-h" || a == "--help") {
      out.showHelp = true;
      return true;
    } else if (a == "--list") {
      return true;  // 在 main 中处理
    } else if (a == "--source") {
      std::string v;
      if (!next(v)) return false;
      if (!parseSourceKind(v, c.source)) {
        std::fprintf(stderr, "未知数据源: %s\n", v.c_str());
        return false;
      }
    } else if (a == "--pose") {
      std::string v;
      if (!next(v)) return false;
      if (!parsePoseBackend(v, c.poseBackend)) {
        std::fprintf(stderr, "未知骨骼后端: %s（可选 dnn / nuitrack / orbbec-astra / mock）\n",
                     v.c_str());
        return false;
      }
    } else if (a == "--pose-model") {
      if (!next(c.poseModelPath)) return false;
    } else if (a == "--pose-score") {
      if (!nextFloat(c.poseScoreThreshold)) return false;
    } else if (a == "--pose-kpt") {
      if (!nextFloat(c.poseKptThreshold)) return false;
    } else if (a == "--pose-size") {
      if (!nextInt(c.poseInputSize)) return false;
    } else if (a == "--no-depth3d") {
      c.useDepthFor3d = false;
    } else if (a == "--backend") {
      if (!next(c.captureBackend)) return false;
    } else if (a == "--camera") {
      if (!nextInt(c.cameraIndex)) return false;
    } else if (a == "--video") {
      if (!next(c.videoPath)) return false;
    } else if (a == "--loop") {
      c.loopVideo = true;
    } else if (a == "--no-loop") {
      c.loopVideo = false;
    } else if (a == "--width") {
      if (!nextInt(c.colorWidth)) return false;
    } else if (a == "--height") {
      if (!nextInt(c.colorHeight)) return false;
    } else if (a == "--fps") {
      if (!nextInt(c.fps)) return false;
    } else if (a == "--angles-2d") {
      c.anglesFrom3d = false;
    } else if (a == "--min-conf") {
      if (!nextFloat(c.minJointConfidence)) return false;
    } else if (a == "--no-skeleton") {
      c.drawSkeleton = false;
    } else if (a == "--no-joint-angles") {
      c.drawJointAngles = false;
    } else if (a == "--no-hud") {
      c.drawHudTable = false;
    } else if (a == "--depth-peek") {
      c.drawDepthPeek = true;
    } else if (a == "--thickness") {
      if (!nextInt(c.boneThickness)) return false;
    } else if (a == "--font") {
      if (!nextFloat(c.fontScale)) return false;
    } else if (a == "--chinese") {
      c.showChinese = true;
    } else if (a == "--threshold") {
      c.enableThreshold = true;
    } else if (a == "--elbow-th") {
      if (!nextFloat(c.elbowStraightenThreshold)) return false;
    } else if (a == "--knee-th") {
      if (!nextFloat(c.kneeStraightenThreshold)) return false;
    } else if (a == "--torso-th") {
      if (!nextFloat(c.torsoLeanWarnDeg)) return false;
    } else if (a == "--neck-th") {
      if (!nextFloat(c.neckTiltWarnDeg)) return false;
    } else if (a == "--no-mirror") {
      out.noMirror = true;
    } else if (a == "--save-video") {
      std::string v;
      if (!next(v)) return false;
      c.saveAnnotatedVideo = true;
      c.outputVideoPath = v;
    } else if (a == "--headless") {
      c.headless = true;
    } else if (a == "--csv") {
      std::string v;
      if (!next(v)) return false;
      c.saveCsv = true;
      c.csvPath = v;
    } else if (a == "--snapshot-every") {
      if (!nextInt(c.snapshotEveryNFrames)) return false;
    } else if (a == "--debug") {
      c.debugPrint = true;
    } else if (a == "--exit-after") {
      if (!nextInt(c.exitAfterFrames)) return false;
    } else {
      std::fprintf(stderr, "未知参数: %s（用 --help 查看用法）\n", a.c_str());
      return false;
    }
  }
  if (out.noMirror) c.mirror = false;
  if (out.disableThreshold) c.enableThreshold = false;
  return true;
}

void printCapabilities() {
  std::printf("编译期能力：\n");
  std::printf("  ONNX 姿态骨骼后端(dnn)   : %s\n", dnnBackendAvailable() ? "已启用" : "未启用");
  std::printf("  Nuitrack 骨骼后端        : %s\n", nuitrackCompiledIn() ? "已启用" : "未启用");
  std::printf("  奥比中光 Body Tracking   : %s\n", astraCompiledIn() ? "已启用" : "未启用");
  std::printf("  Orbbec SDK v2 取流       : %s\n", orbbecCompiledIn() ? "已启用" : "未启用");
  std::printf("  可用数据源               : mock, opencv%s%s\n",
              nuitrackCompiledIn() ? ", nuitrack" : "",
              orbbecCompiledIn() ? ", orbbec" : "");
  std::printf("  可用骨骼后端             : mock, dnn%s%s\n",
              nuitrackCompiledIn() ? ", nuitrack" : "",
              astraCompiledIn() ? ", orbbec-astra" : "");
}

// 崩溃定位：把访问冲突等硬件异常翻译成可读信息（Windows / MSVC）

// 诊断用的阶段性打印：崩溃定位用，正常运行时静默
static bool g_trace = false;
#define TRACE(...)                       \
  do {                                   \
    if (g_trace) {                       \
      std::printf(__VA_ARGS__);          \
      std::fflush(stdout);               \
    }                                    \
  } while (0)

// 探测可用的彩色输入设备：枚举索引 + 抓一帧存图，便于确认哪个索引是哪台相机
void probeVideoDevices(int maxIndex = 4, bool tryDevices = true) {
  std::printf("\n== 视频输入设备探测 ==\n");

  if (tryDevices) {
    bool any = false;
    for (int i = 0; i < maxIndex; ++i) {
      cv::VideoCapture cap;
      bool opened = false;
      const char* via = "";
      try {
        opened = cap.open(i, cv::CAP_OBSENSOR);  // 奥比中光后端优先
        via = "OBSENSOR";
      } catch (...) {
        opened = false;
      }
      if (!opened) {
        try {
          opened = cap.open(i, cv::CAP_DSHOW);
          via = "DSHOW";
        } catch (...) {
          opened = false;
        }
      }
      if (!opened) {
        try {
          opened = cap.open(i);
          via = "默认后端";
        } catch (...) {
          opened = false;
        }
      }
      if (!opened) {
        std::printf("  index %d : 打不开\n", i);
        cap.release();
        continue;
      }
      any = true;
      const int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
      const int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
      // 抓一帧确认真的出图（有些设备能 open 但抓不到帧）
      cv::Mat frame;
      bool got = false;
      for (int attempt = 0; attempt < 5 && !got; ++attempt) {
        got = cap.read(frame) && !frame.empty();
        if (!got) cv::waitKey(30);
      }
      if (got) {
        char name[128];
        std::snprintf(name, sizeof(name), "probe_index_%d.png", i);
        const bool saved = cv::imwrite(name, frame);
        char savedInfo[192];
        if (saved) {
          std::snprintf(savedInfo, sizeof(savedInfo), "已存 %s", name);
        } else {
          std::snprintf(savedInfo, sizeof(savedInfo), "存图失败");
        }
        std::printf("  index %d : 可取帧  %dx%d (%s)  首帧均值=%.1f  %s\n", i, frame.cols,
                    frame.rows, via, cv::mean(frame)[0], savedInfo);
      } else {
        std::printf("  index %d : 能打开但抓不到帧  %dx%d (%s)\n", i, w, h, via);
      }
      cap.release();
    }
    if (!any) {
      std::printf("  → 没有可直接打开的摄像头。若相机是奥比中光 Astra+，通常需要：\n"
                  "     a) 安装奥比中光官方 USB 驱动（设备管理器里不能是未知设备）；\n"
                  "     b) 用 --source orbbec（Orbbec SDK）或 --source nuitrack 打开。\n");
    }
  }

  std::printf("  用法：--source opencv --pose mock --camera <可用索引>\n\n");
}

// 用深度图把 2D 关节补成 3D（角度改用 3D 计算，比纯 2D 更准）。
// 说明：这里只用到深度值 z，x/y 仍按彩色图像素坐标保留（角度只依赖关节间向量，
// 对同一人而言尺度一致，不影响夹角；如需真实 XYZ 需要相机内参，属于后续增强）。
void backfillDepth3d(PoseFrame& poses, const DepthFrame& depth, int colorW, int colorH,
                     float kptThreshold) {
  if (depth.empty() || colorW <= 0 || colorH <= 0) return;
  const float sx = static_cast<float>(depth.width) / colorW;
  const float sy = static_cast<float>(depth.height) / colorH;

  for (Skeleton& sk : poses.skeletons) {
    for (int i = 0; i < kJointCount; ++i) {
      if (!sk.joints2d[i].valid || sk.joints3d[i].valid) continue;
      if (sk.conf[i] > 0.f && sk.conf[i] < kptThreshold) continue;

      const int cx = static_cast<int>(sk.joints2d[i].x * sx);
      const int cy = static_cast<int>(sk.joints2d[i].y * sy);
      if (cx < 0 || cy < 0 || cx >= depth.width || cy >= depth.height) continue;

      // 7x7 邻域取有效深度的均值，抑制噪声
      int sum = 0, cnt = 0;
      for (int dy = -3; dy <= 3; ++dy) {
        const int yy = cy + dy;
        if (yy < 0 || yy >= depth.height) continue;
        for (int dx = -3; dx <= 3; ++dx) {
          const int xx = cx + dx;
          if (xx < 0 || xx >= depth.width) continue;
          const uint16_t d = depth.mm[static_cast<size_t>(yy) * depth.width + xx];
          if (d == 0 || d > 10000) continue;  // 0 = 无效，>10m 视为噪声
          sum += d;
          ++cnt;
        }
      }
      if (cnt < 5) continue;
      const float zmm = static_cast<float>(sum) / cnt;
      sk.joints3d[i] = Vec3{sk.joints2d[i].x, sk.joints2d[i].y, zmm, true};
    }
  }
}

// 用数据源内参把 3D 关节投影到彩色图（当骨骼后端没给 2D 时的兜底）
void fillMissing2d(PoseFrame& poses, const ColorSource::Intrinsics& intr,
                   const PoseProvider& provider, bool useProviderProjection) {
  for (Skeleton& sk : poses.skeletons) {
    for (int i = 0; i < kJointCount; ++i) {
      if (sk.joints2d[i].valid) continue;
      if (!sk.joints3d[i].valid) continue;
      Vec2 p;
      bool ok = false;
      if (useProviderProjection && provider.projectJoint(sk.joints3d[i], p)) {
        ok = true;
      } else if (intr.valid && std::fabs(sk.joints3d[i].z) > 1e-3f) {
        p.x = intr.cx + sk.joints3d[i].x * intr.fx / sk.joints3d[i].z;
        p.y = intr.cy + sk.joints3d[i].y * intr.fy / sk.joints3d[i].z;
        p.valid = true;
        ok = true;
      }
      if (ok) sk.joints2d[i] = p;
    }
  }
}

}  // namespace

// ---------------------------------------------------------------------------
//  崩溃定位处理（必须在全局作用域：匿名命名空间内会让 _EXCEPTION_POINTERS
//  变成另一个类型，导致 _set_se_translator 编译失败）
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
#include <eh.h>
#include <windows.h>

// 注意：这里刻意不使用匿名命名空间。
// 一旦放进匿名命名空间，`struct _EXCEPTION_POINTERS*` 会被解析成该命名空间里
// 新声明的类型，与 _se_translator_function 期望的类型不一致（编译报 C2664）。
static const char* poseSehName(unsigned int code) {
  switch (code) {
    case EXCEPTION_ACCESS_VIOLATION: return "ACCESS_VIOLATION（读写非法地址）";
    case EXCEPTION_INT_DIVIDE_BY_ZERO: return "INT_DIVIDE_BY_ZERO（整数除零）";
    case EXCEPTION_STACK_OVERFLOW: return "STACK_OVERFLOW（栈溢出）";
    case EXCEPTION_ILLEGAL_INSTRUCTION: return "ILLEGAL_INSTRUCTION";
    case EXCEPTION_IN_PAGE_ERROR: return "IN_PAGE_ERROR";
    case EXCEPTION_FLT_DIVIDE_BY_ZERO: return "FLT_DIVIDE_BY_ZERO";
    default: return "未知异常";
  }
}

// 用全局作用域限定 ::，避免 _EXCEPTION_POINTERS 被解析成当前命名空间里的新类型
static void __cdecl poseSehTranslator(unsigned int code, struct ::_EXCEPTION_POINTERS* info) {
  std::fprintf(stderr, "\n[FATAL] 捕获到硬件异常: 0x%08X  %s\n", code, poseSehName(code));
  if (info && code == EXCEPTION_ACCESS_VIOLATION && info->ExceptionRecord &&
      info->ExceptionRecord->NumberParameters >= 2) {
    const ULONG_PTR rw = info->ExceptionRecord->ExceptionInformation[0];
    const ULONG_PTR addr = info->ExceptionRecord->ExceptionInformation[1];
    std::fprintf(stderr, "        操作: %s  地址: 0x%llX\n",
                 rw == 0 ? "读取" : (rw == 1 ? "写入" : "执行"),
                 static_cast<unsigned long long>(addr));
  }
  std::fflush(stderr);
  _set_se_translator(nullptr);
  std::abort();
}
#define POSE_INSTALL_CRASH_HANDLER() _set_se_translator(poseSehTranslator)
#else
#define POSE_INSTALL_CRASH_HANDLER() ((void)0)
#endif

int main(int argc, char** argv) {
  POSE_INSTALL_CRASH_HANDLER();
  CliOptions cli;
  if (!parseCli(argc, argv, cli)) {
    printUsage();
    return 2;
  }
  if (cli.showHelp) {
    printUsage();
    return 0;
  }
  AppConfig& cfg = cli.cfg;

  // --list：打印能力清单（含摄像头探测）
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--list") == 0) {
      // 清掉 OpenCV 后端的告警噪声，只保留我们自己的输出
      cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
      printCapabilities();
      bool skip = false;
      for (int j = 1; j < argc; ++j) {
        if (std::strcmp(argv[j], "--no-probe") == 0) skip = true;
      }
      if (!skip) probeVideoDevices();
      return 0;
    }
  }

  std::printf("=== pose_overlay ===\n");
  printCapabilities();
  std::printf("数据源: %s   骨骼后端: %s\n", sourceKindName(cfg.source),
              poseBackendName(cfg.poseBackend));
  g_trace = cfg.debugPrint;
  std::fflush(stdout);
  TRACE("[trace] 参数解析完成\n");

  // ---- 1) 先建骨骼后端（Nuitrack/Astra 会独占相机，彩色源需要复用它）------
  TRACE("[trace] 创建骨骼后端...\n");
  auto provider = makePoseProvider(cfg);
  if (!provider) {
    std::fprintf(stderr, "创建骨骼后端失败。\n");
    return 3;
  }
  TRACE("[trace] 骨骼后端已创建: %s\n", provider->name());
  if (!provider->start(cfg)) {
    std::fprintf(stderr, "骨骼后端启动失败（%s）：\n%s\n", provider->name(),
                 provider->error().c_str());
    return 3;
  }
  TRACE("[trace] 骨骼后端已启动\n");
  {
    const std::string e = provider->error();
    if (!e.empty()) std::fprintf(stderr, "[pose] 提示：%s\n", e.c_str());
  }

  // ---- 2) 再建彩色数据源 ------------------------------------------------
  TRACE("[trace] 创建数据源...\n");
  auto source = makeColorSource(cfg, provider.get());
  if (!source) {
    std::fprintf(stderr, "创建数据源失败。\n");
    provider->stop();
    return 4;
  }
  TRACE("[trace] 数据源已创建: %s\n", source->name());
  if (!source->open(cfg)) {
    std::fprintf(stderr, "数据源打开失败（%s）：\n%s\n", source->name(),
                 source->error().c_str());
    provider->stop();
    return 4;
  }
  TRACE("[trace] 数据源已打开\n");

  const ColorSource::Intrinsics intr = source->colorIntrinsics();
  if (!intr.valid) {
    std::fprintf(stderr,
                 "[source] 提示：未能获取相机内参，3D 关节无法手工投影到彩色图；"
                 "将优先使用骨骼后端自带的 2D 坐标。\n");
  }
  if (!provider->projectionReady() && cfg.anglesFrom3d == false) {
    std::fprintf(stderr, "[pose] 提示：当前为 2D 角度模式（透视会影响读数，仅作参考）。\n");
  }

  OverlayRenderer renderer(cfg);
  AngleSmoother smoother;
  smoother.configure(0.6f);

  cv::Mat canvas;
  ColorFrame color;
  DepthFrame depth;
  PoseFrame poses;
  AngleList angles;

  const auto t0 = std::chrono::steady_clock::now();
  uint64_t frames = 0;
  double fps = 0.0;
  auto fpsWindowStart = t0;
  uint64_t fpsWindowFrames = 0;

  const std::string winName = "pose_overlay - skeleton & joint angles";
  if (!cfg.headless) {
    cv::namedWindow(winName, cv::WINDOW_NORMAL);
    cv::resizeWindow(winName, std::min(cfg.colorWidth, 1280), std::min(cfg.colorHeight, 720));
  }

  std::FILE* csv = nullptr;  // 在循环外持有，退出时统一关闭
  bool running = true;
  std::string statusLine;
  std::printf("[main] 进入主循环\n");
  std::fflush(stdout);
  while (running) {
    if (!source->grab(color, depth)) {
      const std::string e = source->error();
      std::fprintf(stderr, "取流结束或失败：%s\n", e.empty() ? "(流已结束)" : e.c_str());
      break;
    }
    if (cfg.debugPrint) {
      std::printf("[main] grab 完成 %dx%d\n", color.width, color.height);
      std::fflush(stdout);
    }

    // 彩色帧可能为空（例如 Nuitrack/Astra 首帧尚未到达）：
    // 此时直接跳过本帧，不要画 HUD，否则会在空画面上刷出"等待彩色帧"的告警。
    if (color.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      if (cfg.exitAfterFrames > 0 && frames > static_cast<uint64_t>(cfg.exitAfterFrames)) break;
      continue;
    }

    if (!dnnRunOnFrame(provider.get(), color, poses)) {
      // 非 DNN 后端（mock / nuitrack / astra）：走常规 fetch
      if (!provider->fetch(color.timestampUs, poses)) {
        const std::string e = provider->error();
        statusLine = e.empty() ? "等待骨骼数据..." : e.substr(0, 60);
      } else {
        statusLine.clear();
      }
    } else {
      statusLine.clear();
      TRACE("[trace] DNN 推理完成，人数=%d\n", static_cast<int>(poses.skeletons.size()));
    }

    // 深度补 3D：让角度用上相机坐标系的 z，减少透视误差
    if (cfg.useDepthFor3d) {
      backfillDepth3d(poses, depth, color.width, color.height, cfg.poseKptThreshold);
    }

    // 兜底投影：骨骼后端没给 2D 时用内参补
    fillMissing2d(poses, intr, *provider, true);

    // 角度计算 + 平滑 ---------------------------------------------
    if (cfg.debugPrint) {
      std::printf("[main] 开始算角度，skeletons=%d\n",
                  static_cast<int>(poses.skeletons.size()));
      std::fflush(stdout);
    }
    angles.clear();
    angles.reserve(poses.skeletons.size());
    for (const Skeleton& sk : poses.skeletons) {
      PoseAngles a = computeAngles(sk, cfg);
      smoother.apply(sk.id, a, color.timestampUs);
      angles.push_back(a);
    }

    // ---- 组图 ---------------------------------------------------------
    // 镜像显示的正确顺序（顺序很关键）：
    //   1) 只翻转画面；
    //   2) 把关节 2D 坐标按同一变换镜像，让骨骼与翻转后的画面对齐；
    //   3) 骨骼与关节角度标注画在翻转后的画面上；
    //   4) HUD 文字最后单独画 —— 它不能跟着翻，否则文字会左右反。
    // 角度是在镜像之前用 3D/原始 2D 坐标算好的，因此不受镜像影响。
    canvas = cv::Mat(color.height, color.width, CV_8UC3,
                     const_cast<uint8_t*>(color.bgr.data()));
    if (cfg.mirror) {
      cv::flip(canvas, canvas, 1);
      for (Skeleton& sk : poses.skeletons) {
        for (int i = 0; i < kJointCount; ++i) {
          if (sk.joints2d[i].valid) {
            sk.joints2d[i].x = static_cast<float>(color.width - 1) - sk.joints2d[i].x;
          }
        }
      }
    }
    TRACE("[trace] 组图完成 %dx%d mirror=%d\n", canvas.cols, canvas.rows, cfg.mirror ? 1 : 0);

    ++frames;
    ++fpsWindowFrames;
    const auto now = std::chrono::steady_clock::now();
    const double winSec =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - fpsWindowStart).count();
    if (winSec >= 0.5) {
      fps = fpsWindowFrames / winSec;
      fpsWindowStart = now;
      fpsWindowFrames = 0;
    }

    renderer.draw(canvas, poses, angles, cfg.drawDepthPeek ? &depth : nullptr, fps,
                  source->name(), provider->name(), statusLine);
    if (cfg.debugPrint) {
      for (const Skeleton& sk : poses.skeletons) {
        std::printf("[draw] id=%d mirror=%d ", sk.id, cfg.mirror ? 1 : 0);
        for (int i = 0; i < kJointCount; ++i) {
          if (sk.joints2d[i].valid) {
            std::printf("%s=(%.0f,%.0f) ", jointName(static_cast<JointId>(i)),
                        sk.joints2d[i].x, sk.joints2d[i].y);
          }
        }
        std::printf("\n");
      }
      std::fflush(stdout);
    }
    TRACE("[trace] 渲染完成 frame=%llu\n", static_cast<unsigned long long>(frames));

    // ---- 诊断输出：关节在彩色图上的实际像素包围盒 ------------------------
    if (cfg.debugPrint) {
      for (const Skeleton& sk : poses.skeletons) {
        float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
        int validCount = 0;
        for (int i = 0; i < kJointCount; ++i) {
          if (!sk.joints2d[i].valid) continue;
          ++validCount;
          minX = std::min(minX, sk.joints2d[i].x);
          minY = std::min(minY, sk.joints2d[i].y);
          maxX = std::max(maxX, sk.joints2d[i].x);
          maxY = std::max(maxY, sk.joints2d[i].y);
        }
        std::printf(
            "[debug] frame=%llu id=%d joints=%d bbox x:[%.0f..%.0f] y:[%.0f..%.0f] "
            "canvas=%dx%d mirror=%d\n",
            static_cast<unsigned long long>(frames), sk.id, validCount, minX, maxX, minY,
            maxY, canvas.cols, canvas.rows, cfg.mirror ? 1 : 0);
        if (const Skeleton* s = &sk; true) {
          std::printf("[debug]   head3d=(%.0f,%.0f,%.0f) head2d=(%.1f,%.1f) "
                      "foot3d=(%.0f,%.0f,%.0f) foot2d=(%.1f,%.1f)\n",
                      s->joints3d[0].x, s->joints3d[0].y, s->joints3d[0].z,
                      s->joints2d[0].x, s->joints2d[0].y,
                      s->joints3d[static_cast<int>(JointId::AnkleL)].x,
                      s->joints3d[static_cast<int>(JointId::AnkleL)].y,
                      s->joints3d[static_cast<int>(JointId::AnkleL)].z,
                      s->joints2d[static_cast<int>(JointId::AnkleL)].x,
                      s->joints2d[static_cast<int>(JointId::AnkleL)].y);
        }
      }
    }

    if (cfg.saveAnnotatedVideo) {
      static cv::VideoWriter writer;
      static bool opened = false;
      if (!opened) {
        writer.open(cfg.outputVideoPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    cfg.fps > 0 ? cfg.fps : 25, cv::Size(canvas.cols, canvas.rows));
        opened = writer.isOpened();
        if (!opened) {
          std::fprintf(stderr, "无法创建输出视频: %s\n", cfg.outputVideoPath.c_str());
          cfg.saveAnnotatedVideo = false;
        } else {
          std::printf("正在录制叠加结果 -> %s\n", cfg.outputVideoPath.c_str());
        }
      }
      if (opened) writer.write(canvas);
    }

    // ---- CSV 落盘：每帧每人的角度，便于事后分析/回归 ----------------------
    if (cfg.saveCsv) {
      if (!csv) {
        csv = std::fopen(cfg.csvPath.c_str(), "w");
        if (!csv) {
          std::fprintf(stderr, "无法创建 CSV: %s\n", cfg.csvPath.c_str());
          cfg.saveCsv = false;
        } else {
          std::fprintf(csv, "frame,timestamp_us,skeleton_id,global_conf");
          for (int i = 0; i < kAngleCount; ++i) {
            std::fprintf(csv, ",%s_deg", angleName(static_cast<AngleId>(i)));
          }
          for (int i = 0; i < kAngleCount; ++i) {
            std::fprintf(csv, ",%s_from3d", angleName(static_cast<AngleId>(i)));
          }
          std::fprintf(csv, "\n");
          std::printf("正在写角度 CSV -> %s\n", cfg.csvPath.c_str());
        }
      }
      if (csv) {
        for (size_t pi = 0; pi < poses.skeletons.size(); ++pi) {
          const Skeleton& sk = poses.skeletons[pi];
          std::fprintf(csv, "%llu,%llu,%d,%.3f",
                       static_cast<unsigned long long>(frames),
                       static_cast<unsigned long long>(color.timestampUs), sk.id,
                       static_cast<double>(sk.globalConfidence));
          if (pi < angles.size()) {
            for (int i = 0; i < kAngleCount; ++i) {
              const AngleValue& v = angles[pi].values[i];
              std::fprintf(csv, ",%.2f", v.valid ? static_cast<double>(v.deg) : -1.0);
            }
            for (int i = 0; i < kAngleCount; ++i) {
              const AngleValue& v = angles[pi].values[i];
              std::fprintf(csv, ",%d", v.valid && v.from3d ? 1 : 0);
            }
          } else {
            for (int i = 0; i < kAngleCount * 2; ++i) std::fprintf(csv, ",");
          }
          std::fprintf(csv, "\n");
        }
        std::fflush(csv);  // 及时落盘，便于边跑边看
      }
    }

    // ---- 周期性快照（无窗口也能留证据）-----------------------------------
    if (cfg.snapshotEveryNFrames > 0 &&
        frames % static_cast<uint64_t>(cfg.snapshotEveryNFrames) == 0) {
      char name[256];
      std::snprintf(name, sizeof(name), "%s_%06llu.png", cfg.snapshotPrefix.c_str(),
                    static_cast<unsigned long long>(frames));
      if (cv::imwrite(name, canvas)) {
        std::printf("已保存快照 %s\n", name);
      } else {
        std::fprintf(stderr, "快照保存失败: %s\n", name);
      }
    }

    if (!cfg.headless) {
      cv::imshow(winName, canvas);
      const int key = cv::waitKey(1) & 0xFF;
      switch (key) {
        case 'q':
        case 27:
          running = false;
          break;
        case '1':
          cfg.drawSkeleton = !cfg.drawSkeleton;
          break;
        case '2':
          cfg.drawJointAngles = !cfg.drawJointAngles;
          break;
        case '3':
          cfg.drawHudTable = !cfg.drawHudTable;
          break;
        case '4':
          cfg.drawDepthPeek = !cfg.drawDepthPeek;
          break;
        case '5':
          cfg.anglesFrom3d = !cfg.anglesFrom3d;
          std::printf("角度模式: %s\n", cfg.anglesFrom3d ? "3D 优先" : "2D");
          break;
        case 'm':
          cfg.mirror = !cfg.mirror;
          break;
        case 't':
          cfg.enableThreshold = !cfg.enableThreshold;
          break;
        case 's': {
          char name[64];
          std::snprintf(name, sizeof(name), "snapshot_%06llu.png",
                        static_cast<unsigned long long>(frames));
          cv::imwrite(name, canvas);
          std::printf("已保存 %s\n", name);
          break;
        }
        default:
          break;
      }
    }

    if (cfg.exitAfterFrames > 0 && frames >= static_cast<uint64_t>(cfg.exitAfterFrames)) {
      std::printf("已达到 --exit-after %d 帧，正常退出。\n", cfg.exitAfterFrames);
      break;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  const double totalSec =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
  std::printf("处理 %llu 帧，用时 %.2f 秒，平均 %.1f FPS\n",
              static_cast<unsigned long long>(frames), totalSec,
              totalSec > 0 ? frames / totalSec : 0.0);

  source->close();
  provider->stop();
  if (csv) {
    std::fclose(csv);
    csv = nullptr;
    std::printf("角度 CSV 已写入 -> %s\n", cfg.csvPath.c_str());
  }
  if (!cfg.headless) cv::destroyAllWindows();
  return 0;
}
