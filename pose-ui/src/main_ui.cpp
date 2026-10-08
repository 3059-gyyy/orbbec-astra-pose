// ============================================================================
//  src/main_ui.cpp  —  图形界面主程序
//
//  UI 结构（全部用色块与自绘控件，不依赖任何贴图素材）：
//    ┌───────────┬──────────────────────────────┬───────────┐
//    │ 相机面板   │        主视图（视口）          │  显示设置  │
//    │ 开关/设备  │  彩色图 / 深度图 / 点云 / 骨骼  │  叠加开关  │
//    │ 彩色/深度  │                              │  阈值参数  │
//    ├───────────┴──────────────────────────────┴───────────┤
//    │ 状态栏：FPS / 分辨率 / 推理耗时 / 检测人数 / 快捷键提示  │
//    └───────────────────────────────────────────────────────┘
//
//  渲染要点：
//   * 彩色/深度/骨骼：CPU 绘制 -> OpenGL 纹理 -> ImGui::Image
//   * 点云：OpenGL FBO 离屏渲染 -> 纹理 -> ImGui::Image
//   * 鼠标左键拖拽旋转点云、滚轮缩放（仅在点云视图生效）
// ============================================================================
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Windows 平台下 GL/gl.h 依赖 windows.h 里的 WINGDIAPI / APIENTRY 定义，
// 必须在 GL 头文件之前包含。
#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "app_config.hpp"
#include "camera_pipeline.hpp"
#include "capture.hpp"
#include "gl_loader.hpp"
#include "video_device.hpp"
#include "pointcloud_gl.hpp"
#include "render_common.hpp"

namespace {

// 版本号：每次修改后递增，便于确认运行的是哪一版（窗口标题与启动日志都会显示）
constexpr const char* kAppVersion = "v2.0 (accurate recording duration)";
// GL 函数指针/常量在 gl:: 命名空间（见 gl_loader.hpp）
using namespace gl;

// ---------------------------------------------------------------------------
//  字体：加载系统中文字体（否则中文会显示成方框）
//  优先级：微软雅黑 -> 黑体 -> 宋体 -> 等线；都找不到则退回 ImGui 默认字体
// ---------------------------------------------------------------------------
void loadFonts() {
  ImGuiIO& io = ImGui::GetIO();
  const char* candidates[] = {
      "C:/Windows/Fonts/msyh.ttc",    // 微软雅黑
      "C:/Windows/Fonts/msyhl.ttc",   // 微软雅黑 Light
      "C:/Windows/Fonts/simhei.ttf",  // 黑体
      "C:/Windows/Fonts/simsun.ttc",  // 宋体
      "C:/Windows/Fonts/Deng.ttf",    // 等线
  };
  ImFontConfig cfg;
  cfg.OversampleH = 2;
  cfg.OversampleV = 2;
  cfg.PixelSnapH = false;
  for (const char* path : candidates) {
    if (io.Fonts->AddFontFromFileTTF(path, 17.0f, &cfg,
                                     io.Fonts->GetGlyphRangesChineseSimplifiedCommon()) != nullptr) {
      std::printf("[ui] loaded CJK font: %s\n", path);
      return;
    }
  }
  io.Fonts->AddFontDefault();
  std::fprintf(stderr, "[ui] CJK font not found; labels may show as boxes\n");
}

// ---------------------------------------------------------------------------
//  现代化深色主题（纯色块，无贴图）
// ---------------------------------------------------------------------------
void applyModernTheme() {
  ImGuiStyle& s = ImGui::GetStyle();
  ImGui::StyleColorsDark();

  // 圆角与间距：偏大圆角 + 舒展间距 = 现代感
  s.WindowRounding = 10.f;
  s.ChildRounding = 8.f;
  s.FrameRounding = 7.f;
  s.PopupRounding = 8.f;
  s.ScrollbarRounding = 8.f;
  s.GrabRounding = 6.f;
  s.TabRounding = 8.f;

  s.WindowBorderSize = 0.f;
  s.ChildBorderSize = 1.f;
  s.FrameBorderSize = 0.f;
  s.PopupBorderSize = 1.f;

  s.WindowPadding = ImVec2(14, 12);
  s.FramePadding = ImVec2(10, 6);
  s.ItemSpacing = ImVec2(9, 8);
  s.ItemInnerSpacing = ImVec2(7, 6);
  s.IndentSpacing = 18.f;
  s.ScrollbarSize = 12.f;
  s.GrabMinSize = 10.f;

  ImVec4* c = s.Colors;
  const ImVec4 bg0(0.075f, 0.082f, 0.10f, 1.00f);   // 窗口底色
  const ImVec4 bg1(0.105f, 0.115f, 0.14f, 1.00f);   // 控件底色
  const ImVec4 bg2(0.145f, 0.155f, 0.185f, 1.00f);  // 悬停
  const ImVec4 bg3(0.185f, 0.20f, 0.235f, 1.00f);   // 按下
  const ImVec4 accent(0.22f, 0.62f, 0.98f, 1.00f);  // 主色（蓝）
  const ImVec4 accentDim(0.18f, 0.44f, 0.72f, 1.00f);

  c[ImGuiCol_Text] = ImVec4(0.92f, 0.93f, 0.96f, 1.00f);
  c[ImGuiCol_TextDisabled] = ImVec4(0.48f, 0.51f, 0.56f, 1.00f);
  c[ImGuiCol_WindowBg] = bg0;
  c[ImGuiCol_ChildBg] = ImVec4(0.095f, 0.105f, 0.128f, 1.00f);
  c[ImGuiCol_PopupBg] = ImVec4(0.11f, 0.12f, 0.145f, 0.98f);
  c[ImGuiCol_Border] = ImVec4(0.24f, 0.26f, 0.30f, 0.65f);
  c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_FrameBg] = bg1;
  c[ImGuiCol_FrameBgHovered] = bg2;
  c[ImGuiCol_FrameBgActive] = bg3;
  c[ImGuiCol_TitleBg] = ImVec4(0.085f, 0.092f, 0.112f, 1.00f);
  c[ImGuiCol_TitleBgActive] = ImVec4(0.105f, 0.115f, 0.14f, 1.00f);
  c[ImGuiCol_TitleBgCollapsed] = bg0;
  c[ImGuiCol_MenuBarBg] = ImVec4(0.095f, 0.105f, 0.128f, 1.00f);
  c[ImGuiCol_ScrollbarBg] = ImVec4(0.075f, 0.082f, 0.10f, 1.00f);
  c[ImGuiCol_ScrollbarGrab] = ImVec4(0.24f, 0.26f, 0.31f, 1.00f);
  c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.31f, 0.34f, 0.40f, 1.00f);
  c[ImGuiCol_ScrollbarGrabActive] = accentDim;
  c[ImGuiCol_CheckMark] = accent;
  c[ImGuiCol_SliderGrab] = accent;
  c[ImGuiCol_SliderGrabActive] = ImVec4(0.35f, 0.72f, 1.00f, 1.00f);
  c[ImGuiCol_Button] = bg1;
  c[ImGuiCol_ButtonHovered] = bg2;
  c[ImGuiCol_ButtonActive] = accentDim;
  c[ImGuiCol_Header] = ImVec4(0.16f, 0.22f, 0.31f, 1.00f);
  c[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.29f, 0.40f, 1.00f);
  c[ImGuiCol_HeaderActive] = accentDim;
  c[ImGuiCol_Separator] = ImVec4(0.22f, 0.24f, 0.28f, 0.85f);
  c[ImGuiCol_SeparatorHovered] = accentDim;
  c[ImGuiCol_SeparatorActive] = accent;
  c[ImGuiCol_ResizeGrip] = ImVec4(0.24f, 0.26f, 0.31f, 0.70f);
  c[ImGuiCol_Tab] = ImVec4(0.11f, 0.12f, 0.145f, 1.00f);
  c[ImGuiCol_TabHovered] = ImVec4(0.20f, 0.29f, 0.40f, 1.00f);
  c[ImGuiCol_TabSelected] = ImVec4(0.16f, 0.24f, 0.34f, 1.00f);
  c[ImGuiCol_PlotLines] = accent;
  c[ImGuiCol_PlotHistogram] = accent;
  c[ImGuiCol_TextSelectedBg] = ImVec4(0.22f, 0.62f, 0.98f, 0.35f);
}

// ---------------------------------------------------------------------------
//  OpenGL 纹理包装（CPU 图 -> 纹理）
// ---------------------------------------------------------------------------
class GlTexture {
 public:
  ~GlTexture() { destroy(); }

  void uploadBgr(const cv::Mat& bgr) {
    if (bgr.empty()) return;
    if (w_ != bgr.cols || h_ != bgr.rows || id_ == 0) {
      destroy();
      glGenTextures(1, &id_);
      glBindTexture(GL_TEXTURE_2D, id_);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      w_ = bgr.cols;
      h_ = bgr.rows;
    } else {
      glBindTexture(GL_TEXTURE_2D, id_);
    }
    // OpenCV 是 BGR，OpenGL 期望 RGB
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, bgr.cols, bgr.rows, 0, GL_BGR, GL_UNSIGNED_BYTE,
                 bgr.data);
  }

  void destroy() {
    if (id_) {
      glDeleteTextures(1, &id_);
      id_ = 0;
    }
    w_ = h_ = 0;
  }

  unsigned int id() const { return id_; }
  int width() const { return w_; }
  int height() const { return h_; }

 private:
  unsigned int id_ = 0;
  int w_ = 0;
  int h_ = 0;
};

// ---------------------------------------------------------------------------
//  应用状态
// ---------------------------------------------------------------------------
struct AppState {
  ui::CameraPipeline pipeline;
  ui::PipelineOptions opt;

  ui::ViewMode view = ui::ViewMode::Color;
  ui::DrawStyle style;

  ui::PointCloudView cloudView;
  ui::PointCloudRenderer cloud;

  // 骨骼抖动平滑（显示用，不影响角度计算）
  bool smoothSkeleton = true;
  float smoothAlpha = 0.55f;

  std::vector<pose::GroundCalib> calib;   // 当前帧的人体坐标系标定
  int cameraCount = 4;
  bool showAbout = false;
  bool statsWindow = true;

  // 点云数据（避免每帧重建：仅在需要时更新）
  std::vector<float> cloudPos;
  std::vector<uint8_t> cloudCol;
  bool cloudDirty = true;
  int cloudPoints = 0;
  double cloudBuildMs = 0.0;

  // 显示用缓存
  cv::Mat canvas;          // 当前要显示的画面（彩色叠加 / 深度 / 骨骼）
  ui::Sample sample;       // 最新一帧副本
  uint64_t lastFrameStamp = 0;

  // ---- 摄像头设备列表（按唯一 SN 聚合，一台设备一行）----
  std::vector<ui::PhysicalDevice> devices;
  std::mutex deviceMtx;
  std::atomic<bool> scanning{false};
  std::thread scanThread;
  int selectedDeviceIdx = 0;   // devices 数组下标（选中的物理设备）
  int deviceScanAttempts = 0;  // 自动重扫次数（插上设备后自动发现）

  // 演示骨骼（合成火柴人，仅用于验证坐标/角度链路；默认关闭）
  // 打开时会在状态栏给出醒目警告，避免被误认为真实检测结果。
  bool demoSkeleton = false;

  // ---- 截图 / 录像 ----
  ui::CaptureConfig capCfg;        // 保存目录与格式（可被 config.json 覆盖）
  ui::VideoRecorder recorder;      // 录像器
  cv::Mat lastCanvas;              // 上一帧已绘制完的画面（截图/录像取它）
  std::string toast;               // 界面右下角的一句提示
  double toastUntil = 0.0;
};

AppState g;

// ---------------------------------------------------------------------------
//  按当前视图自动选择通道：
//    彩色图像 / 骨骼单独输出 / 骨骼坐标 -> 优先 rgb
//    深度图 / 点云三维                 -> 优先 depth
//  界面上不再暴露通道选项，避免与视图区的选择重复而产生冲突。
// ---------------------------------------------------------------------------
// 界面提示：右下角显示若干秒
void showToast(const std::string& msg, double seconds = 4.0) {
  g.toast = msg;
  g.toastUntil = ImGui::GetTime() + seconds;
  std::printf("[ui] %s\n", msg.c_str());
  std::fflush(stdout);
}

// 截图：把当前画面存成图片
void doSnapshot() {
  const ui::SnapshotResult r = ui::saveSnapshot(g.lastCanvas, g.capCfg);
  showToast(r.message);
}

// 开始/停止录制
void toggleRecording() {
  if (g.recorder.recording()) {
    const std::string p = g.recorder.stop();
    char buf[512];
    std::snprintf(buf, sizeof(buf), "录制结束：%d 帧 / %.1f 秒 -> %s", g.recorder.frameCount(),
                  g.recorder.seconds(), p.c_str());
    showToast(buf);
  } else {
    std::string msg;
    if (!g.recorder.start(g.lastCanvas, g.capCfg, &msg)) {
      showToast(msg);
    } else {
      showToast(msg);
    }
  }
}

void applyAutoChannel() {
  std::vector<ui::PhysicalDevice> devs;
  {
    std::lock_guard<std::mutex> lk(g.deviceMtx);
    devs = g.devices;
  }
  if (devs.empty()) return;
  if (g.selectedDeviceIdx < 0 || g.selectedDeviceIdx >= static_cast<int>(devs.size())) {
    g.selectedDeviceIdx = 0;
  }
  const ui::PhysicalDevice& d = devs[static_cast<size_t>(g.selectedDeviceIdx)];

  const bool wantDepth =
      (g.view == ui::ViewMode::Depth || g.view == ui::ViewMode::PointCloud);
  const char* wanted = wantDepth ? "depth" : "rgb";

  const ui::DeviceChannel* pick = d.findChannel(wanted);
  if (!pick || !pick->openable) pick = d.findChannel(wantDepth ? "ir" : "rgb");
  if (!pick || !pick->openable) pick = d.preferredColorChannel();
  if (!pick) return;

  if (g.opt.cameraIndex != pick->index) {
    g.opt.cameraIndex = pick->index;
    g.pipeline.requestCamera(true, pick->index);
    std::printf("[ui] 视图切换 -> 使用通道 %s (index %d)\n", pick->kind.c_str(), pick->index);
    std::fflush(stdout);
  }
}

// ---------------------------------------------------------------------------
//  摄像头设备扫描（后台线程，避免卡住界面）
//    1) 先用 DirectShow 枚举拿设备友好名（毫秒级）
//    2) 再逐索引实测能否打开并抓到帧（每个约 1-3 秒）
// ---------------------------------------------------------------------------
void runDeviceScan() {
  std::vector<ui::PhysicalDevice> found = ui::enumeratePhysicalDevices();
  ui::probePhysicalDevices(found, 8);
  {
    std::lock_guard<std::mutex> lk(g.deviceMtx);
    g.devices = std::move(found);
  }
  g.scanning.store(false);

  std::lock_guard<std::mutex> lk(g.deviceMtx);
  std::printf("[ui] 设备扫描完成：%d 台物理设备\n", static_cast<int>(g.devices.size()));
  for (const auto& d : g.devices) {
    std::printf("[ui]   设备: %s%s%s\n", d.displayName.c_str(),
                d.pid.empty() ? "" : (" [PID " + d.pid + "]").c_str(),
                d.isOrbbec ? "  (Orbbec)" : "");
    for (const auto& c : d.channels) {
      std::printf("[ui]     通道 %-6s index=%d %dx%d 亮度%.1f %s\n", c.kind.c_str(), c.index,
                  c.width, c.height, c.firstFrameMean,
                  c.deliversFrames ? "可取帧" : (c.openable ? "能开无帧" : "打不开"));
    }
  }
  std::fflush(stdout);
}

// 退出前必须 join 扫描线程：AppState 是全局对象，若 std::thread 在析构时仍
// joinable，std::thread 的析构函数会直接调用 std::terminate()（表现为
// 0xC0000409 / 0xC0000374 崩溃）。这是之前"扫描完成即崩"的真正原因。
void shutdownDeviceScan() {
  g.scanning.store(false);
  if (g.scanThread.joinable()) g.scanThread.join();
}

void requestDeviceScan() {
  if (g.scanning.exchange(true)) return;
  if (g.scanThread.joinable()) g.scanThread.join();
  g.scanThread = std::thread(runDeviceScan);
}

// ---------------------------------------------------------------------------
//  小组件：设置分组的标题条（纯色块 + 小色条，无贴图）
// ---------------------------------------------------------------------------
void sectionHeader(const char* title) {
  ImGui::Spacing();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const float h = ImGui::GetTextLineHeight();
  // 左侧主色小竖条
  dl->AddRectFilled(ImVec2(p.x, p.y + 2), ImVec2(p.x + 3, p.y + h - 1),
                    ImGui::GetColorU32(ImGuiCol_SliderGrab), 2.f);
  ImGui::SetCursorScreenPos(ImVec2(p.x + 11, p.y));
  ImGui::TextUnformatted(title);
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();
}

// 状态圆点 + 文案（表示相机/推理是否正常）
void statusDot(bool ok, const char* text) {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float r = 5.f;
  const float y = p.y + ImGui::GetTextLineHeight() * 0.5f;
  dl->AddCircleFilled(ImVec2(p.x + r, y), r,
                      ok ? IM_COL32(80, 220, 120, 255) : IM_COL32(230, 90, 90, 255));
  ImGui::Dummy(ImVec2(r * 2 + 6, 0));
  ImGui::SameLine(0, 0);
  ImGui::TextUnformatted(text);
}

// ---------------------------------------------------------------------------
//  左侧：相机面板
// ---------------------------------------------------------------------------
void drawCameraPanel() {
  ImGui::Begin("相机 / 数据源", nullptr, ImGuiWindowFlags_NoCollapse);

  // ==========================================================================
  //  设备选择（置顶）：按唯一 SN 聚合成一台设备一行，通道在设备内部切换
  // ==========================================================================
  sectionHeader("设备选择");
  {
    if (ImGui::Button(g.scanning.load() ? "扫描中…" : "重新扫描设备", ImVec2(-1, 0))) {
      requestDeviceScan();
    }

    std::vector<ui::PhysicalDevice> devs;
    {
      std::lock_guard<std::mutex> lk(g.deviceMtx);
      devs = g.devices;
    }

    // 显示名：只用设备名称；**名称重复时才**附 PID 区分（识别号一律不展示）
    std::vector<std::string> labels;
    labels.reserve(devs.size());
    for (size_t i = 0; i < devs.size(); ++i) {
      std::string name = devs[i].displayName.empty() ? std::string("(未命名设备)")
                                                     : devs[i].displayName;
      bool duplicated = false;
      for (size_t j = 0; j < devs.size(); ++j) {
        if (i != j && devs[j].displayName == devs[i].displayName) { duplicated = true; break; }
      }
      if (duplicated && !devs[i].pid.empty()) name += "  [PID " + devs[i].pid + "]";
      labels.push_back(std::move(name));
    }

    const char* preview = "（未检测到设备）";
    if (!devs.empty()) {
      if (g.selectedDeviceIdx >= static_cast<int>(devs.size())) g.selectedDeviceIdx = 0;
      preview = labels[static_cast<size_t>(g.selectedDeviceIdx)].c_str();
    }
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##devcombo", preview)) {
      for (int i = 0; i < static_cast<int>(devs.size()); ++i) {
        const ui::PhysicalDevice& d = devs[static_cast<size_t>(i)];
        // 有可用彩色通道才算"可用"
        const ui::DeviceChannel* color = d.preferredColorChannel();
        const bool ok = color && color->deliversFrames;
        char label[384];
        std::snprintf(label, sizeof(label), "%s  (%d 个通道)%s",
                      labels[static_cast<size_t>(i)].c_str(),
                      static_cast<int>(d.channels.size()), ok ? "" : "  不可用");
        if (ImGui::Selectable(label, i == g.selectedDeviceIdx)) {
          g.selectedDeviceIdx = i;
          // 选中设备后：按当前视图自动选到合适的通道（彩色视图用 rgb，
          // 深度/点云视图用 depth），界面上不再让用户手动挑通道。
          applyAutoChannel();
        }
        if (ImGui::IsItemHovered()) {
          std::string tip = labels[static_cast<size_t>(i)] + "\n通道:";
          for (const auto& c : d.channels) {
            char line[160];
            std::snprintf(line, sizeof(line), "\n  %s (index %d, %dx%d) %s", c.kind.c_str(),
                          c.index, c.width, c.height,
                          c.deliversFrames ? "可取帧"
                                           : (c.openable ? "能开无帧" : "打不开"));
            tip += line;
          }
          ImGui::SetTooltip("%s", tip.c_str());
        }
      }
      ImGui::EndCombo();
    }

    if (devs.empty()) {
      ImGui::Spacing();
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.62f, 0.35f, 1.f));
      ImGui::TextWrapped(g.scanning.load() ? "正在枚举设备…"
                                           : "未检测到摄像头。请插好设备（USB 3.0 口），并确认未被 "
                                             "OrbbecViewer 等程序占用。");
      ImGui::PopStyleColor();
    } else if (g.selectedDeviceIdx >= 0 &&
               g.selectedDeviceIdx < static_cast<int>(devs.size())) {
      const ui::PhysicalDevice& d = devs[static_cast<size_t>(g.selectedDeviceIdx)];

    }
  }

  sectionHeader("相机控制");
  {
    bool on = g.opt.cameraOn;
    if (ImGui::Checkbox("开启相机", &on)) {
      g.opt.cameraOn = on;
      g.pipeline.requestCamera(on, g.opt.cameraIndex);
    }
    ImGui::SameLine();
    if (ImGui::Button("刷新 / 重连")) {
      g.pipeline.requestReopen();
    }
    // 相机状态
    statusDot(g.pipeline.cameraOpen(), g.pipeline.cameraOpen() ? "已连接" : "未连接");
    const std::string err = g.pipeline.cameraError();
    if (!err.empty()) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.55f, 0.55f, 1.f));
      ImGui::TextWrapped("%s", err.c_str());
      ImGui::PopStyleColor();
    }
  }

  sectionHeader("设备与格式");
  {


    static const char* kBands[] = {"640 x 480", "1280 x 720", "1920 x 1080"};
    static const int kW[] = {640, 1280, 1920};
    static const int kH[] = {480, 720, 1080};
    int bandIdx = 1;
    for (int i = 0; i < 3; ++i) {
      if (g.opt.width == kW[i] && g.opt.height == kH[i]) bandIdx = i;
    }
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##res", &bandIdx, kBands, 3)) {
      g.opt.width = kW[bandIdx];
      g.opt.height = kH[bandIdx];
      g.pipeline.requestReopen();
    }

    static const char* kBackends[] = {"auto（需要深度时用 obsensor）", "dshow", "obsensor",
                                      "msmf", "any"};
    static const char* kBackendIds[] = {"auto", "dshow", "obsensor", "msmf", "any"};
    int bIdx = 0;
    for (int i = 0; i < 5; ++i) {
      if (g.opt.captureBackend == kBackendIds[i]) bIdx = i;
    }
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##backend", &bIdx, kBackends, 5)) {
      g.opt.captureBackend = kBackendIds[bIdx];
      g.pipeline.requestReopen();
    }

    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##fps", &g.opt.fps, 5, 60, "目标帧率 %d");
  }

  sectionHeader("视频流");
  {
    bool color = true;  // 彩色流始终开启（界面需要画面）
    ImGui::BeginDisabled(true);
    ImGui::Checkbox("彩色图像流", &color);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("(必需)");

    bool depth = g.opt.depthEnabled;
    if (ImGui::Checkbox("深度流（obsensor）", &depth)) {
      g.opt.depthEnabled = depth;
      g.cloudDirty = true;
      g.pipeline.requestReopen();  // 深度需要 obsensor 后端，重开相机
    }
    if (depth && !g.sample.usingObsensor) {
      ImGui::TextDisabled("等待 obsensor 后端生效...");
    }
  }

  sectionHeader("骨骼检测");
  {
    bool pose = g.opt.poseEnabled;
    if (ImGui::Checkbox("启用人形骨骼检测", &pose)) {
      g.opt.poseEnabled = pose;
      g.pipeline.requestReopen();  // 让采集线程重新加载模型
    }
    ImGui::TextDisabled("模型：YOLOv8-pose (ONNX)");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##score", &g.opt.poseScoreThreshold, 0.05f, 0.9f, "人体阈值 %.2f");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##kpt", &g.opt.poseKptThreshold, 0.05f, 0.9f, "关键点阈值 %.2f");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##persons", &g.opt.maxPersons, 1, 8, "最多人数 %d");

    ImGui::Spacing();
    ImGui::Checkbox("显示演示骨骼（示例，非真实检测）", &g.demoSkeleton);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("画一副合成的火柴人，用来演示坐标/角度链路是否正常。\n"
                        "它不是摄像头检测到的真人骨骼——要看真实检测请关闭本项。");
    }
    if (g.demoSkeleton) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.62f, 0.25f, 1.0f));
      ImGui::TextWrapped("注意：当前显示的是演示骨骼，不代表真实人体检测结果");
      ImGui::PopStyleColor();
    }
  }

  ImGui::End();
}

// ---------------------------------------------------------------------------
//  右侧：显示设置
// ---------------------------------------------------------------------------
void drawDisplayPanel() {
  ImGui::Begin("显示设置", nullptr, ImGuiWindowFlags_NoCollapse);

  sectionHeader("截图与录制");
  {
    // 第一个按键：截图（所见即所得，含骨骼叠加与坐标标注）
    if (ImGui::Button("截图", ImVec2(-1, 34))) doSnapshot();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("把当前画面保存为图片（快捷键 S）\n目录: %s/%s",
                        g.capCfg.rootDir.c_str(), g.capCfg.snapshotSub.c_str());
    }

    // 第二个按键：录制（可反复开始/停止）
    const bool rec = g.recorder.recording();
    if (rec) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.16f, 0.16f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.84f, 0.22f, 0.22f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.60f, 0.12f, 0.12f, 1.0f));
    }
    if (ImGui::Button(rec ? "停止录制" : "录制", ImVec2(-1, 34))) toggleRecording();
    if (rec) ImGui::PopStyleColor(3);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("把当前画面录制为 AVI（MJPG）\n目录: %s/%s\n快捷键 R",
                        g.capCfg.rootDir.c_str(), g.capCfg.videoSub.c_str());
    }

    if (rec) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.45f, 1.0f));
      ImGui::Text("● 录制中  %d 帧 / %.1f 秒", g.recorder.frameCount(), g.recorder.seconds());
      ImGui::PopStyleColor();
      ImGui::PushTextWrapPos(0.0f);
      ImGui::TextDisabled("%s", g.recorder.path().c_str());
      ImGui::PopTextWrapPos();
    } else {
      ImGui::TextDisabled("未在录制");
    }
  }

  sectionHeader("视图");
  {
    const float w = ImGui::GetContentRegionAvail().x;
    const float bw = (w - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    struct Item {
      ui::ViewMode mode;
      const char* label;
    };
    const Item items[] = {{ui::ViewMode::Color, "彩色图像"},
                          {ui::ViewMode::Depth, "深度图"},
                          {ui::ViewMode::PointCloud, "点云三维"},
                          {ui::ViewMode::SkeletonOnly, "骨骼单独输出"},
                          {ui::ViewMode::SkeletonCoords, "骨骼坐标 (X,Y)"}};
    for (int i = 0; i < 5; ++i) {
      const bool active = (g.view == items[i].mode);
      if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_SliderGrab));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              ImGui::GetStyleColorVec4(ImGuiCol_SliderGrabActive));
      }
      if (ImGui::Button(items[i].label, ImVec2(bw, 34))) {
        g.view = items[i].mode;
        applyAutoChannel();
        g.cloudDirty = true;
      }
      if (active) ImGui::PopStyleColor(2);
      if (i % 2 == 0) ImGui::SameLine();
    }
  }

  sectionHeader("骨骼叠加");
  {
    ImGui::Checkbox("在画面上叠加骨骼", &g.style.bones);
    ImGui::SameLine();
    ImGui::TextDisabled("(彩色/深度视图)");
    ImGui::Checkbox("显示关节点", &g.style.joints);
    ImGui::Checkbox("标注关节角度", &g.style.jointAngles);
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##thick", &g.style.boneThickness, 1, 8, "骨骼线宽 %d");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##jrad", &g.style.jointRadius, 2, 10, "关节半径 %d");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##alpha", &g.style.alpha, 0.2f, 1.0f, "不透明度 %.2f");

    ImGui::Checkbox("骨骼位置平滑（减少抖动）", &g.smoothSkeleton);
  }

  sectionHeader("人体相对坐标系");
  {
    ImGui::TextWrapped("原点 = 地面 × 人体中心。X 向右为正、人体中心为 0；"
                       "Y 向上为正、地面为 0。数值为相对像素坐标，不做物理单位换算。");
    ImGui::Spacing();
    ImGui::Checkbox("关节旁标注 (X, Y)", &g.style.jointCoords);
    ImGui::SameLine();
    ImGui::Checkbox("地面网格", &g.style.coordGrid);
    ImGui::Checkbox("按身高归一化（地面 0 / 头顶约 1）", &g.style.coordNormalized);
    if (g.view != ui::ViewMode::SkeletonCoords) {
      ImGui::TextDisabled("提示：切到「骨骼坐标 (X,Y)」视图查看完整坐标系");
    }
    for (size_t i = 0; i < g.calib.size() && i < 2; ++i) {
      const pose::GroundCalib& c = g.calib[i];
      if (!c.valid) {
        ImGui::TextDisabled("参考系未就绪（需要看到踝关节与髋/肩）");
        continue;
      }
      ImGui::TextDisabled("地面 y=%.0f  中心 x=%.0f  身高 %.0f px",
                          static_cast<double>(c.groundYpx),
                          static_cast<double>(c.centerXpx), static_cast<double>(c.spanPx));
    }
    if (g.smoothSkeleton) {
      ImGui::SetNextItemWidth(-1);
      ImGui::SliderFloat("##salpha", &g.smoothAlpha, 0.0f, 0.9f, "平滑强度 %.2f");
    }
  }

  sectionHeader("深度显示");
  {
    static const char* kMaps[] = {"Turbo", "Jet", "Viridis", "Gray"};
    int m = static_cast<int>(g.opt.depthMap);
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##cmap", &m, kMaps, 4)) {
      g.opt.depthMap = static_cast<ui::DepthColorMap>(m);
    }
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##dmin", &g.opt.depthMinMm, 100, 2000, "最近 %d mm");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##dmax", &g.opt.depthMaxMm, 1000, 10000, "最远 %d mm");
    if (g.opt.depthMaxMm <= g.opt.depthMinMm + 50) {
      g.opt.depthMaxMm = g.opt.depthMinMm + 50;
    }
  }

  sectionHeader("点云");
  {
    ImGui::SetNextItemWidth(-1);
    if (ImGui::SliderInt("##stride", &g.opt.cloudStride, 1, 8, "抽样步长 %d")) {
      g.cloudDirty = true;
    }
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##psize", &g.opt.pointSize, 1.0f, 6.0f, "点大小 %.1f");
    if (ImGui::Checkbox("用彩色图着色", &g.opt.cloudUseColor)) g.cloudDirty = true;
    ImGui::Checkbox("显示地面网格", &g.cloudView.showGrid);
    ImGui::TextDisabled("左键拖拽旋转 / 滚轮缩放");
    if (ImGui::Button("重置视角", ImVec2(-1, 0))) {
      g.cloudView = ui::PointCloudView{};
    }
  }

  ImGui::End();
}

// ---------------------------------------------------------------------------
//  底部：状态栏
// ---------------------------------------------------------------------------
void drawStatusBar() {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float h = ImGui::GetFrameHeight() + 14.f;
  ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - h));
  ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, h));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14, 7));
  ImGui::Begin("##status", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

  const ui::Sample& s = g.sample;
  ImGui::Text("FPS %.1f", s.frameFps);
  ImGui::SameLine(0, 22);
  ImGui::Text("画面 %d x %d", s.colorW, s.colorH);
  ImGui::SameLine(0, 22);
  ImGui::Text("取流 %.1f ms", s.captureMs);
  ImGui::SameLine(0, 22);
  ImGui::Text("推理 %.1f ms", s.inferMs);
  ImGui::SameLine(0, 22);
  ImGui::Text("人数 %d", static_cast<int>(s.poses.skeletons.size()));
  if (g.view == ui::ViewMode::PointCloud) {
    ImGui::SameLine(0, 22);
    ImGui::Text("点云 %d 点 (%.1f ms)", g.cloudPoints, g.cloudBuildMs);
  }
  ImGui::SameLine(0, 22);
  if (g.demoSkeleton) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.72f, 0.2f, 1.0f));
    ImGui::TextUnformatted("演示骨骼（非真实检测）");
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();
  }
  statusDot(g.pipeline.cameraOpen(), g.pipeline.cameraOpen() ? "相机在线" : "相机离线");

  // 右侧提示
  const char* hint = "Q 退出  |  M 镜像  |  1/2/3/4 切换视图";
  const float tw = ImGui::CalcTextSize(hint).x;
  ImGui::SameLine(ImGui::GetWindowWidth() - tw - 18);
  ImGui::TextDisabled("%s", hint);

  ImGui::End();
  ImGui::PopStyleVar();
}

// ---------------------------------------------------------------------------
//  视口：按视图模式准备画面并绘制
// ---------------------------------------------------------------------------
void drawViewport() {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const float barH = ImGui::GetFrameHeight() + 14.f;
  const ImVec2 pos(vp->WorkPos.x, vp->WorkPos.y);
  const ImVec2 size(vp->WorkSize.x, vp->WorkSize.y - barH);

  ImGui::SetNextWindowPos(pos);
  ImGui::SetNextWindowSize(size);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::Begin("##viewport", nullptr,
               ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  ImDrawList* dl = ImGui::GetWindowDrawList();

  // 视口底色（纯色块）
  dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y),
                    IM_COL32(14, 15, 19, 255));

  if (g.view == ui::ViewMode::PointCloud) {
    // ---- 点云：OpenGL 离屏渲染 ----
    if (g.sample.hasIntrinsics && g.sample.depthW > 0 && g.cloudDirty) {
      const auto t0 = std::chrono::steady_clock::now();
      g.cloudPoints = ui::buildPointCloud(
          g.sample.depthMm, g.sample.depthW, g.sample.depthH, g.sample.fx, g.sample.fy,
          g.sample.icx, g.sample.icy, g.opt.cloudStride, g.opt.depthMinMm,
          g.opt.depthMaxMm, g.sample.bgr.empty() ? nullptr : &g.sample.bgr, g.sample.colorW,
          g.sample.colorH, g.opt.cloudUseColor, g.cloudPos, g.cloudCol);
      g.cloud.upload(g.cloudPos, g.cloudCol, g.opt.pointSize);
      g.cloudBuildMs =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
              .count();
      g.cloudDirty = false;
    }

    const int vw = std::max(64, static_cast<int>(avail.x));
    const int vh = std::max(64, static_cast<int>(avail.y));

    // 鼠标交互（仅在悬停视口时）
    ImGui::InvisibleButton("##cloudcanvas", avail,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      const ImVec2 d = ImGui::GetIO().MouseDelta;
      g.cloudView.yawDeg += d.x * 0.35f;
      g.cloudView.pitchDeg = std::max(-85.f, std::min(85.f, g.cloudView.pitchDeg - d.y * 0.35f));
    }
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      const ImVec2 d = ImGui::GetIO().MouseDelta;
      g.cloudView.panX -= d.x * 0.0015f * g.cloudView.distM;
      g.cloudView.panY += d.y * 0.0015f * g.cloudView.distM;
    }
    if (hovered) {
      const float wheel = ImGui::GetIO().MouseWheel;
      if (wheel != 0.f) {
        g.cloudView.distM = std::max(0.3f, std::min(15.f, g.cloudView.distM * (1.f - wheel * 0.12f)));
      }
    }

    const unsigned int tex = g.cloud.render(vw, vh, g.cloudView);
    if (tex != 0) {
      // OpenGL 纹理原点在左下，ImGui 需要翻转 UV
      dl->AddImage(static_cast<ImTextureID>(static_cast<uintptr_t>(tex)), origin,
                   ImVec2(origin.x + avail.x, origin.y + avail.y), ImVec2(0, 1), ImVec2(1, 0));
    } else {
      dl->AddText(ImVec2(origin.x + 18, origin.y + 18), IM_COL32(230, 120, 120, 255),
                  g.cloud.lastError());
    }
    if (g.cloudPoints == 0) {
      const char* msg = g.sample.hasIntrinsics
                            ? "等待深度数据…（请勾选左侧「深度流（obsensor）」）"
                            : "当前相机未提供内参，无法生成点云。请用 obsensor 后端并开启深度流。";
      dl->AddText(ImVec2(origin.x + 18, origin.y + avail.y - 30),
                  IM_COL32(200, 200, 210, 220), msg);
    }
  } else {
    // ---- 2D 视图：彩色 / 深度 / 骨骼 ----
    cv::Mat& img = g.canvas;
    if (g.view == ui::ViewMode::Depth) {
      if (g.sample.depthW > 0) {
        ui::colorizeDepth(g.sample.depthMm, g.sample.depthW, g.sample.depthH,
                          g.opt.depthMap, g.opt.depthMinMm, g.opt.depthMaxMm, img);
      } else {
        ui::ensureBgr(img, 640, 360, cv::Scalar(18, 18, 22));
      }
      // 深度图上叠加骨骼（深度与彩色分辨率不同，按比例缩放关节）
      if (g.style.bones && g.sample.depthW > 0 && g.sample.colorW > 0 &&
          !g.sample.poses.skeletons.empty()) {
        ui::Sample tmp = g.sample;
        const float sx = static_cast<float>(g.sample.depthW) / g.sample.colorW;
        const float sy = static_cast<float>(g.sample.depthH) / g.sample.colorH;
        for (auto& sk : tmp.poses.skeletons) {
          for (int i = 0; i < pose::kJointCount; ++i) {
            if (sk.joints2d[i].valid) {
              sk.joints2d[i].x *= sx;
              sk.joints2d[i].y *= sy;
            }
          }
        }
        ui::drawSkeletonOverlay(img, tmp.poses, g.sample.angles, g.style);
      }
    } else if (g.view == ui::ViewMode::SkeletonCoords) {
      const int w = g.sample.colorW > 0 ? g.sample.colorW : 1280;
      const int h = g.sample.colorH > 0 ? g.sample.colorH : 720;
      ui::ensureBgr(img, w, h, cv::Scalar(16, 17, 21));

      // 坐标视图自己处理镜像（原始坐标未在上方被镜像）
      if (g.opt.mirror) {
        for (auto& sk : g.sample.poses.skeletons) {
          for (int i = 0; i < pose::kJointCount; ++i) {
            if (sk.joints2d[i].valid) {
              sk.joints2d[i].x = static_cast<float>(w - 1) - sk.joints2d[i].x;
            }
          }
        }
      }
      // 坐标视图的镜像在此处理（上面的全局镜像已对 SkeletonCoords 跳过）
      if (g.opt.mirror) {
        for (auto& sk : g.sample.poses.skeletons) {
          for (int i = 0; i < pose::kJointCount; ++i) {
            if (sk.joints2d[i].valid) {
              sk.joints2d[i].x = static_cast<float>(w - 1) - sk.joints2d[i].x;
            }
          }
        }
      }
      ui::drawCoordView(img, g.sample.poses, g.style, cv::Scalar(16, 17, 21), &g.calib);
    } else if (g.view == ui::ViewMode::SkeletonOnly) {
      const int w = g.sample.colorW > 0 ? g.sample.colorW : 1280;
      const int h = g.sample.colorH > 0 ? g.sample.colorH : 720;
      ui::ensureBgr(img, w, h, cv::Scalar(16, 17, 21));

      ui::drawSkeletonOnly(img, g.sample.poses, g.sample.angles, g.style,
                           cv::Scalar(16, 17, 21));
    } else {  // Color
      if (!g.sample.bgr.empty()) {
        cv::Mat view(g.sample.colorH, g.sample.colorW, CV_8UC3,
                     const_cast<uint8_t*>(g.sample.bgr.data()));
        view.copyTo(img);
        if (g.opt.mirror) cv::flip(img, img, 1);   // 只翻画面，骨骼已在统一入口镜像
        if (g.style.bones) {
          ui::drawSkeletonOverlay(img, g.sample.poses, g.sample.angles, g.style);
        }
      } else {
        ui::ensureBgr(img, 1280, 720, cv::Scalar(16, 17, 21));
      }
    }

    static GlTexture tex;
    tex.uploadBgr(img);
    if (tex.id() != 0) {
      // 等比例适应视口，居中显示
      const float ia = static_cast<float>(tex.width()) / std::max(1, tex.height());
      const float va = avail.x / std::max(1.f, avail.y);
      float dw = avail.x, dh = avail.y;
      if (ia > va) {
        dh = avail.x / ia;
      } else {
        dw = avail.y * ia;
      }
      const ImVec2 p0(origin.x + (avail.x - dw) * 0.5f, origin.y + (avail.y - dh) * 0.5f);
      dl->AddImage(static_cast<ImTextureID>(static_cast<uintptr_t>(tex.id())), p0,
                   ImVec2(p0.x + dw, p0.y + dh));
    }

    // 顶部提示条（纯色块）
    if (!g.sample.valid) {
      dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + 30),
                        IM_COL32(30, 32, 40, 230));
      dl->AddText(ImVec2(origin.x + 12, origin.y + 7), IM_COL32(230, 200, 120, 255),
                  g.sample.note.empty() ? "等待相机画面…" : g.sample.note.c_str());
    }
  }

  ImGui::End();
  ImGui::PopStyleVar();
}

// ---------------------------------------------------------------------------
//  键盘快捷键
// ---------------------------------------------------------------------------
void handleShortcuts(GLFWwindow* win) {
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput) return;
  if (ImGui::IsKeyPressed(ImGuiKey_M)) g.opt.mirror = !g.opt.mirror;
  if (ImGui::IsKeyPressed(ImGuiKey_S)) doSnapshot();
  if (ImGui::IsKeyPressed(ImGuiKey_R)) toggleRecording();
  if (ImGui::IsKeyPressed(ImGuiKey_1)) g.view = ui::ViewMode::Color;
  if (ImGui::IsKeyPressed(ImGuiKey_2)) g.view = ui::ViewMode::Depth;
  if (ImGui::IsKeyPressed(ImGuiKey_3)) g.view = ui::ViewMode::PointCloud;
  if (ImGui::IsKeyPressed(ImGuiKey_4)) g.view = ui::ViewMode::SkeletonOnly;
  if (ImGui::IsKeyPressed(ImGuiKey_Escape)) glfwSetWindowShouldClose(win, 1);
}

}  // namespace

// ---------------------------------------------------------------------------
//  main
// ---------------------------------------------------------------------------
// 退出守卫：保证任何返回路径都先"收线程 -> 停采集 -> 释放 GL 资源"，
// 再让全局 AppState 析构。否则全局 std::thread 在析构时仍 joinable 会
// 触发 std::terminate（表现为访问冲突 / fast-fail）。
struct AppShutdownGuard {
  bool done = false;
  void run() {
    if (done) return;
    done = true;
    std::printf("[ui] 正在退出：停止采集与设备扫描…\n");
    std::fflush(stdout);
    if (g.recorder.recording()) {
      const std::string p = g.recorder.stop();
      std::printf("[ui] 退出前结束录制: %s (%d 帧)\n", p.c_str(), g.recorder.frameCount());
      std::fflush(stdout);
    }
    if (g.scanThread.joinable()) g.scanThread.join();
    g.scanning.store(false);
    g.pipeline.stop();
    g.cloud.shutdown();
  }
  ~AppShutdownGuard() { run(); }
};

int main(int argc, char** argv) {
  AppShutdownGuard shutdownGuard;

  // ---- 读取 config.json（含"开发者模式"开关）----
  // 默认开发者模式 = 开：保留命令行窗口，便于查看设备扫描、通道切换等日志。
  const ui::AppConfigFile fileCfg = ui::loadAppConfigFile("config.json");

  // 命令行可强制覆盖：--dev / --no-dev
  bool devMode = fileCfg.developerMode;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--dev") devMode = true;
    else if (a == "--no-dev" || a == "--hide-console") devMode = false;
  }
  // 默认隐藏命令行窗口；隐藏时把运行日志写入 pose_ui.log，便于排查。
  // （开发者模式打开则保留窗口直接看日志）
  static std::FILE* logFile = nullptr;
  if (fileCfg.logToFile) {
    logFile = std::freopen("pose_ui.log", "w", stdout);
  }
  if (devMode) {
    if (logFile) {
      // 需要同时输出到窗口时，改用"只写文件 + 窗口另打印一份"的做法：
      // 这里简单处理——窗口模式不重定向，直接关掉文件重定向回到控制台
      std::fclose(logFile);
      logFile = nullptr;
      std::freopen("CONOUT$", "w", stdout);
    }
  }
  ui::applyConsoleVisibility(devMode);

#if defined(_WIN32)
  if (devMode && GetConsoleWindow()) {
    SetConsoleTitleW(L"pose_ui - 运行日志（关闭本窗口会结束程序）");
  }
#endif
  // 打印启动信息（开发者模式下用户能在命令行窗口里直接看到）
  std::printf("=== pose_ui ===\n");
  std::printf("config.json : %s\n", fileCfg.status.c_str());
  std::printf("开发者模式  : %s（命令行窗口%s）\n", devMode ? "开" : "关",
              devMode ? "保持显示" : "已隐藏");
  std::fflush(stdout);
  // 允许用命令行覆盖模型路径 / 起始视图，方便打包与自动化截图
  std::string modelPath = "models/yolov8n-pose.onnx";
  std::string shotPath;
  int exitAfterMs = 0;
  double shotDelaySec = 6.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--model" && i + 1 < argc) modelPath = argv[++i];
    else if (a == "--screenshot" && i + 1 < argc) shotPath = argv[++i];
    else if (a == "--exit-after-ms" && i + 1 < argc) exitAfterMs = std::atoi(argv[++i]);
    else if (a == "--no-camera") g.opt.cameraOn = false;
    else if (a == "--depth") g.opt.depthEnabled = true;
    else if (a == "--view" && i + 1 < argc) {
      const std::string v = argv[++i];
      if (v == "color") g.view = ui::ViewMode::Color;
      else if (v == "depth") g.view = ui::ViewMode::Depth;
      else if (v == "cloud" || v == "pointcloud") g.view = ui::ViewMode::PointCloud;
      else if (v == "skeleton") g.view = ui::ViewMode::SkeletonOnly;
      else if (v == "coords" || v == "coord") g.view = ui::ViewMode::SkeletonCoords;
    } else if (a == "--shot-delay" && i + 1 < argc) {
      shotDelaySec = std::atof(argv[++i]);
    } else if (a == "--backend" && i + 1 < argc) {
      g.opt.captureBackend = argv[++i];
    } else if (a == "--no-mirror") {
      g.opt.mirror = false;
    } else if (a == "--mirror") {
      g.opt.mirror = true;
    }
  }

  // ---- 自动化抓图/录制（便于无人值守或脚本调用）----
  //   --snap-after 秒   启动后延迟指定秒数自动截图一张
  //   --record 秒       启动后立即录制指定秒数并保存
  double autoSnapAfter = -1.0;
  double autoRecordSecs = -1.0;
  for (int i = 1; i + 1 < argc; ++i) {
    const std::string a = argv[i];
    try {
      if (a == "--snap-after") autoSnapAfter = std::stod(argv[i + 1]);
      else if (a == "--record") autoRecordSecs = std::stod(argv[i + 1]);
    } catch (...) {
    }
  }

  // ---- 套用 config.json（命令行已显式给出的项优先，不被覆盖）----
  {
    // cameraIndex：仅当命令行没给 --camera 时套用
    bool camGiven = false;
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--camera") { camGiven = true; break; }
    }
    if (!camGiven) {
      g.opt.cameraIndex = fileCfg.cameraIndex;
      g.selectedDeviceIdx = 0;
    }
    if (!fileCfg.captureBackend.empty()) g.opt.captureBackend = fileCfg.captureBackend;
    g.opt.poseEnabled = fileCfg.poseEnabled;
    g.opt.depthEnabled = fileCfg.depthStream;
    g.opt.mirror = fileCfg.mirror;
    g.opt.modelPath = fileCfg.poseModel;
    g.capCfg.rootDir = fileCfg.captureDir.empty() ? "captures" : fileCfg.captureDir;
    g.capCfg.videoFps = fileCfg.videoFps;

    // 视图：命令行有 --view 时以命令行为准
    bool viewGiven = false;
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--view") { viewGiven = true; break; }
    }
    if (!viewGiven) {
      const std::string v = fileCfg.view;
      if (v == "color") g.view = ui::ViewMode::Color;
      else if (v == "depth") g.view = ui::ViewMode::Depth;
      else if (v == "cloud" || v == "pointcloud") g.view = ui::ViewMode::PointCloud;
      else if (v == "skeleton") g.view = ui::ViewMode::SkeletonOnly;
      else if (v == "coords") g.view = ui::ViewMode::SkeletonCoords;
    }
    std::printf("已套用配置: 相机索引=%d 视图=%s 镜像=%d 深度流=%d 骨骼=%d\n",
                g.opt.cameraIndex, ui::viewModeName(g.view), g.opt.mirror ? 1 : 0,
                g.opt.depthEnabled ? 1 : 0, g.opt.poseEnabled ? 1 : 0);
    std::fflush(stdout);
  }

  // ---- 诊断用：合成一副完整全身骨骼（验证坐标换算，不依赖相机与人体入画）----
  // 与界面上的「显示演示骨骼」共用同一个状态，这样警告提示一定会出现，
  // 避免把演示火柴人误当成真实检测结果。
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--test-skeleton") g.demoSkeleton = true;
  }
  std::printf("[ui] pose_ui %s\n", kAppVersion);
  std::printf("[ui] args: view=%d depth=%d backend=%s model=%s shot=%s delay=%.1f\n",
              static_cast<int>(g.view), g.opt.depthEnabled ? 1 : 0,
              g.opt.captureBackend.c_str(), modelPath.c_str(), shotPath.c_str(),
              shotDelaySec);
  std::fflush(stdout);

  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW 初始化失败\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow* win = glfwCreateWindow(1680, 980, "Orbbec Astra+ 骨骼与三维可视化  v2.0", nullptr, nullptr);
  if (!win) {
    std::fprintf(stderr, "创建窗口失败（需要 OpenGL 3.3）\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  // 先加载 GL 函数指针（点云渲染需要 GL 3.3 的 FBO/VBO/着色器）
  if (!gl::init()) {
    std::fprintf(stderr, "[warn] OpenGL 加载不完整：%s\n", gl::missingFunctions());
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;  // 不落地布局文件，保证每次启动一致
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  loadFonts();               // 必须在后端 Init 之前加载字体
  applyModernTheme();

  if (!ImGui_ImplGlfw_InitForOpenGL(win, true) || !ImGui_ImplOpenGL3_Init("#version 330")) {
    std::fprintf(stderr, "ImGui 初始化失败\n");
    return 1;
  }

  // ---- 采集管道 -----------------------------------------------------------
  g.opt.modelPath = modelPath;
  g.opt.cameraOn = g.opt.cameraOn;  // 命令行可能已关闭
  g.pipeline.updateOptions(g.opt);
  g.pipeline.requestCamera(g.opt.cameraOn, g.opt.cameraIndex);
  g.pipeline.start();

  // 启动后台设备扫描：多摄像头时用户可在左侧列表里挑选正确的硬件
  g.selectedDeviceIdx = 0;
  requestDeviceScan();

  if (!g.cloud.init()) {
    std::fprintf(stderr, "[warn] 点云渲染器初始化失败：%s\n", g.cloud.lastError());
  }

  // 姿态平滑：保存上一帧关节位置
  std::vector<pose::Skeleton> smoothed;

  const double startTime = glfwGetTime();
  bool shotTaken = false;
  double lastDevScan = 0.0;   // 上次设备扫描时间（用于自动重扫）

  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();

    // 取最新一帧（拷贝，后续可自由修改关节坐标做镜像/缩放）
    ui::Sample fresh;
    if (g.pipeline.fetchLatest(fresh)) {
      g.sample = std::move(fresh);
      if (g.sample.timestampUs != g.lastFrameStamp) {
        g.lastFrameStamp = g.sample.timestampUs;
        g.cloudDirty = true;
      }
    }
    if (!g.opt.cameraOn) {
      static const ui::Sample kEmpty{};
      if (!g.sample.valid) g.sample = kEmpty;
    }

    // ---- 诊断模式：用合成骨骼替换检测结果 ----
    if (g.demoSkeleton) {
      const int W = g.sample.colorW > 0 ? g.sample.colorW : 1280;
      const int H = g.sample.colorH > 0 ? g.sample.colorH : 720;
      if (g.sample.colorW == 0) {
        g.sample.colorW = W;
        g.sample.colorH = H;
      }
      g.sample.poses.skeletons.clear();
      pose::Skeleton sk;
      sk.id = 99;
      sk.globalConfidence = 1.f;
      // 站立姿势：脚底 y=640（地面），头顶 y=140，人体中心 x=640
      const float groundY = 640.f;
      const float centerX = 640.f;
      auto put = [&](pose::JointId id, float x, float y) {
        const int k = static_cast<int>(id);
        sk.joints2d[k] = pose::Vec2{x, y, true};
        sk.conf[k] = 1.f;
      };
      put(pose::JointId::Head, centerX + 4.f, 150.f);
      put(pose::JointId::Neck, centerX, 200.f);
      put(pose::JointId::Spine, centerX, 300.f);
      put(pose::JointId::ShoulderL, centerX - 90.f, 215.f);
      put(pose::JointId::ElbowL, centerX - 140.f, 340.f);
      put(pose::JointId::WristL, centerX - 165.f, 460.f);
      put(pose::JointId::ShoulderR, centerX + 90.f, 215.f);
      put(pose::JointId::ElbowR, centerX + 140.f, 340.f);
      put(pose::JointId::WristR, centerX + 165.f, 460.f);
      put(pose::JointId::HipL, centerX - 55.f, 430.f);
      put(pose::JointId::KneeL, centerX - 62.f, 535.f);
      put(pose::JointId::AnkleL, centerX - 66.f, 632.f);
      put(pose::JointId::HipR, centerX + 55.f, 430.f);
      put(pose::JointId::KneeR, centerX + 62.f, 535.f);
      put(pose::JointId::AnkleR, centerX + 66.f, 632.f);
      put(pose::JointId::FootL, centerX - 66.f, 632.f);
      put(pose::JointId::FootR, centerX + 66.f, 632.f);
      g.sample.poses.skeletons.push_back(sk);
    }

    // 骨骼平滑（仅显示用；角度仍由原始坐标计算）
    if (g.smoothSkeleton && !g.sample.poses.skeletons.empty()) {
      if (smoothed.size() != g.sample.poses.skeletons.size()) {
        smoothed = g.sample.poses.skeletons;
      } else {
        const float a = g.smoothAlpha;
        for (size_t si = 0; si < smoothed.size(); ++si) {
          for (int j = 0; j < pose::kJointCount; ++j) {
            const pose::Vec2& cur = g.sample.poses.skeletons[si].joints2d[j];
            pose::Vec2& prev = smoothed[si].joints2d[j];
            if (cur.valid) {
              if (prev.valid) {
                prev.x = prev.x * a + cur.x * (1.f - a);
                prev.y = prev.y * a + cur.y * (1.f - a);
              } else {
                prev = cur;
              }
            } else {
              prev.valid = false;
            }
          }
        }
        g.sample.poses.skeletons = smoothed;
      }
    }

    // ---- 镜像：只作用于"本帧的渲染副本"，绝对不写回平滑状态 ----
    // 顺序很关键：平滑必须先于镜像，且只平滑"未镜像"的原始坐标。
    // 否则镜像后的坐标会被存进平滑状态、下一帧又被镜像一次，
    // 结果每帧翻转 180°（X、Y 同时取反），表现为画面上下颠倒。
    // 骨骼坐标视图由其自身分支处理镜像，这里跳过以免重复。
    if (g.opt.mirror && g.sample.colorW > 0 &&
        g.view != ui::ViewMode::SkeletonCoords) {
      const float wMinus1 = static_cast<float>(g.sample.colorW - 1);
      for (auto& sk : g.sample.poses.skeletons) {
        for (int i = 0; i < pose::kJointCount; ++i) {
          if (sk.joints2d[i].valid) {
            sk.joints2d[i].x = wMinus1 - sk.joints2d[i].x;
          }
        }
      }
    }
    // 设备列表里没有可用设备时，每隔几秒自动重扫一次：
    // 插上相机后无需手动点「重新扫描」就会自动出现在列表里。
    if (!g.scanning.load()) {
      bool anyUsable = false;
      {
        std::lock_guard<std::mutex> lk(g.deviceMtx);
        for (const auto& d : g.devices) {
          for (const auto& c : d.channels) {
            if (c.deliversFrames) { anyUsable = true; break; }
          }
          if (anyUsable) break;
        }
      }
      if (!anyUsable && ImGui::GetTime() - lastDevScan > 3.0) {
        lastDevScan = ImGui::GetTime();
        if (g.deviceScanAttempts < 40) {   // 最多自动重试约 2 分钟
          ++g.deviceScanAttempts;
          requestDeviceScan();
        }
      }
    }

    handleShortcuts(win);

    // ---- 绘制 -------------------------------------------------------------
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float barH = ImGui::GetFrameHeight() + 14.f;
    const float sideW = 306.f;

    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + sideW, vp->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x - sideW * 2.f, vp->WorkSize.y - barH));
    drawViewport();

    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(sideW, vp->WorkSize.y - barH));
    drawCameraPanel();

    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - sideW, vp->WorkPos.y));
    ImGui::SetNextWindowSize(ImVec2(sideW, vp->WorkSize.y - barH));
    drawDisplayPanel();

    drawStatusBar();

    // 提示条：右下角显示"已截图/录制中"等反馈
    const bool recOn = g.recorder.recording();
    if (recOn) {
      char rb[256];
      std::snprintf(rb, sizeof(rb), "● 录制中  %d 帧 / %.1f 秒", g.recorder.frameCount(),
                    g.recorder.seconds());
      g.toast = rb;
      g.toastUntil = ImGui::GetTime() + 1.0;
    }
    if (!g.toast.empty() && ImGui::GetTime() < g.toastUntil) {
      const ImGuiViewport* vp = ImGui::GetMainViewport();
      const float pad = 16.0f;
      const ImVec2 tsz = ImGui::CalcTextSize(g.toast.c_str());
      const ImVec2 pos(vp->WorkPos.x + vp->WorkSize.x - tsz.x - pad * 2,
                       vp->WorkPos.y + vp->WorkSize.y - tsz.y - pad * 3.4f);
      ImDrawList* dl = ImGui::GetForegroundDrawList();
      dl->AddRectFilled(ImVec2(pos.x - pad * 0.5f, pos.y - pad * 0.4f),
                        ImVec2(pos.x + tsz.x + pad * 0.5f, pos.y + tsz.y + pad * 0.4f),
                        IM_COL32(24, 26, 32, 235), 6.0f);
      dl->AddRect(ImVec2(pos.x - pad * 0.5f, pos.y - pad * 0.4f),
                  ImVec2(pos.x + tsz.x + pad * 0.5f, pos.y + tsz.y + pad * 0.4f),
                  IM_COL32(90, 150, 240, 255), 6.0f);
      dl->AddText(pos, IM_COL32(235, 240, 250, 255), g.toast.c_str());
    }

    ImGui::Render();
    int fbW = 0, fbH = 0;
    glfwGetFramebufferSize(win, &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
    glClearColor(0.055f, 0.06f, 0.075f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // ---- 自动化截图（打包/验证用）----------------------------------------
    if (!shotPath.empty() && !shotTaken && glfwGetTime() - startTime > shotDelaySec) {
      const int w = fbW, h = fbH;
      std::vector<uint8_t> px(static_cast<size_t>(w) * h * 3);
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      glReadBuffer(GL_BACK);
      glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
      cv::Mat img(h, w, CV_8UC3, px.data());
      cv::flip(img, img, 0);  // OpenGL 原点在左下
      cv::Mat bgr;
      cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
      if (cv::imwrite(shotPath, bgr)) {
        std::printf("[ui] 已保存界面截图: %s\n", shotPath.c_str());
      } else {
        std::fprintf(stderr, "[ui] 截图保存失败: %s\n", shotPath.c_str());
      }
      shotTaken = true;
    }
    if (exitAfterMs > 0 && (glfwGetTime() - startTime) * 1000.0 > exitAfterMs) {
      glfwSetWindowShouldClose(win, 1);
    }

    // ---- 自动化抓图/录制 ----
    if (autoRecordSecs > 0.0 && !g.recorder.recording() && !g.lastCanvas.empty() &&
        ImGui::GetTime() > 0.5) {
      std::string msg;
      g.recorder.start(g.lastCanvas, g.capCfg, &msg);
      showToast(msg);
    }
    // 保护：AVI 单文件上限 2GB，超长录制会写坏文件。
    // 按 20 分钟或 1.8GB 先到者自动停止，并提示用户。
    if (g.recorder.recording()) {
      const bool tooLong = g.recorder.seconds() > 20.0 * 60.0;
      const bool tooBig = g.recorder.writtenBytes() > 1932735283LL;   // 1.8 GB（读真实文件大小）
      if (tooLong || tooBig) {
        const std::string p = g.recorder.stop();
        char buf[512];
        std::snprintf(buf, sizeof(buf), "录制已达上限自动停止（%s）：%d 帧 / %.1f 秒 -> %s",
                      tooBig ? "文件接近 2GB" : "超过 20 分钟", g.recorder.frameCount(),
                      g.recorder.seconds(), p.c_str());
        showToast(buf, 8.0);
        autoRecordSecs = -1.0;
      }
    }
    if (g.recorder.recording() && autoRecordSecs > 0.0 &&
        g.recorder.seconds() >= autoRecordSecs) {
      const std::string p = g.recorder.stop();
      char buf[512];
      std::snprintf(buf, sizeof(buf), "自动录制结束：%d 帧 / %.1f 秒 -> %s",
                    g.recorder.frameCount(), g.recorder.seconds(), p.c_str());
      showToast(buf);
      autoRecordSecs = -1.0;
    }
    if (autoSnapAfter > 0.0 && ImGui::GetTime() >= autoSnapAfter) {
      autoSnapAfter = -1.0;
      doSnapshot();
    }

    // 保存本帧已绘制完的画面：供"截图/录像"按键使用（下一帧生效）
    if (!g.canvas.empty()) {
      g.lastCanvas = g.canvas.clone();
      if (g.recorder.recording()) g.recorder.addFrame(g.lastCanvas);
    }

    glfwSwapBuffers(win);
  }

  shutdownGuard.run();   // 幂等：先收线程与采集，再释放 GL 资源
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
