// ============================================================================
//  src/app_config.cpp  —  config.json 读取 + 控制台窗口可见性
// ============================================================================
#include "app_config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

#if defined(_WIN32)
#include <windows.h>
#endif

namespace ui {
namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// ---- 极简 JSON 取值：在文本里找 "key"，再解析其后的值 ----------------------
// 只支持扁平的 {"k": v, ...}，足够配置文件使用；解析失败即回退默认值。
bool findValue(const std::string& text, const std::string& key, std::string* rawOut) {
  const std::string needle = "\"" + key + "\"";
  size_t pos = text.find(needle);
  while (pos != std::string::npos) {
    // 跳过 key 与其后的空白，找冒号
    size_t colon = text.find(':', pos + needle.size());
    if (colon == std::string::npos) return false;
    // 确认冒号与 key 之间只有空白
    bool ok = true;
    for (size_t i = pos + needle.size(); i < colon; ++i) {
      if (!std::isspace(static_cast<unsigned char>(text[i]))) { ok = false; break; }
    }
    if (!ok) { pos = text.find(needle, pos + 1); continue; }

    size_t i = colon + 1;
    while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
    if (i >= text.size()) return false;

    if (text[i] == '"') {                       // 字符串
      const size_t end = text.find('"', i + 1);
      if (end == std::string::npos) return false;
      *rawOut = text.substr(i + 1, end - i - 1);
      return true;
    }
    // 数字 / true / false / null：取到逗号或右花括号为止
    size_t end = i;
    while (end < text.size() && text[end] != ',' && text[end] != '}' &&
           text[end] != '\n' && text[end] != '\r') {
      ++end;
    }
    std::string v = text.substr(i, end - i);
    while (!v.empty() && std::isspace(static_cast<unsigned char>(v.back()))) v.pop_back();
    *rawOut = v;
    return true;
  }
  return false;
}

bool toBool(const std::string& v, bool dflt) {
  const std::string s = lower(v);
  if (s == "true" || s == "1" || s == "yes" || s == "on") return true;
  if (s == "false" || s == "0" || s == "no" || s == "off") return false;
  return dflt;
}

int toInt(const std::string& v, int dflt) {
  try {
    return std::stoi(v);
  } catch (...) {
    return dflt;
  }
}

// 去掉 // 与 /* */ 注释，便于在配置里写说明
std::string stripComments(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    if (in[i] == '/' && i + 1 < in.size() && in[i + 1] == '/') {
      while (i < in.size() && in[i] != '\n') ++i;
      out.push_back('\n');
    } else if (in[i] == '/' && i + 1 < in.size() && in[i + 1] == '*') {
      i += 2;
      while (i + 1 < in.size() && !(in[i] == '*' && in[i + 1] == '/')) ++i;
      ++i;
    } else {
      out.push_back(in[i]);
    }
  }
  return out;
}

}  // namespace

AppConfigFile loadAppConfigFile(const std::string& path) {
  AppConfigFile cfg;

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    cfg.status = "未找到 " + path + "，使用内置默认值（命令行窗口隐藏）";
    return cfg;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  std::string raw = ss.str();

  // 去掉 UTF-8 BOM：带 BOM 的 config.json（记事本 / PowerShell 保存常见）
  // 会让首个键名变成 "\xEF\xBB\xBF\"developerMode\""，导致所有配置都读不到。
  if (raw.size() >= 3 && static_cast<unsigned char>(raw[0]) == 0xEF &&
      static_cast<unsigned char>(raw[1]) == 0xBB &&
      static_cast<unsigned char>(raw[2]) == 0xBF) {
    raw.erase(0, 3);
  }

  const std::string text = stripComments(raw);
  if (text.find('{') == std::string::npos) {
    cfg.status = path + " 内容不像 JSON，使用内置默认值";
    return cfg;
  }

  std::string v;
  if (findValue(text, "developerMode", &v)) cfg.developerMode = toBool(v, cfg.developerMode);
  if (findValue(text, "logToFile", &v)) cfg.logToFile = toBool(v, cfg.logToFile);
  if (findValue(text, "cameraIndex", &v)) cfg.cameraIndex = toInt(v, cfg.cameraIndex);
  if (findValue(text, "view", &v)) cfg.view = lower(v);
  if (findValue(text, "mirror", &v)) cfg.mirror = toBool(v, cfg.mirror);
  if (findValue(text, "depthStream", &v)) cfg.depthStream = toBool(v, cfg.depthStream);
  if (findValue(text, "poseEnabled", &v)) cfg.poseEnabled = toBool(v, cfg.poseEnabled);
  if (findValue(text, "poseModel", &v)) cfg.poseModel = v;
  if (findValue(text, "captureBackend", &v)) cfg.captureBackend = v;
  if (findValue(text, "captureDir", &v)) cfg.captureDir = v;
  if (findValue(text, "videoFps", &v)) {
    try { cfg.videoFps = std::stod(v); } catch (...) { }
  }

  cfg.status = "已读取 " + path;
  return cfg;
}

void applyConsoleVisibility(bool wantVisible) {
#if defined(_WIN32)
  HWND console = GetConsoleWindow();
  if (wantVisible) {
    // 保留命令行窗口：若被父进程隐藏则恢复显示，并把它带到前台
    if (console) {
      ShowWindow(console, SW_SHOW);
      SetWindowPos(console, HWND_TOP, 0, 0, 0, 0,
                   SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
    }
    return;
  }
  // 隐藏：把控制台从当前进程分离，窗口消失但进程照常运行
  if (console) {
    FreeConsole();
  }
#else
  (void)wantVisible;
#endif
}

}  // namespace ui
