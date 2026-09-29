// ============================================================================
//  src/pose_provider.cpp  —  骨骼后端工厂
// ============================================================================
#include <cstdio>

#include "pose/pose_provider.hpp"

namespace pose {

std::unique_ptr<PoseProvider> makeMockPoseProvider();
std::unique_ptr<PoseProvider> makeNuitrackPoseProvider();
std::unique_ptr<PoseProvider> makeOrbbecAstraPoseProvider();
std::unique_ptr<PoseProvider> makeDnnPoseProvider();

// 注意：nuitrackCompiledIn() / astraCompiledIn() 分别定义在各自的实现文件里，
// 此处不再重复定义。

std::unique_ptr<PoseProvider> makePoseProvider(const AppConfig& cfg) {
  return makePoseProviderWithBackend(cfg, cfg.poseBackend);
}

std::unique_ptr<PoseProvider> makePoseProviderWithBackend(const AppConfig& cfg,
                                                          AppConfig::PoseBackend backend) {
  switch (backend) {
    case AppConfig::PoseBackend::Dnn:
      return makeDnnPoseProvider();
    case AppConfig::PoseBackend::Nuitrack: {
      if (!nuitrackCompiledIn()) {
        std::fprintf(stderr,
                     "[pose] 警告：请求 Nuitrack 后端，但本程序编译时未启用 Nuitrack，"
                     "已自动降级为 mock 骨骼（叠加与角度逻辑仍可验证）。\n");
        return makeMockPoseProvider();
      }
      return makeNuitrackPoseProvider();
    }
    case AppConfig::PoseBackend::OrbbecAstra: {
      if (!astraCompiledIn()) {
        std::fprintf(stderr,
                     "[pose] 警告：请求奥比中光 Body Tracking 后端，但编译时未启用 "
                     "Astra SDK（-DHAVE_ASTRA_SDK=ON），已自动降级为 mock 骨骼。\n");
        return makeMockPoseProvider();
      }
      return makeOrbbecAstraPoseProvider();
    }
    case AppConfig::PoseBackend::Mock:
    default:
      return makeMockPoseProvider();
  }
}

const char* poseBackendName(AppConfig::PoseBackend b) {
  switch (b) {
    case AppConfig::PoseBackend::Dnn: return "dnn";
    case AppConfig::PoseBackend::Nuitrack: return "nuitrack";
    case AppConfig::PoseBackend::OrbbecAstra: return "orbbec-astra";
    case AppConfig::PoseBackend::Mock: return "mock";
    default: return "unknown";
  }
}

bool parsePoseBackend(const std::string& s, AppConfig::PoseBackend& out) {
  std::string v;
  v.reserve(s.size());
  for (char c : s) v.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  if (v == "dnn" || v == "onnx" || v == "yolo" || v == "yolov8" || v == "model") {
    out = AppConfig::PoseBackend::Dnn;
    return true;
  }
  if (v == "nuitrack") {
    out = AppConfig::PoseBackend::Nuitrack;
    return true;
  }
  if (v == "orbbec-astra" || v == "orbbec" || v == "astra" || v == "bodytracking" ||
      v == "body-tracking") {
    out = AppConfig::PoseBackend::OrbbecAstra;
    return true;
  }
  if (v == "mock" || v == "synthetic") {
    out = AppConfig::PoseBackend::Mock;
    return true;
  }
  return false;
}

}  // namespace pose
