// ============================================================================
//  src/color_source.cpp  —  数据源工厂
//
//  创建顺序很重要：先创建骨骼后端，再创建数据源。
//  因为 `--source nuitrack` 时彩色底图要从 Nuitrack 后端内部缓存取，
//  避免两个 SDK 抢同一台相机。
// ============================================================================
#include <cstdio>

#include "pose/color_source.hpp"
#include "pose/pose_provider.hpp"

namespace pose {

std::unique_ptr<ColorSource> makeMockColorSource();
std::unique_ptr<ColorSource> makeOpenCvColorSource();
std::unique_ptr<ColorSource> makeOrbbecColorSource();
std::unique_ptr<ColorSource> makeNuitrackColorSource(PoseProvider* provider);

std::unique_ptr<ColorSource> makeColorSource(const AppConfig& cfg, PoseProvider* provider) {
  switch (cfg.source) {
    case AppConfig::SourceKind::Mock:
      return makeMockColorSource();

    case AppConfig::SourceKind::OpenCv:
      return makeOpenCvColorSource();

    case AppConfig::SourceKind::Orbbec:
      if (!orbbecCompiledIn()) {
        std::fprintf(stderr,
                     "[source] 警告：请求 Orbbec SDK 数据源，但编译时未启用 "
                     "-DHAVE_ORBBEC_SDK=ON，已降级为 mock 画面（骨骼后端仍正常）。\n");
        return makeMockColorSource();
      }
      return makeOrbbecColorSource();

    case AppConfig::SourceKind::Nuitrack:
      if (!nuitrackCompiledIn()) {
        std::fprintf(stderr,
                     "[source] 警告：请求 Nuitrack 数据源，但编译时未启用 "
                     "-DHAVE_NUITRACK=ON，已降级为 mock 画面。\n");
        return makeMockColorSource();
      }
      return makeNuitrackColorSource(provider);

    default:
      return makeMockColorSource();
  }
}

}  // namespace pose
