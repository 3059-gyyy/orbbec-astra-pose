// ============================================================================
//  pose/types.cpp  —  关节/角度名称表、骨骼连线表、枚举字符串解析
// ============================================================================
#include "pose/types.hpp"

#include <algorithm>
#include <cctype>

#include "pose/color_source.hpp"
#include "pose/pose_provider.hpp"

namespace pose {

const char* jointName(JointId id) {
  switch (id) {
    case JointId::Head: return "Head";
    case JointId::Neck: return "Neck";
    case JointId::ShoulderL: return "ShoulderL";
    case JointId::ElbowL: return "ElbowL";
    case JointId::WristL: return "WristL";
    case JointId::ShoulderR: return "ShoulderR";
    case JointId::ElbowR: return "ElbowR";
    case JointId::WristR: return "WristR";
    case JointId::Spine: return "Spine";
    case JointId::HipL: return "HipL";
    case JointId::KneeL: return "KneeL";
    case JointId::AnkleL: return "AnkleL";
    case JointId::HipR: return "HipR";
    case JointId::KneeR: return "KneeR";
    case JointId::AnkleR: return "AnkleR";
    case JointId::FootL: return "FootL";
    case JointId::FootR: return "FootR";
    default: return "Unknown";
  }
}

const char* jointNameZh(JointId id) {
  switch (id) {
    case JointId::Head: return "头";
    case JointId::Neck: return "颈";
    case JointId::ShoulderL: return "左肩";
    case JointId::ElbowL: return "左肘";
    case JointId::WristL: return "左腕";
    case JointId::ShoulderR: return "右肩";
    case JointId::ElbowR: return "右肘";
    case JointId::WristR: return "右腕";
    case JointId::Spine: return "脊柱";
    case JointId::HipL: return "左髋";
    case JointId::KneeL: return "左膝";
    case JointId::AnkleL: return "左踝";
    case JointId::HipR: return "右髋";
    case JointId::KneeR: return "右膝";
    case JointId::AnkleR: return "右踝";
    case JointId::FootL: return "左脚";
    case JointId::FootR: return "右脚";
    default: return "未知";
  }
}

// 17 条连线，覆盖头-躯干-四肢。顺序不影响绘制结果。
const std::array<Bone, 17> kBones = {{
    {static_cast<int>(JointId::Head), static_cast<int>(JointId::Neck), false},
    {static_cast<int>(JointId::Neck), static_cast<int>(JointId::Spine), false},
    {static_cast<int>(JointId::Spine), static_cast<int>(JointId::HipL), false},
    {static_cast<int>(JointId::Spine), static_cast<int>(JointId::HipR), false},
    {static_cast<int>(JointId::HipL), static_cast<int>(JointId::HipR), false},
    {static_cast<int>(JointId::Neck), static_cast<int>(JointId::ShoulderL), true},
    {static_cast<int>(JointId::ShoulderL), static_cast<int>(JointId::ElbowL), true},
    {static_cast<int>(JointId::ElbowL), static_cast<int>(JointId::WristL), true},
    {static_cast<int>(JointId::Neck), static_cast<int>(JointId::ShoulderR), false},
    {static_cast<int>(JointId::ShoulderR), static_cast<int>(JointId::ElbowR), false},
    {static_cast<int>(JointId::ElbowR), static_cast<int>(JointId::WristR), false},
    {static_cast<int>(JointId::HipL), static_cast<int>(JointId::KneeL), true},
    {static_cast<int>(JointId::KneeL), static_cast<int>(JointId::AnkleL), true},
    {static_cast<int>(JointId::AnkleL), static_cast<int>(JointId::FootL), true},
    {static_cast<int>(JointId::HipR), static_cast<int>(JointId::KneeR), false},
    {static_cast<int>(JointId::KneeR), static_cast<int>(JointId::AnkleR), false},
    {static_cast<int>(JointId::AnkleR), static_cast<int>(JointId::FootR), false},
}};

const char* angleName(AngleId id) {
  switch (id) {
    case AngleId::ElbowL: return "ElbowL";
    case AngleId::ElbowR: return "ElbowR";
    case AngleId::ShoulderL: return "ShoulderL";
    case AngleId::ShoulderR: return "ShoulderR";
    case AngleId::HipL: return "HipL";
    case AngleId::HipR: return "HipR";
    case AngleId::KneeL: return "KneeL";
    case AngleId::KneeR: return "KneeR";
    case AngleId::Neck: return "Neck";
    case AngleId::Torso: return "Torso";
    default: return "Unknown";
  }
}

const char* angleNameZh(AngleId id) {
  switch (id) {
    case AngleId::ElbowL: return "左肘";
    case AngleId::ElbowR: return "右肘";
    case AngleId::ShoulderL: return "左肩";
    case AngleId::ShoulderR: return "右肩";
    case AngleId::HipL: return "左髋";
    case AngleId::HipR: return "右髋";
    case AngleId::KneeL: return "左膝";
    case AngleId::KneeR: return "右膝";
    case AngleId::Neck: return "颈部";
    case AngleId::Torso: return "躯干";
    default: return "未知";
  }
}

static std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

const char* sourceKindName(AppConfig::SourceKind k) {
  switch (k) {
    case AppConfig::SourceKind::Mock: return "mock";
    case AppConfig::SourceKind::OpenCv: return "opencv";
    case AppConfig::SourceKind::Nuitrack: return "nuitrack";
    case AppConfig::SourceKind::Orbbec: return "orbbec";
    default: return "unknown";
  }
}

bool parseSourceKind(const std::string& s, AppConfig::SourceKind& out) {
  const std::string v = lower(s);
  if (v == "mock") { out = AppConfig::SourceKind::Mock; return true; }
  if (v == "opencv" || v == "cv" || v == "camera" || v == "video") {
    out = AppConfig::SourceKind::OpenCv;
    return true;
  }
  if (v == "nuitrack") { out = AppConfig::SourceKind::Nuitrack; return true; }
  if (v == "orbbec" || v == "astra" || v == "astra+") {
    out = AppConfig::SourceKind::Orbbec;
    return true;
  }
  return false;
}

// 注意：poseBackendName() / parsePoseBackend() 定义在 pose_provider.cpp，
// 此处不再重复定义（否则会 link 报重复符号）。

}  // namespace pose
