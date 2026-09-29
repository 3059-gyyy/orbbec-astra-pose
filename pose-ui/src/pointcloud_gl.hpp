// ============================================================================
//  src/pointcloud_gl.hpp  —  点云三维渲染（OpenGL 3.3 core，离屏 FBO -> 纹理）
//
//  数据来源：深度图 + 相机内参。每个像素反投影成相机坐标系下的三维点：
//      Z = depth(mm) / 1000
//      X = (u - cx) * Z / fx
//      Y = (v - cy) * Z / fy          （Y 轴向上，便于观察）
//  颜色：可用彩色图着色，或按深度着色。
//
//  轨迹球交互（鼠标左键旋转 / 滚轮缩放）由外部传入 yaw/pitch/dist。
// ============================================================================
#pragma once

#include <cstdint>
#include <vector>

namespace ui {

struct PointCloudView {
  float yawDeg = 0.f;
  float pitchDeg = -12.f;
  float distM = 2.6f;
  float panX = 0.f;
  float panY = 0.f;
  float fovDeg = 45.f;
  bool showGrid = true;
  bool showAxes = true;
};

// 保持尺寸与视口一致的离屏渲染器
class PointCloudRenderer {
 public:
  ~PointCloudRenderer();

  bool init();
  void shutdown();

  // 设置点的位置/颜色缓冲（只在数据变化时调用，避免每帧上传）
  // positions: xyz 交错，单位米；colors: rgb 交错，0..255
  void upload(const std::vector<float>& positions, const std::vector<uint8_t>& colors,
              float pointSize);

  // 渲染到离屏纹理，返回纹理 id（0 表示失败）。宽高变化时会重建 FBO。
  unsigned int render(int width, int height, const PointCloudView& view);

  int pointCount() const { return pointCount_; }
  const char* lastError() const { return err_; }

 private:
  bool ensureTarget(int width, int height);
  void destroyTarget();

  unsigned int program_ = 0;
  unsigned int vao_ = 0;
  unsigned int vboPos_ = 0;
  unsigned int vboCol_ = 0;
  unsigned int fbo_ = 0;
  unsigned int tex_ = 0;
  unsigned int depthRb_ = 0;
  int texW_ = 0;
  int texH_ = 0;
  int pointCount_ = 0;
  float pointSize_ = 2.f;
  bool ready_ = false;
  const char* err_ = "";
};

// 深度 + 内参 -> 点云缓冲（CPU 端，含抽样与深度范围过滤）
// 返回点数量。positions/colors 会被 resize。
int buildPointCloud(const std::vector<uint16_t>& depthMm, int depthW, int depthH,
                    double fx, double fy, double icx, double icy, int stride,
                    int minMm, int maxMm, const std::vector<uint8_t>* colorBgr, int colorW,
                    int colorH, bool useColor, std::vector<float>& positions,
                    std::vector<uint8_t>& colors);

}  // namespace ui
