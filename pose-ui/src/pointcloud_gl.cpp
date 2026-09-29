// ============================================================================
//  src/pointcloud_gl.cpp  —  点云渲染实现
//
//  OpenGL 函数指针来自工程自带的 gl_loader（见 gl_loader.hpp）：
//  GL 1.1 走 opengl32.lib，1.2+ 走 wglGetProcAddress。
// ============================================================================
#include "pointcloud_gl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "gl_loader.hpp"

namespace ui {
namespace {

// GL 函数指针与常量都在 gl:: 命名空间
using namespace gl;

const char* kVertSrc = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMVP;
uniform float uPointSize;
out vec3 vColor;
void main() {
  vColor = aColor;
  gl_Position = uMVP * vec4(aPos, 1.0);
  gl_PointSize = uPointSize;
}
)";

const char* kFragSrc = R"(#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() { FragColor = vec4(vColor, 1.0); }
)";

unsigned int compileShader(unsigned int type, const char* src) {
  const unsigned int s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  int ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[1024] = {0};
    glGetShaderInfoLog(s, sizeof(log) - 1, nullptr, log);
    std::fprintf(stderr, "[gl] 着色器编译失败: %s\n", log);
    glDeleteShader(s);
    return 0;
  }
  return s;
}

// 4x4 矩阵（列主序，直接喂给 OpenGL）
struct Mat4 {
  float m[16];
};

Mat4 identity() {
  Mat4 r{};
  r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.f;
  return r;
}

Mat4 multiply(const Mat4& a, const Mat4& b) {
  Mat4 r{};
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      float sum = 0.f;
      for (int k = 0; k < 4; ++k) sum += a.m[k * 4 + row] * b.m[c * 4 + k];
      r.m[c * 4 + row] = sum;
    }
  }
  return r;
}

Mat4 perspective(float fovDeg, float aspect, float zn, float zf) {
  Mat4 r{};
  const float f = 1.f / std::tan(fovDeg * 3.14159265f / 360.f);
  r.m[0] = f / aspect;
  r.m[5] = f;
  r.m[10] = (zf + zn) / (zn - zf);
  r.m[11] = -1.f;
  r.m[14] = (2.f * zf * zn) / (zn - zf);
  return r;
}

Mat4 lookAt(float ex, float ey, float ez, float cx, float cy, float cz) {
  float fx = cx - ex, fy = cy - ey, fz = cz - ez;
  float fl = std::sqrt(fx * fx + fy * fy + fz * fz);
  if (fl < 1e-6f) fl = 1.f;
  fx /= fl; fy /= fl; fz /= fl;

  // up = (0,1,0)，s = f x up
  float sx = fy * 0.f - fz * 1.f;
  float sy = fz * 0.f - fx * 0.f;
  float sz = fx * 1.f - fy * 0.f;
  float sl = std::sqrt(sx * sx + sy * sy + sz * sz);
  if (sl < 1e-6f) { sx = 1.f; sy = 0.f; sz = 0.f; sl = 1.f; }
  sx /= sl; sy /= sl; sz /= sl;

  const float ux = sy * fz - sz * fy;
  const float uy = sz * fx - sx * fz;
  const float uz = sx * fy - sy * fx;

  Mat4 r = identity();
  r.m[0] = sx; r.m[4] = sy; r.m[8] = sz;
  r.m[1] = ux; r.m[5] = uy; r.m[9] = uz;
  r.m[2] = -fx; r.m[6] = -fy; r.m[10] = -fz;
  r.m[12] = -(sx * ex + sy * ey + sz * ez);
  r.m[13] = -(ux * ex + uy * ey + uz * ez);
  r.m[14] = (fx * ex + fy * ey + fz * ez);
  return r;
}

}  // namespace

PointCloudRenderer::~PointCloudRenderer() { shutdown(); }

bool PointCloudRenderer::init() {
  if (ready_) return true;
  if (glCreateShader == nullptr || glGenVertexArrays == nullptr) {
    err_ = "OpenGL 函数指针未加载（需要 OpenGL 3.3 上下文）";
    return false;
  }

  const unsigned int vs = compileShader(GL_VERTEX_SHADER, kVertSrc);
  const unsigned int fs = compileShader(GL_FRAGMENT_SHADER, kFragSrc);
  if (!vs || !fs) {
    err_ = "着色器编译失败";
    return false;
  }
  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);
  int linked = 0;
  glGetProgramiv(program_, GL_LINK_STATUS, &linked);
  glDeleteShader(vs);
  glDeleteShader(fs);
  if (!linked) {
    err_ = "着色器链接失败";
    return false;
  }

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vboPos_);
  glGenBuffers(1, &vboCol_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vboPos_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, vboCol_);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, 3 * sizeof(uint8_t), (void*)0);
  glBindVertexArray(0);

  ready_ = true;
  return true;
}

void PointCloudRenderer::shutdown() {
  destroyTarget();
  if (vboPos_) { glDeleteBuffers(1, &vboPos_); vboPos_ = 0; }
  if (vboCol_) { glDeleteBuffers(1, &vboCol_); vboCol_ = 0; }
  if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
  if (program_) { glDeleteProgram(program_); program_ = 0; }
  ready_ = false;
}

void PointCloudRenderer::upload(const std::vector<float>& positions,
                                const std::vector<uint8_t>& colors, float pointSize) {
  if (!ready_) return;
  pointCount_ = static_cast<int>(positions.size() / 3);
  pointSize_ = pointSize > 0.f ? pointSize : 2.f;
  if (pointCount_ == 0) return;

  glBindBuffer(GL_ARRAY_BUFFER, vboPos_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptrARB>(positions.size() * sizeof(float)),
               positions.data(), GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, vboCol_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptrARB>(colors.size() * sizeof(uint8_t)), colors.data(),
               GL_DYNAMIC_DRAW);
}

bool PointCloudRenderer::ensureTarget(int width, int height) {
  if (width <= 0 || height <= 0) return false;
  if (fbo_ && texW_ == width && texH_ == height) return true;
  destroyTarget();

  glGenTextures(1, &tex_);
  glBindTexture(GL_TEXTURE_2D, tex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glGenRenderbuffers(1, &depthRb_);
  glBindRenderbuffer(GL_RENDERBUFFER, depthRb_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);

  glGenFramebuffers(1, &fbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex_, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRb_);

  const unsigned int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    err_ = "离屏帧缓冲创建失败";
    destroyTarget();
    return false;
  }
  texW_ = width;
  texH_ = height;
  return true;
}

void PointCloudRenderer::destroyTarget() {
  if (fbo_) { glDeleteFramebuffers(1, &fbo_); fbo_ = 0; }
  if (depthRb_) { glDeleteRenderbuffers(1, &depthRb_); depthRb_ = 0; }
  if (tex_) { glDeleteTextures(1, &tex_); tex_ = 0; }
  texW_ = texH_ = 0;
}

unsigned int PointCloudRenderer::render(int width, int height, const PointCloudView& view) {
  if (!ready_ || !ensureTarget(width, height)) return 0;

  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  glViewport(0, 0, width, height);
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.055f, 0.06f, 0.075f, 1.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (pointCount_ > 0) {
    // 相机绕点云中心环绕（点云以相机为原点，z 朝前）
    const float yaw = view.yawDeg * 3.14159265f / 180.f;
    const float pitch = view.pitchDeg * 3.14159265f / 180.f;
    const float d = std::max(0.3f, view.distM);
    const float eyeX = d * std::cos(pitch) * std::sin(yaw) + view.panX;
    const float eyeY = -d * std::sin(pitch) + view.panY;
    const float eyeZ = -d * std::cos(pitch) * std::cos(yaw);

    const Mat4 proj = perspective(view.fovDeg, static_cast<float>(width) / height, 0.05f, 60.f);
    const Mat4 mview = lookAt(eyeX, eyeY, eyeZ, view.panX, view.panY, 0.f);
    const Mat4 mvp = multiply(proj, mview);

    glUseProgram(program_);
    const int loc = glGetUniformLocation(program_, "uMVP");
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, mvp.m);
    const int locPs = glGetUniformLocation(program_, "uPointSize");
    if (locPs >= 0) glUniform1f(locPs, pointSize_);

    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, pointCount_);
    glBindVertexArray(0);
    glUseProgram(0);
  }

  glDisable(GL_DEPTH_TEST);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  return tex_;
}

// ---------------------------------------------------------------------------
//  CPU 端点云构建：深度 + 内参 -> 三维点
// ---------------------------------------------------------------------------
int buildPointCloud(const std::vector<uint16_t>& depthMm, int depthW, int depthH, double fx,
                    double fy, double icx, double icy, int stride, int minMm, int maxMm,
                    const std::vector<uint8_t>* colorBgr, int colorW, int colorH,
                    bool useColor, std::vector<float>& positions,
                    std::vector<uint8_t>& colors) {
  positions.clear();
  colors.clear();
  if (depthW <= 0 || depthH <= 0 || fx < 1.0 || fy < 1.0) return 0;
  if (depthMm.size() < static_cast<size_t>(depthW) * depthH) return 0;

  const int st = std::max(1, stride);
  const int est = (depthW / st + 1) * (depthH / st + 1);
  positions.reserve(static_cast<size_t>(est) * 3);
  colors.reserve(static_cast<size_t>(est) * 3);

  const float sx = colorW > 0 ? static_cast<float>(colorW) / depthW : 1.f;
  const float sy = colorH > 0 ? static_cast<float>(colorH) / depthH : 1.f;

  for (int v = 0; v < depthH; v += st) {
    const uint16_t* row = depthMm.data() + static_cast<size_t>(v) * depthW;
    for (int u = 0; u < depthW; u += st) {
      const int dmm = row[u];
      if (dmm <= minMm || dmm >= maxMm) continue;

      const float z = dmm / 1000.f;                              // mm -> m
      const float x = static_cast<float>((u - icx) * z / fx);
      const float y = static_cast<float>(-(v - icy) * z / fy);   // 图像 y 向下 -> 世界 y 向上

      positions.push_back(x);
      positions.push_back(y);
      positions.push_back(z);

      if (useColor && colorBgr && colorW > 0 && colorH > 0) {
        const int cu = std::min(colorW - 1, std::max(0, static_cast<int>(u * sx)));
        const int cv = std::min(colorH - 1, std::max(0, static_cast<int>(v * sy)));
        const size_t idx = (static_cast<size_t>(cv) * colorW + cu) * 3;
        if (idx + 2 < colorBgr->size()) {
          colors.push_back((*colorBgr)[idx + 2]);  // R
          colors.push_back((*colorBgr)[idx + 1]);  // G
          colors.push_back((*colorBgr)[idx + 0]);  // B
        } else {
          colors.push_back(200); colors.push_back(200); colors.push_back(200);
        }
      } else {
        // 按深度着色（近暖远冷）
        const float t = std::min(1.f, std::max(0.f, static_cast<float>(dmm - minMm) /
                                                          std::max(1, maxMm - minMm)));
        colors.push_back(static_cast<uint8_t>(255 * std::min(1.f, t * 2.f)));
        colors.push_back(static_cast<uint8_t>(255 * (1.f - std::fabs(t - 0.5f) * 2.f)));
        colors.push_back(static_cast<uint8_t>(255 * std::min(1.f, (1.f - t) * 2.f)));
      }
    }
  }
  return static_cast<int>(positions.size() / 3);
}

}  // namespace ui
