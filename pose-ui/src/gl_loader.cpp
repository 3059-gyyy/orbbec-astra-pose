// ============================================================================
//  src/gl_loader.cpp  —  OpenGL 3.3 函数指针加载实现
// ============================================================================
#include "gl_loader.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace gl {

PFN_glGenBuffers glGenBuffers = nullptr;
PFN_glBindBuffer glBindBuffer = nullptr;
PFN_glBufferData glBufferData = nullptr;
PFN_glDeleteBuffers glDeleteBuffers = nullptr;
PFN_glGenVertexArrays glGenVertexArrays = nullptr;
PFN_glBindVertexArray glBindVertexArray = nullptr;
PFN_glDeleteVertexArrays glDeleteVertexArrays = nullptr;
PFN_glEnableVertexAttribArray glEnableVertexAttribArray = nullptr;
PFN_glVertexAttribPointer glVertexAttribPointer = nullptr;

PFN_glCreateShader glCreateShader = nullptr;
PFN_glShaderSource glShaderSource = nullptr;
PFN_glCompileShader glCompileShader = nullptr;
PFN_glGetShaderiv glGetShaderiv = nullptr;
PFN_glGetShaderInfoLog glGetShaderInfoLog = nullptr;
PFN_glDeleteShader glDeleteShader = nullptr;
PFN_glCreateProgram glCreateProgram = nullptr;
PFN_glAttachShader glAttachShader = nullptr;
PFN_glLinkProgram glLinkProgram = nullptr;
PFN_glGetProgramiv glGetProgramiv = nullptr;
PFN_glUseProgram glUseProgram = nullptr;
PFN_glDeleteProgram glDeleteProgram = nullptr;
PFN_glGetUniformLocation glGetUniformLocation = nullptr;
PFN_glUniformMatrix4fv glUniformMatrix4fv = nullptr;
PFN_glUniform1f glUniform1f = nullptr;

PFN_glGenFramebuffers glGenFramebuffers = nullptr;
PFN_glBindFramebuffer glBindFramebuffer = nullptr;
PFN_glDeleteFramebuffers glDeleteFramebuffers = nullptr;
PFN_glFramebufferTexture2D glFramebufferTexture2D = nullptr;
PFN_glCheckFramebufferStatus glCheckFramebufferStatus = nullptr;
PFN_glGenRenderbuffers glGenRenderbuffers = nullptr;
PFN_glBindRenderbuffer glBindRenderbuffer = nullptr;
PFN_glRenderbufferStorage glRenderbufferStorage = nullptr;
PFN_glDeleteRenderbuffers glDeleteRenderbuffers = nullptr;
PFN_glFramebufferRenderbuffer glFramebufferRenderbuffer = nullptr;

namespace {

std::string g_missing;

// 先查 wglGetProcAddress（1.2+ 扩展/核心函数），失败再查 opengl32.dll 导出表
void* getProc(const char* name) {
  void* p = nullptr;
#if defined(_WIN32)
  p = reinterpret_cast<void*>(wglGetProcAddress(name));
  if (p == nullptr || p == reinterpret_cast<void*>(1) ||
      p == reinterpret_cast<void*>(2) || p == reinterpret_cast<void*>(3) ||
      p == reinterpret_cast<void*>(-1)) {
    static HMODULE gl = LoadLibraryA("opengl32.dll");
    if (gl) p = reinterpret_cast<void*>(GetProcAddress(gl, name));
  }
#else
  (void)name;
#endif
  return p;
}

template <typename T>
void load(T& fn, const char* name) {
  fn = reinterpret_cast<T>(getProc(name));
  if (!fn) {
    if (!g_missing.empty()) g_missing += ", ";
    g_missing += name;
  }
}

}  // namespace

bool init() {
  g_missing.clear();

  load(glGenBuffers, "glGenBuffers");
  load(glBindBuffer, "glBindBuffer");
  load(glBufferData, "glBufferData");
  load(glDeleteBuffers, "glDeleteBuffers");
  load(glGenVertexArrays, "glGenVertexArrays");
  load(glBindVertexArray, "glBindVertexArray");
  load(glDeleteVertexArrays, "glDeleteVertexArrays");
  load(glEnableVertexAttribArray, "glEnableVertexAttribArray");
  load(glVertexAttribPointer, "glVertexAttribPointer");

  load(glCreateShader, "glCreateShader");
  load(glShaderSource, "glShaderSource");
  load(glCompileShader, "glCompileShader");
  load(glGetShaderiv, "glGetShaderiv");
  load(glGetShaderInfoLog, "glGetShaderInfoLog");
  load(glDeleteShader, "glDeleteShader");
  load(glCreateProgram, "glCreateProgram");
  load(glAttachShader, "glAttachShader");
  load(glLinkProgram, "glLinkProgram");
  load(glGetProgramiv, "glGetProgramiv");
  load(glUseProgram, "glUseProgram");
  load(glDeleteProgram, "glDeleteProgram");
  load(glGetUniformLocation, "glGetUniformLocation");
  load(glUniformMatrix4fv, "glUniformMatrix4fv");
  load(glUniform1f, "glUniform1f");

  load(glGenFramebuffers, "glGenFramebuffers");
  load(glBindFramebuffer, "glBindFramebuffer");
  load(glDeleteFramebuffers, "glDeleteFramebuffers");
  load(glFramebufferTexture2D, "glFramebufferTexture2D");
  load(glCheckFramebufferStatus, "glCheckFramebufferStatus");
  load(glGenRenderbuffers, "glGenRenderbuffers");
  load(glBindRenderbuffer, "glBindRenderbuffer");
  load(glRenderbufferStorage, "glRenderbufferStorage");
  load(glDeleteRenderbuffers, "glDeleteRenderbuffers");
  load(glFramebufferRenderbuffer, "glFramebufferRenderbuffer");

  if (!g_missing.empty()) {
    std::fprintf(stderr, "[gl] 以下 OpenGL 函数加载失败: %s\n", g_missing.c_str());
    return false;
  }
  return true;
}

const char* missingFunctions() { return g_missing.c_str(); }

}  // namespace gl
