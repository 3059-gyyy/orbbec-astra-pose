// ============================================================================
//  src/gl_loader.hpp  —  精简的 OpenGL 3.3 加载器
//
//  为什么自己写：
//   Windows SDK 的 GL/gl.h 只到 OpenGL 1.1，而点云渲染需要 FBO / VBO / VAO /
//   着色器（GL 2.0~3.0）。ImGui 后端自带的 imgl3w 加载器只包含它自己需要的
//   函数与常量（缺少 GL_FRAMEBUFFER / GL_DYNAMIC_DRAW 等），因此这里补一个小
//   而完整的加载器：GL 1.1 走 opengl32.lib，1.2+ 走 wglGetProcAddress。
//
//  用法（必须在 GL 上下文创建之后调用一次）：
//      if (!gl::init()) { 报错 }
// ============================================================================
#pragma once

#if defined(_WIN32)
#include <windows.h>
#endif
#include <GL/gl.h>
#include <cstddef>

namespace gl {

// ---------------------------------------------------------------------------
//  补齐 Windows gl.h 缺失的常量（值取自 OpenGL 规范）
// ---------------------------------------------------------------------------
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_ELEMENT_ARRAY_BUFFER
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#endif
#ifndef GL_STATIC_DRAW
#define GL_STATIC_DRAW 0x88E4
#endif
#ifndef GL_DYNAMIC_DRAW
#define GL_DYNAMIC_DRAW 0x88E8
#endif
#ifndef GL_FRAGMENT_SHADER
#define GL_FRAGMENT_SHADER 0x8B30
#endif
#ifndef GL_VERTEX_SHADER
#define GL_VERTEX_SHADER 0x8B31
#endif
#ifndef GL_COMPILE_STATUS
#define GL_COMPILE_STATUS 0x8B81
#endif
#ifndef GL_LINK_STATUS
#define GL_LINK_STATUS 0x8B82
#endif
#ifndef GL_INFO_LOG_LENGTH
#define GL_INFO_LOG_LENGTH 0x8B84
#endif
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif
#ifndef GL_FRAMEBUFFER
#define GL_FRAMEBUFFER 0x8D40
#endif
#ifndef GL_RENDERBUFFER
#define GL_RENDERBUFFER 0x8D41
#endif
#ifndef GL_COLOR_ATTACHMENT0
#define GL_COLOR_ATTACHMENT0 0x8CE0
#endif
#ifndef GL_DEPTH_ATTACHMENT
#define GL_DEPTH_ATTACHMENT 0x8D00
#endif
#ifndef GL_FRAMEBUFFER_COMPLETE
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#endif
#ifndef GL_DEPTH_COMPONENT24
#define GL_DEPTH_COMPONENT24 0x81A6
#endif
#ifndef GL_RGBA8
#define GL_RGBA8 0x8058
#endif
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_BGRA
#define GL_BGRA 0x80E1
#endif
// Windows 的 gl.h 只到 GL 1.1，没有 GL_BGR；这里补齐，避免条件编译分叉。
#ifndef GL_BGR
#define GL_BGR 0x80E0
#endif
#ifndef GL_POINT_SPRITE
#define GL_POINT_SPRITE 0x8861
#endif
#ifndef GL_PROGRAM_POINT_SIZE
#define GL_PROGRAM_POINT_SIZE 0x8642
#endif
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif
#ifndef GL_FUNC_ADD
#define GL_FUNC_ADD 0x8006
#endif

// ---------------------------------------------------------------------------
//  GL 1.2+ 函数指针
// ---------------------------------------------------------------------------
typedef ptrdiff_t GLsizeiptrARB;
typedef ptrdiff_t GLintptrARB;
typedef char GLcharARB;

// 顶点缓冲 / 顶点数组
typedef void(APIENTRY* PFN_glGenBuffers)(GLsizei, GLuint*);
typedef void(APIENTRY* PFN_glBindBuffer)(GLenum, GLuint);
typedef void(APIENTRY* PFN_glBufferData)(GLenum, GLsizeiptrARB, const void*, GLenum);
typedef void(APIENTRY* PFN_glDeleteBuffers)(GLsizei, const GLuint*);
typedef void(APIENTRY* PFN_glGenVertexArrays)(GLsizei, GLuint*);
typedef void(APIENTRY* PFN_glBindVertexArray)(GLuint);
typedef void(APIENTRY* PFN_glDeleteVertexArrays)(GLsizei, const GLuint*);
typedef void(APIENTRY* PFN_glEnableVertexAttribArray)(GLuint);
typedef void(APIENTRY* PFN_glVertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei,
                                                  const void*);

// 着色器
typedef GLuint(APIENTRY* PFN_glCreateShader)(GLenum);
typedef void(APIENTRY* PFN_glShaderSource)(GLuint, GLsizei, const GLcharARB* const*, const GLint*);
typedef void(APIENTRY* PFN_glCompileShader)(GLuint);
typedef void(APIENTRY* PFN_glGetShaderiv)(GLuint, GLenum, GLint*);
typedef void(APIENTRY* PFN_glGetShaderInfoLog)(GLuint, GLsizei, GLsizei*, GLcharARB*);
typedef void(APIENTRY* PFN_glDeleteShader)(GLuint);
typedef GLuint(APIENTRY* PFN_glCreateProgram)(void);
typedef void(APIENTRY* PFN_glAttachShader)(GLuint, GLuint);
typedef void(APIENTRY* PFN_glLinkProgram)(GLuint);
typedef void(APIENTRY* PFN_glGetProgramiv)(GLuint, GLenum, GLint*);
typedef void(APIENTRY* PFN_glUseProgram)(GLuint);
typedef void(APIENTRY* PFN_glDeleteProgram)(GLuint);
typedef GLint(APIENTRY* PFN_glGetUniformLocation)(GLuint, const GLcharARB*);
typedef void(APIENTRY* PFN_glUniformMatrix4fv)(GLint, GLsizei, GLboolean, const GLfloat*);
typedef void(APIENTRY* PFN_glUniform1f)(GLint, GLfloat);

// 帧缓冲
typedef void(APIENTRY* PFN_glGenFramebuffers)(GLsizei, GLuint*);
typedef void(APIENTRY* PFN_glBindFramebuffer)(GLenum, GLuint);
typedef void(APIENTRY* PFN_glDeleteFramebuffers)(GLsizei, const GLuint*);
typedef void(APIENTRY* PFN_glFramebufferTexture2D)(GLenum, GLenum, GLenum, GLuint, GLint);
typedef GLenum(APIENTRY* PFN_glCheckFramebufferStatus)(GLenum);
typedef void(APIENTRY* PFN_glGenRenderbuffers)(GLsizei, GLuint*);
typedef void(APIENTRY* PFN_glBindRenderbuffer)(GLenum, GLuint);
typedef void(APIENTRY* PFN_glRenderbufferStorage)(GLenum, GLenum, GLsizei, GLsizei);
typedef void(APIENTRY* PFN_glDeleteRenderbuffers)(GLsizei, const GLuint*);
typedef void(APIENTRY* PFN_glFramebufferRenderbuffer)(GLenum, GLenum, GLenum, GLuint);

// ---------------------------------------------------------------------------
//  导出的函数指针（定义在 gl_loader.cpp）
// ---------------------------------------------------------------------------
extern PFN_glGenBuffers glGenBuffers;
extern PFN_glBindBuffer glBindBuffer;
extern PFN_glBufferData glBufferData;
extern PFN_glDeleteBuffers glDeleteBuffers;
extern PFN_glGenVertexArrays glGenVertexArrays;
extern PFN_glBindVertexArray glBindVertexArray;
extern PFN_glDeleteVertexArrays glDeleteVertexArrays;
extern PFN_glEnableVertexAttribArray glEnableVertexAttribArray;
extern PFN_glVertexAttribPointer glVertexAttribPointer;

extern PFN_glCreateShader glCreateShader;
extern PFN_glShaderSource glShaderSource;
extern PFN_glCompileShader glCompileShader;
extern PFN_glGetShaderiv glGetShaderiv;
extern PFN_glGetShaderInfoLog glGetShaderInfoLog;
extern PFN_glDeleteShader glDeleteShader;
extern PFN_glCreateProgram glCreateProgram;
extern PFN_glAttachShader glAttachShader;
extern PFN_glLinkProgram glLinkProgram;
extern PFN_glGetProgramiv glGetProgramiv;
extern PFN_glUseProgram glUseProgram;
extern PFN_glDeleteProgram glDeleteProgram;
extern PFN_glGetUniformLocation glGetUniformLocation;
extern PFN_glUniformMatrix4fv glUniformMatrix4fv;
extern PFN_glUniform1f glUniform1f;

extern PFN_glGenFramebuffers glGenFramebuffers;
extern PFN_glBindFramebuffer glBindFramebuffer;
extern PFN_glDeleteFramebuffers glDeleteFramebuffers;
extern PFN_glFramebufferTexture2D glFramebufferTexture2D;
extern PFN_glCheckFramebufferStatus glCheckFramebufferStatus;
extern PFN_glGenRenderbuffers glGenRenderbuffers;
extern PFN_glBindRenderbuffer glBindRenderbuffer;
extern PFN_glRenderbufferStorage glRenderbufferStorage;
extern PFN_glDeleteRenderbuffers glDeleteRenderbuffers;
extern PFN_glFramebufferRenderbuffer glFramebufferRenderbuffer;

// 初始化：加载所有函数指针。要求在 GL 上下文已 current 的情况下调用。
// 返回 false 表示当前驱动缺少必需函数（一般是没拿到 3.3 core 上下文）。
bool init();

// 初始化失败时返回缺失的函数名列表（便于排查）
const char* missingFunctions();

}  // namespace gl
