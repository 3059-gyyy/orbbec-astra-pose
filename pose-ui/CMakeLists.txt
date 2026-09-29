cmake_minimum_required(VERSION 3.16)
project(pose_ui LANGUAGES CXX)

# ============================================================================
#  pose_ui —— 图形界面版：相机开关 / 彩色图 / 深度图 / 点云 / 骨骼叠加
#
#  依赖：
#    * OpenCV（core imgproc imgcodecs dnn videoio）—— 取流、着色、姿态推理
#    * GLFW 3.4                                     —— 窗口与输入
#    * Dear ImGui（docking 分支）                    —— 界面（纯色块，无贴图素材）
#    * OpenGL 3.3 core                              —— 点云三维渲染
#
#  依赖路径可用 -D 覆盖：
#    -DOpenCV_DIR=...  -DGLFW_ROOT=...  -DIMGUI_ROOT=...
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(MSVC)
  add_compile_options(/utf-8 /W3 /permissive- /MP)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
#  依赖定位
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn videoio)

set(GLFW_ROOT "" CACHE PATH "GLFW 根目录（含 include/GLFW 与 lib-vc20xx）")
if(GLFW_ROOT STREQUAL "")
  set(GLFW_ROOT "D:/pdf/_deps/glfw/glfw-3.4.bin.WIN64")
endif()
find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h HINTS "${GLFW_ROOT}/include")
find_library(GLFW_LIBRARY NAMES glfw3 glfw glfw3dll
  HINTS "${GLFW_ROOT}/lib-vc2022" "${GLFW_ROOT}/lib-vc2019"
        "${GLFW_ROOT}/lib-vc2015" "${GLFW_ROOT}/lib" "${GLFW_ROOT}/lib/x64")
if(NOT GLFW_INCLUDE_DIR OR NOT GLFW_LIBRARY)
  message(FATAL_ERROR "未找到 GLFW。请用 -DGLFW_ROOT=<解压目录> 指定（需含 include/GLFW 与 lib）")
endif()

set(IMGUI_ROOT "" CACHE PATH "Dear ImGui 根目录（含 imgui.cpp 与 backends/）")
if(IMGUI_ROOT STREQUAL "")
  set(IMGUI_ROOT "D:/pdf/_deps/imgui/imgui-docking")
endif()
foreach(f imgui.cpp imgui_draw.cpp imgui_tables.cpp imgui_widgets.cpp
          backends/imgui_impl_glfw.cpp backends/imgui_impl_opengl3.cpp)
  if(NOT EXISTS "${IMGUI_ROOT}/${f}")
    message(FATAL_ERROR "ImGui 缺少文件: ${IMGUI_ROOT}/${f}（请用 -DIMGUI_ROOT= 指定）")
  endif()
endforeach()

# ---------------------------------------------------------------------------
#  界面工程源码
# ---------------------------------------------------------------------------
set(UI_SOURCES
  src/main_ui.cpp
  src/camera_pipeline.cpp
  src/skeleton_draw.cpp
  src/pointcloud_gl.cpp
  src/gl_loader.cpp
  src/video_device.cpp
  src/capture.cpp
  src/app_config.cpp
  # 复用已验证的姿态模型后端与角度算法（同一份源码，避免两处维护）
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/dnn_pose.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/angles.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/world_coords.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/types.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/pose_provider.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/mock_source.cpp"
  # 这两个文件在未定义 HAVE_NUITRACK / HAVE_ASTRA_SDK 时只提供占位实现，
  # 但骨骼后端工厂需要它们的符号（否则链接报 LNK2019）
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/nuitrack_provider.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/src/orbbec_pose_provider.cpp"
)

add_executable(pose_ui ${UI_SOURCES}
  "${IMGUI_ROOT}/imgui.cpp"
  "${IMGUI_ROOT}/imgui_draw.cpp"
  "${IMGUI_ROOT}/imgui_tables.cpp"
  "${IMGUI_ROOT}/imgui_widgets.cpp"
  "${IMGUI_ROOT}/backends/imgui_impl_glfw.cpp"
  "${IMGUI_ROOT}/backends/imgui_impl_opengl3.cpp"
)

target_include_directories(pose_ui PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pose/include"
  "${IMGUI_ROOT}"
  "${IMGUI_ROOT}/backends"
  "${GLFW_INCLUDE_DIR}"
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(pose_ui PRIVATE
  ${OpenCV_LIBS}
  "${GLFW_LIBRARY}"
  opengl32
)

if(WIN32)
  # GLFW 预编译包提供的是静态库；若换成 dll 版，把 glfw3.dll 放到 exe 同目录
  target_compile_definitions(pose_ui PRIVATE GLFW_STATIC NOMINMAX)
  # strmiids：DirectShow 设备枚举所需的 GUID 库
  # （CLSID_SystemDeviceEnum / CLSID_VideoInputDeviceCategory），
  #  用于给摄像头索引取友好名，多摄像头时供用户选择。
  target_link_libraries(pose_ui PRIVATE strmiids cfgmgr32)
endif()

message(STATUS "---- pose_ui 配置摘要 ----")
message(STATUS "  OpenCV : ${OpenCV_VERSION} (${OpenCV_DIR})")
message(STATUS "  GLFW   : ${GLFW_LIBRARY}")
message(STATUS "  ImGui  : ${IMGUI_ROOT}")
message(STATUS "  构建类型: ${CMAKE_BUILD_TYPE}")
