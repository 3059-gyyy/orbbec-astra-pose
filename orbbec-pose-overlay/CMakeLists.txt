cmake_minimum_required(VERSION 3.16)
project(pose_overlay LANGUAGES CXX)

# ---------------------------------------------------------------------------
#  基础设置
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(MSVC)
  add_compile_options(/utf-8 /W4 /permissive-)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
#  依赖 1：OpenCV（必需——负责取图、绘制、窗口显示）
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio dnn)

# ---------------------------------------------------------------------------
#  依赖 2/3：Nuitrack（骨骼）与 Orbbec SDK v2（取流），均可选
# ---------------------------------------------------------------------------
option(HAVE_NUITRACK "启用 Nuitrack 骨骼后端" OFF)
option(HAVE_ORBBEC_SDK "启用 Orbbec SDK v2 彩色/深度取流" OFF)
option(HAVE_ASTRA_SDK "启用奥比中光 Astra SDK Body Tracking 骨骼后端" OFF)
set(NUITRACK_ROOT "" CACHE PATH "Nuitrack 安装目录（含 include/ 与 lib/）")
set(ORBBEC_SDK_ROOT "" CACHE PATH "Orbbec SDK v2 安装目录")
set(ASTRA_SDK_ROOT "" CACHE PATH "Astra SDK 2.x 安装目录（含 include/astra 与 lib/）")

# ---------------------------------------------------------------------------
#  源文件
# ---------------------------------------------------------------------------
set(POSE_SOURCES
  src/main.cpp
  src/types.cpp
  src/angles.cpp
  src/world_coords.cpp
  src/renderer.cpp
  src/color_source.cpp
  src/pose_provider.cpp
  src/mock_source.cpp
  src/opencv_source.cpp
  src/dnn_pose.cpp
  src/orbbec_source.cpp
  src/orbbec_pose_provider.cpp
  src/nuitrack_provider.cpp
  src/nuitrack_source.cpp
)

add_executable(pose_overlay ${POSE_SOURCES})
target_include_directories(pose_overlay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pose_overlay PRIVATE ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
#  Nuitrack
# ---------------------------------------------------------------------------
if(HAVE_NUITRACK)
  if(NUITRACK_ROOT STREQUAL "")
    message(FATAL_ERROR "启用 HAVE_NUITRACK 时必须指定 -DNUITRACK_ROOT=<Nuitrack 安装目录>")
  endif()
  find_path(NUITRACK_INCLUDE_DIR Nuitrack.h
    HINTS "${NUITRACK_ROOT}/include/nuitrack" "${NUITRACK_ROOT}/include")
  find_library(NUITRACK_LIBRARY
    NAMES nuitrack
    HINTS "${NUITRACK_ROOT}/lib" "${NUITRACK_ROOT}/lib/x64" "${NUITRACK_ROOT}/bin")
  if(NOT NUITRACK_INCLUDE_DIR OR NOT NUITRACK_LIBRARY)
    message(FATAL_ERROR
      "未在 ${NUITRACK_ROOT} 下找到 Nuitrack 头文件或库。\n"
      "请确认目录结构类似：<NUITRACK_ROOT>/include/nuitrack/Nuitrack.h 与 <NUITRACK_ROOT>/lib/nuitrack.lib")
  endif()
  target_include_directories(pose_overlay PRIVATE "${NUITRACK_INCLUDE_DIR}/..")
  target_link_libraries(pose_overlay PRIVATE "${NUITRACK_LIBRARY}")
  target_compile_definitions(pose_overlay PRIVATE HAVE_NUITRACK=1)
  message(STATUS "Nuitrack: ${NUITRACK_INCLUDE_DIR} | ${NUITRACK_LIBRARY}")
endif()

# ---------------------------------------------------------------------------
#  Orbbec SDK v2
# ---------------------------------------------------------------------------
if(HAVE_ORBBEC_SDK)
  if(ORBBEC_SDK_ROOT STREQUAL "")
    message(FATAL_ERROR "启用 HAVE_ORBBEC_SDK 时必须指定 -DORBBEC_SDK_ROOT=<OrbbecSDK 安装目录>")
  endif()
  find_path(ORBBEC_INCLUDE_DIR libobsensor/ObSensor.hpp
    HINTS "${ORBBEC_SDK_ROOT}/include" "${ORBBEC_SDK_ROOT}/SDK/include")
  find_library(ORBBEC_LIBRARY
    NAMES ob obsensor OrbbecSDK
    HINTS "${ORBBEC_SDK_ROOT}/lib" "${ORBBEC_SDK_ROOT}/lib/x64" "${ORBBEC_SDK_ROOT}/SDK/lib")
  if(NOT ORBBEC_INCLUDE_DIR OR NOT ORBBEC_LIBRARY)
    message(FATAL_ERROR
      "未在 ${ORBBEC_SDK_ROOT} 下找到 Orbbec SDK 头文件或库。\n"
      "期望：<ROOT>/include/libobsensor/ObSensor.hpp 与 <ROOT>/lib/ob.lib（或 obsensor.lib）")
  endif()
  target_include_directories(pose_overlay PRIVATE "${ORBBEC_INCLUDE_DIR}")
  target_link_libraries(pose_overlay PRIVATE "${ORBBEC_LIBRARY}")
  target_compile_definitions(pose_overlay PRIVATE HAVE_ORBBEC_SDK=1)
  message(STATUS "Orbbec SDK: ${ORBBEC_INCLUDE_DIR} | ${ORBBEC_LIBRARY}")
endif()

# ---------------------------------------------------------------------------
#  奥比中光 Astra SDK Body Tracking（骨骼）
#  注意：需要机器上有有效的 Body Tracking License；试用版会过期。
# ---------------------------------------------------------------------------
if(HAVE_ASTRA_SDK)
  if(ASTRA_SDK_ROOT STREQUAL "")
    message(FATAL_ERROR "启用 HAVE_ASTRA_SDK 时必须指定 -DASTRA_SDK_ROOT=<Astra SDK 安装目录>")
  endif()
  find_path(ASTRA_INCLUDE_DIR astra/astra.hpp
    HINTS "${ASTRA_SDK_ROOT}/include" "${ASTRA_SDK_ROOT}/Include")
  find_library(ASTRA_LIBRARY
    NAMES astra libastra
    HINTS "${ASTRA_SDK_ROOT}/lib" "${ASTRA_SDK_ROOT}/lib/x64" "${ASTRA_SDK_ROOT}/Lib")
  if(NOT ASTRA_INCLUDE_DIR OR NOT ASTRA_LIBRARY)
    message(FATAL_ERROR
      "未在 ${ASTRA_SDK_ROOT} 下找到 Astra SDK 头文件或库。\n"
      "期望：<ROOT>/include/astra/astra.hpp 与 <ROOT>/lib/astra.lib")
  endif()
  target_include_directories(pose_overlay PRIVATE "${ASTRA_INCLUDE_DIR}")
  target_link_libraries(pose_overlay PRIVATE "${ASTRA_LIBRARY}")
  target_compile_definitions(pose_overlay PRIVATE HAVE_ASTRA_SDK=1)
  message(STATUS "Astra SDK: ${ASTRA_INCLUDE_DIR} | ${ASTRA_LIBRARY}")
endif()

# ---------------------------------------------------------------------------
#  Windows：把运行时 DLL 复制到可执行文件旁边（可选，方便直接双击运行）
# ---------------------------------------------------------------------------
message(STATUS "---- pose_overlay 配置摘要 ----")
message(STATUS "  OpenCV        : ${OpenCV_VERSION} (${OpenCV_DIR})")
message(STATUS "  Nuitrack      : ${HAVE_NUITRACK}")
message(STATUS "  Orbbec SDK v2 : ${HAVE_ORBBEC_SDK}")
message(STATUS "  Astra BodyTrk : ${HAVE_ASTRA_SDK}")
message(STATUS "  构建类型      : ${CMAKE_BUILD_TYPE}")
