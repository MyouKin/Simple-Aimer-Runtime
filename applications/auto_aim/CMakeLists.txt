cmake_minimum_required(VERSION 3.10)
project(AutoAim)

set(CMAKE_CXX_STANDARD 17)

# ---- 路径 ----
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/include/imgui-1.92.8")
set(RUNTIME_INCLUDE "${CMAKE_SOURCE_DIR}/include")

# ---- 依赖 ----
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# System specific for GLFW
if(APPLE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW3 REQUIRED glfw3)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(OPENGL_LIBRARY OpenGL)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(COREVIDEO_LIBRARY CoreVideo)
else()
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
endif()

# ---- YAML-CPP（用于配置加载） ----
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(YAMLCPP yaml-cpp)
    endif()
endif()
# 如果仍未找到，尝试 conda 路径
if(NOT yaml-cpp_FOUND AND NOT YAMLCPP_FOUND)
    if(DEFINED ENV{CONDA_PREFIX})
        set(YAMLCPP_INCLUDE_DIRS "$ENV{CONDA_PREFIX}/include")
        set(YAMLCPP_LIBRARIES "$ENV{CONDA_PREFIX}/lib/libyaml-cpp.dylib")
        message(STATUS "AutoAim: using conda yaml-cpp at $ENV{CONDA_PREFIX}")
    endif()
endif()

# ---- include 目录 ----
include_directories(
    ${RUNTIME_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)

# ---- TinyMPC（可选） ----
# 下载 TinyMPC 后，通过 -DTINYMPC_INCLUDE_DIR=/path/to/TinyMPC/include 启用 MPC
if(DEFINED TINYMPC_INCLUDE_DIR)
    add_definitions(-DTINYMPC_INCLUDE_DIR="${TINYMPC_INCLUDE_DIR}")
    add_definitions(-DAIM_HAS_TINYMPC)
    include_directories(${TINYMPC_INCLUDE_DIR})
    message(STATUS "AutoAim: TinyMPC enabled at ${TINYMPC_INCLUDE_DIR}")
else()
    message(STATUS "AutoAim: TinyMPC not found — MPC will be stubbed. "
                    "Download from https://github.com/TinyMPC/TinyMPC "
                    "and pass -DTINYMPC_INCLUDE_DIR=...")
endif()

if(APPLE)
    include_directories(${GLFW3_INCLUDE_DIRS})
endif()

# ---- 源文件 ----
file(GLOB APP_SOURCES "*.cpp")

# Runtime 共享工具（EKF、弹道、MPC 等）
file(GLOB TOOLS_SOURCES
    "${CMAKE_SOURCE_DIR}/src/tools/*.cpp"
)

# Runtime 调试界面
file(GLOB DEBUG_SOURCES
    "${CMAKE_SOURCE_DIR}/src/debug/*.cpp"
)

# ImGui
file(GLOB IMGUI_SOURCES
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)

add_executable(auto_aim ${APP_SOURCES} ${TOOLS_SOURCES} ${DEBUG_SOURCES} ${IMGUI_SOURCES})
target_compile_definitions(auto_aim PRIVATE GL_SILENCE_DEPRECATION)

# ---- 链接 ----
if(APPLE)
    target_link_directories(auto_aim PRIVATE ${GLFW3_LIBRARY_DIRS})
    target_link_libraries(auto_aim
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${GLFW3_LIBRARIES}
        ${COCOA_LIBRARY}
        ${OPENGL_LIBRARY}
        ${IOKIT_LIBRARY}
        ${COREVIDEO_LIBRARY}
    )
else()
    target_link_libraries(auto_aim
        ${OpenCV_LIBS}
        Eigen3::Eigen
        glfw
        ${OPENGL_LIBRARIES}
    )
endif()

# YAML-CPP
if(yaml-cpp_FOUND)
    target_link_libraries(auto_aim yaml-cpp)
elseif(YAMLCPP_FOUND)
    target_link_libraries(auto_aim ${YAMLCPP_LIBRARIES})
    target_include_directories(auto_aim PRIVATE ${YAMLCPP_INCLUDE_DIRS})
endif()