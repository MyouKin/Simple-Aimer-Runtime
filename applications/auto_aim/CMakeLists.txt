cmake_minimum_required(VERSION 3.10)
project(AutoAim)

set(CMAKE_CXX_STANDARD 17)

# ---- 路径 ----
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/include/imgui-1.92.8")
set(RUNTIME_INCLUDE "${CMAKE_SOURCE_DIR}/include")

# ---- 依赖 ----
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# OpenVINO（分类器使用，来源: spr_vision_try 的 classifier.cpp）
find_package(OpenVINO QUIET)
if(NOT OpenVINO_FOUND)
    # 尝试 conda 环境
    if(DEFINED ENV{CONDA_PREFIX})
        set(OpenVINO_DIR "$ENV{CONDA_PREFIX}/runtime/cmake")
        find_package(OpenVINO QUIET)
    endif()
endif()
if(NOT OpenVINO_FOUND)
    message(WARNING "AutoAim: OpenVINO not found — classifier will fail to link. "
                    "Install from https://docs.openvino.ai/")
endif()

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

# ---- SPDLOG（日志库，MindVision/HikRobot 驱动依赖） ----
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    if(DEFINED ENV{CONDA_PREFIX})
        set(spdlog_DIR "$ENV{CONDA_PREFIX}/share/spdlog")
        find_package(spdlog QUIET)
    endif()
endif()
if(NOT spdlog_FOUND)
    message(WARNING "AutoAim: spdlog not found — camera logger will fail to link. "
                    "Install with: sudo apt install libspdlog-dev  or  conda install spdlog")
endif()

# 启用 yaml-cpp 配置加载（AutoAimSystem / AutoAimSolver 通过此宏选择解析器）
add_definitions(-DHAS_YAML_CPP)

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

# ---- AutoAimProvider 相关路径（相机驱动 io + tools） ----
set(AUTO_AIM_DIR "${CMAKE_SOURCE_DIR}/applications/auto_aim")
set(PROVIDER_DIR "${AUTO_AIM_DIR}/AutoAimProvider")

# ---- include 目录 ----
include_directories(
    ${RUNTIME_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${PROVIDER_DIR}                                    # for AutoAimProvider/io/camera.hpp etc.
    ${PROVIDER_DIR}/io/mindvision/include               # for CameraApi.h (MindVision SDK)
    ${PROVIDER_DIR}/io/hikrobot/include                 # for MvCameraControl.h (HikRobot SDK)
    ${PROVIDER_DIR}/io/serial/include                   # for serial.h (串口库)
)

# ---- TinyMPC（位于 include/tinympc/，spr_vision_try 移植） ----
set(TINYMPC_DIR "${CMAKE_SOURCE_DIR}/include/tinympc")
if(EXISTS "${TINYMPC_DIR}/tiny_api.cpp")
    add_definitions(-DTINYMPC_INCLUDE_DIR="${TINYMPC_DIR}")
    add_definitions(-DAIM_HAS_TINYMPC)
    include_directories(${TINYMPC_DIR})
    message(STATUS "AutoAim: TinyMPC enabled at ${TINYMPC_DIR}")
else()
    message(STATUS "AutoAim: TinyMPC not found — MPC will be stubbed.")
endif()

if(APPLE)
    include_directories(${GLFW3_INCLUDE_DIRS})
endif()

# ---- 源文件 ----
file(GLOB APP_SOURCES "*.cpp")

# 确保 assets 软链接存在（模型文件路径）
add_custom_target(assets_link ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${CMAKE_SOURCE_DIR}/applications/auto_aim/AutoAimProvider/assets"
        "${CMAKE_BINARY_DIR}/assets"
    COMMENT "Creating assets symlink"
)

# AutoAimProvider（相机驱动 + 检测器 + 分类器 + PnP + YOLO + 串口云台）
file(GLOB PROVIDER_SOURCES
    "${PROVIDER_DIR}/AutoAimProvider.cpp"
    "${PROVIDER_DIR}/detector.cpp"
    "${PROVIDER_DIR}/classifier.cpp"
    "${PROVIDER_DIR}/pnp_solver.cpp"
    "${PROVIDER_DIR}/yolo.cpp"
    "${PROVIDER_DIR}/yolos/yolov5.cpp"
    "${PROVIDER_DIR}/yolos/yolov8.cpp"
    "${PROVIDER_DIR}/yolos/yolo11.cpp"
    "${PROVIDER_DIR}/io/gimbal/gimbal.cpp"
)

# 串口库（wjwwood/serial）
file(GLOB SERIAL_SOURCES
    "${PROVIDER_DIR}/io/serial/src/serial.cc"
    "${PROVIDER_DIR}/io/serial/src/impl/unix.cc"
    "${PROVIDER_DIR}/io/serial/src/impl/list_ports/list_ports_linux.cc"
)

# TinyMPC 求解器
file(GLOB TINYMPC_SOURCES
    "${TINYMPC_DIR}/admm.cpp"
    "${TINYMPC_DIR}/tiny_api.cpp"
    "${TINYMPC_DIR}/codegen.cpp"
    "${TINYMPC_DIR}/rho_benchmark.cpp"
)

# AutoAimProvider 相机驱动（io: camera + mindvision + hikrobot）
file(GLOB PROVIDER_IO_SOURCES
    "${PROVIDER_DIR}/io/camera.cpp"
    "${PROVIDER_DIR}/io/mindvision/mindvision.cpp"
    "${PROVIDER_DIR}/io/hikrobot/hikrobot.cpp"
)

# AutoAimProvider 共享工具（logger / img_tools / math_tools 等）
file(GLOB PROVIDER_TOOLS_SOURCES
    "${PROVIDER_DIR}/tools/logger.cpp"
    "${PROVIDER_DIR}/tools/math_tools.cpp"
    "${PROVIDER_DIR}/tools/img_tools.cpp"
)

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

add_executable(auto_aim ${APP_SOURCES} ${PROVIDER_SOURCES} ${PROVIDER_IO_SOURCES} ${PROVIDER_TOOLS_SOURCES} ${TOOLS_SOURCES} ${DEBUG_SOURCES} ${IMGUI_SOURCES} ${SERIAL_SOURCES} ${TINYMPC_SOURCES})
add_dependencies(auto_aim assets_link)
target_compile_definitions(auto_aim PRIVATE GL_SILENCE_DEPRECATION)

# ---- 链接 ----
# ---- MindVision / HikRobot SDK 库路径（根据架构） ----
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(MINDVISION_LIB_DIR "${PROVIDER_DIR}/io/mindvision/lib/amd64")
    set(HIKROBOT_LIB_DIR "${PROVIDER_DIR}/io/hikrobot/lib/amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(MINDVISION_LIB_DIR "${PROVIDER_DIR}/io/mindvision/lib/arm64")
    set(HIKROBOT_LIB_DIR "${PROVIDER_DIR}/io/hikrobot/lib/arm64")
else()
    message(WARNING "Unknown architecture, mindvision/hikrobot libs may not link!")
endif()

if(APPLE)
    target_link_directories(auto_aim PRIVATE
        ${GLFW3_LIBRARY_DIRS}
        ${MINDVISION_LIB_DIR}
        ${HIKROBOT_LIB_DIR}
    )
    target_link_libraries(auto_aim
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${GLFW3_LIBRARIES}
        ${COCOA_LIBRARY}
        ${OPENGL_LIBRARY}
        ${IOKIT_LIBRARY}
        ${COREVIDEO_LIBRARY}
        MVSDK
        MvCameraControl
        usb-1.0
        spdlog
        fmt
        openvino        # OpenVINO runtime (classifier)
        pthread
    )
else()
    target_link_directories(auto_aim PRIVATE
        ${MINDVISION_LIB_DIR}
        ${HIKROBOT_LIB_DIR}
    )

    target_link_libraries(auto_aim
        ${OpenCV_LIBS}
        Eigen3::Eigen
        glfw
        ${OPENGL_LIBRARIES}
        MVSDK            # MindVision SDK
        MvCameraControl  # HikRobot SDK
        usb-1.0          # libusb
        spdlog           # 日志
        fmt              # spdlog 依赖的格式化库
        openvino         # OpenVINO runtime (classifier)
        pthread          # 线程
        rt               # 实时库（串口）
    )
endif()

# YAML-CPP
if(yaml-cpp_FOUND)
    target_link_libraries(auto_aim yaml-cpp)
elseif(YAMLCPP_FOUND)
    target_link_libraries(auto_aim ${YAMLCPP_LIBRARIES})
    target_include_directories(auto_aim PRIVATE ${YAMLCPP_INCLUDE_DIRS})
endif()