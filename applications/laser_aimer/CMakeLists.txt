cmake_minimum_required(VERSION 3.10)
project(LaserAimer)

set(CMAKE_CXX_STANDARD 17)

# Set up ImGui
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/include/imgui-1.92.8")

# Find packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# MindVision Camera SDK
set(MVSDK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include/mvsdk/include")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(MVSDK_LIB_DIR "${CMAKE_SOURCE_DIR}/include/mvsdk/lib/amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(MVSDK_LIB_DIR "${CMAKE_SOURCE_DIR}/include/mvsdk/lib/arm64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# System specific configurations for UI/GLFW
if(APPLE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW3 REQUIRED glfw3)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(OPENGL_LIBRARY OpenGL)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(COREVIDEO_LIBRARY CoreVideo)
else()
    # For Linux and Windows, standard CMake find_package for OpenGL and GLFW is usually sufficient
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${MVSDK_INCLUDE_DIR}
)

if(APPLE)
    include_directories(${GLFW3_INCLUDE_DIRS})
endif()

file(GLOB SOURCES 
    "*.cpp"
    "${CMAKE_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/debug/*.cpp"
)

# ImGui sources
file(GLOB IMGUI_SOURCES
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)

add_executable(laser_aimer ${SOURCES} ${IMGUI_SOURCES})

target_compile_definitions(laser_aimer PRIVATE GL_SILENCE_DEPRECATION)

if(APPLE)
    target_link_directories(laser_aimer PRIVATE ${GLFW3_LIBRARY_DIRS})
    target_link_libraries(laser_aimer
        ${OpenCV_LIBS}
        Eigen3::Eigen
        ${GLFW3_LIBRARIES}
        ${COCOA_LIBRARY}
        ${OPENGL_LIBRARY}
        ${IOKIT_LIBRARY}
        ${COREVIDEO_LIBRARY}
    )
else()
    target_link_directories(laser_aimer PRIVATE ${MVSDK_LIB_DIR})
    target_link_libraries(laser_aimer
        ${OpenCV_LIBS}
        Eigen3::Eigen
        glfw
        ${OPENGL_LIBRARIES}
        MVSDK
    )
endif()