cmake_minimum_required(VERSION 3.16)
project(LaserAimer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(RUNTIME_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set(RUNTIME_INCLUDE "${RUNTIME_ROOT}/include")
set(AUTO_AIM_SERIAL_DIR "${RUNTIME_ROOT}/applications/auto_aim/io/serial")
set(LASER_AIMER_OUTPUT_DIR "${CMAKE_CURRENT_LIST_DIR}/bin")
option(LASER_AIMER_WITH_TRT "Enable TensorRT/TRTInferX drone detector backend" OFF)
set(LASER_AIMER_TRTINFERX_INCLUDE "" CACHE PATH "TRTInferX include directory, for example references/.../src/detector/TRTInferX/include")
set(LASER_AIMER_TRTINFERX_LIBRARY "" CACHE FILEPATH "Built TRTInferX/trt_yolo library")
set(LASER_AIMER_TENSORRT_INCLUDE "" CACHE PATH "TensorRT include directory")
set(LASER_AIMER_TENSORRT_LIBRARY_DIR "" CACHE PATH "TensorRT library directory")
set(LASER_AIMER_CUDA_LIBRARY_DIR "" CACHE PATH "CUDA library directory")
set(LASER_AIMER_TRT_EXTRA_LIBS "" CACHE STRING "Extra TRT/CUDA libraries needed for linking, as a CMake semicolon-separated list")

set(LASER_AIMER_SOURCES
    main.cpp
    types.cpp
    config.cpp
    LaserAimerProvider.cpp
    LaserAimerSystem.cpp
    LaserAimerSelector.cpp
    LaserAimerSolver.cpp
    LaserAimerActuator.cpp
    camera/FrameSource.cpp
    camera/MindVisionCamera.cpp
    control/Controller.cpp
    detector/DroneDetector.cpp
    detector/FixedTargetDetector.cpp
    io/GimbalSerial.cpp
    "${AUTO_AIM_SERIAL_DIR}/src/serial.cc"
)

if(WIN32)
  list(APPEND LASER_AIMER_SOURCES
      "${AUTO_AIM_SERIAL_DIR}/src/impl/win.cc")
else()
  list(APPEND LASER_AIMER_SOURCES
      "${AUTO_AIM_SERIAL_DIR}/src/impl/unix.cc"
      "${AUTO_AIM_SERIAL_DIR}/src/impl/list_ports/list_ports_linux.cc")
endif()

add_executable(laser_aimer ${LASER_AIMER_SOURCES})

set_target_properties(laser_aimer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${LASER_AIMER_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${LASER_AIMER_OUTPUT_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${LASER_AIMER_OUTPUT_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${LASER_AIMER_OUTPUT_DIR}/RelWithDebInfo"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${LASER_AIMER_OUTPUT_DIR}/MinSizeRel"
)

target_include_directories(laser_aimer PRIVATE
    ${RUNTIME_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    ${AUTO_AIM_SERIAL_DIR}/include
)

target_link_libraries(laser_aimer PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

if(LASER_AIMER_WITH_TRT)
  if(NOT EXISTS "${LASER_AIMER_TRTINFERX_INCLUDE}/api.h")
    message(FATAL_ERROR "LASER_AIMER_WITH_TRT=ON but api.h was not found. Set LASER_AIMER_TRTINFERX_INCLUDE.")
  endif()
  if(NOT EXISTS "${LASER_AIMER_TRTINFERX_LIBRARY}")
    message(FATAL_ERROR "LASER_AIMER_WITH_TRT=ON but TRTInferX library was not found. Set LASER_AIMER_TRTINFERX_LIBRARY.")
  endif()

  target_compile_definitions(laser_aimer PRIVATE LASER_AIMER_WITH_TRT)
  target_include_directories(laser_aimer PRIVATE
      "${LASER_AIMER_TRTINFERX_INCLUDE}"
      "${LASER_AIMER_TENSORRT_INCLUDE}"
  )
  target_link_libraries(laser_aimer PRIVATE "${LASER_AIMER_TRTINFERX_LIBRARY}")

  set(_laser_aimer_trt_library_dirs)
  if(LASER_AIMER_TENSORRT_LIBRARY_DIR)
    list(APPEND _laser_aimer_trt_library_dirs "${LASER_AIMER_TENSORRT_LIBRARY_DIR}")
  endif()
  if(LASER_AIMER_CUDA_LIBRARY_DIR)
    list(APPEND _laser_aimer_trt_library_dirs "${LASER_AIMER_CUDA_LIBRARY_DIR}")
  endif()
  if(LASER_AIMER_TENSORRT_INCLUDE)
    get_filename_component(_laser_aimer_cuda_root "${LASER_AIMER_TENSORRT_INCLUDE}" DIRECTORY)
    list(APPEND _laser_aimer_trt_library_dirs
        "${_laser_aimer_cuda_root}/lib"
        "${_laser_aimer_cuda_root}/lib/x64")
  endif()

  find_library(LASER_AIMER_NVINFER_LIBRARY nvinfer PATHS ${_laser_aimer_trt_library_dirs} NO_DEFAULT_PATH)
  find_library(LASER_AIMER_NVINFER_PLUGIN_LIBRARY nvinfer_plugin PATHS ${_laser_aimer_trt_library_dirs} NO_DEFAULT_PATH)
  find_library(LASER_AIMER_NVONNXPARSER_LIBRARY nvonnxparser PATHS ${_laser_aimer_trt_library_dirs} NO_DEFAULT_PATH)
  find_library(LASER_AIMER_CUDART_LIBRARY cudart PATHS ${_laser_aimer_trt_library_dirs} NO_DEFAULT_PATH)

  foreach(_laser_aimer_required_trt_lib
      LASER_AIMER_NVINFER_LIBRARY
      LASER_AIMER_NVINFER_PLUGIN_LIBRARY
      LASER_AIMER_NVONNXPARSER_LIBRARY
      LASER_AIMER_CUDART_LIBRARY)
    if(NOT ${_laser_aimer_required_trt_lib})
      message(FATAL_ERROR "LASER_AIMER_WITH_TRT=ON but ${_laser_aimer_required_trt_lib} was not found. Set LASER_AIMER_TENSORRT_LIBRARY_DIR and LASER_AIMER_CUDA_LIBRARY_DIR.")
    endif()
    target_link_libraries(laser_aimer PRIVATE "${${_laser_aimer_required_trt_lib}}")
  endforeach()

  if(LASER_AIMER_TRT_EXTRA_LIBS)
    foreach(_laser_aimer_extra_trt_lib IN LISTS LASER_AIMER_TRT_EXTRA_LIBS)
      target_link_libraries(laser_aimer PRIVATE "${_laser_aimer_extra_trt_lib}")
    endforeach()
  endif()
endif()

if(MSVC)
  target_compile_options(laser_aimer PRIVATE /utf-8)
endif()

if(WIN32)
  target_link_libraries(laser_aimer PRIVATE setupapi)

  set(MINDVISION_ROOT "C:/Program Files (x86)/MindVision" CACHE PATH "MindVision SDK root")
  set(MINDVISION_INCLUDE "${MINDVISION_ROOT}/Demo/VC++/Include" CACHE PATH "MindVision include path")
  set(MINDVISION_LIB "${MINDVISION_ROOT}/SDK/X64/MVCAMSDK_X64.lib" CACHE FILEPATH "MindVision x64 import library")
  set(MINDVISION_DLL "${MINDVISION_ROOT}/SDK/X64/MVCAMSDK_X64.dll" CACHE FILEPATH "MindVision x64 runtime DLL")

  if(NOT EXISTS "${MINDVISION_INCLUDE}/CameraApi.h")
    message(FATAL_ERROR "MindVision CameraApi.h not found. Set MINDVISION_INCLUDE to your SDK Include directory.")
  endif()
  if(NOT EXISTS "${MINDVISION_LIB}")
    message(FATAL_ERROR "MindVision MVCAMSDK_X64.lib not found. Set MINDVISION_LIB to your SDK import library.")
  endif()

  target_include_directories(laser_aimer PRIVATE "${MINDVISION_INCLUDE}")
  target_link_libraries(laser_aimer PRIVATE "${MINDVISION_LIB}")

  if(EXISTS "${MINDVISION_DLL}")
    add_custom_command(TARGET laser_aimer POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${MINDVISION_DLL}" "$<TARGET_FILE_DIR:laser_aimer>")
  endif()

else()
  message(FATAL_ERROR "laser_aimer currently targets Windows MindVision SDK. Add Linux SDK paths before building on Linux.")
endif()
