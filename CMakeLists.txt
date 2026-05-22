cmake_minimum_required(VERSION 3.10)
project(SimpleAimerRuntime)

option(BUILD_LASER_AIMER "Build LaserTracking-based laser aimer application" ON)
if(WIN32)
  option(BUILD_AUTO_AIM "Build auto_aim application" OFF)
else()
  option(BUILD_AUTO_AIM "Build auto_aim application" ON)
endif()

if(BUILD_LASER_AIMER)
  add_subdirectory(applications/laser_aimer)
endif()

if(BUILD_AUTO_AIM)
  add_subdirectory(applications/auto_aim)
endif()
