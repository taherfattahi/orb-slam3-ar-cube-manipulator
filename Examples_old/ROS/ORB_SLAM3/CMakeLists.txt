cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Find libmp.so
find_library(LIBMP mp PATHS /home/taher/mediapipe/bazel-bin/mediapipe/examples/desktop/libmp)
if(NOT LIBMP)
  message(FATAL_ERROR "libmp not found in /home/taher/mediapipe/bazel-bin/mediapipe/examples/desktop/libmp")
endif()	

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../../../
  ${PROJECT_SOURCE_DIR}/../../../include
  ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  /home/taher/mediapipe
  /home/taher/mediapipe/mediapipe/examples/desktop/libmp
)

set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
  -lboost_system
  ${LIBMP}
)

# Custom target to copy MediaPipe resources
add_custom_target(copy_mediapipe_resources
  COMMAND ${CMAKE_COMMAND} -E copy_directory /home/taher/mediapipe/bazel-bin/mediapipe/modules ${CMAKE_BINARY_DIR}/modules
  COMMAND ${CMAKE_COMMAND} -E copy_directory /home/taher/mediapipe/bazel-bin/mediapipe/models ${CMAKE_BINARY_DIR}/models
)

# Node for monocular camera
rosbuild_add_executable(Mono
  src/ros_mono.cc
)
target_link_libraries(Mono ${LIBS})
add_dependencies(Mono copy_mediapipe_resources)

# Node for monocular camera (Augmented Reality Demo)
rosbuild_add_executable(MonoAR
  src/AR/ros_mono_ar.cc
  src/AR/ViewerAR.h
  src/AR/ViewerAR.cc
)
target_link_libraries(MonoAR ${LIBS})
add_dependencies(MonoAR copy_mediapipe_resources)

# Node for stereo camera
rosbuild_add_executable(Stereo
  src/ros_stereo.cc
)
target_link_libraries(Stereo ${LIBS})
add_dependencies(Stereo copy_mediapipe_resources)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
  src/ros_rgbd.cc
)
target_link_libraries(RGBD ${LIBS})
add_dependencies(RGBD copy_mediapipe_resources)

# Node for monocular-inertial camera
rosbuild_add_executable(Mono_Inertial
  src/ros_mono_inertial.cc
)
target_link_libraries(Mono_Inertial ${LIBS})
add_dependencies(Mono_Inertial copy_mediapipe_resources)

# Node for stereo-inertial camera
rosbuild_add_executable(Stereo_Inertial
  src/ros_stereo_inertial.cc
)
target_link_libraries(Stereo_Inertial ${LIBS})
add_dependencies(Stereo_Inertial copy_mediapipe_resources)
