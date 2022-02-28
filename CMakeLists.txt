cmake_minimum_required(VERSION 2.8)
project(rvio)

# Set build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

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

# Enable debug flags (if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g3")

# Find catkin (the ROS build system)
# Add this for finding ros dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp roslib tf sensor_msgs geometry_msgs nav_msgs cv_bridge eigen_conversions
)

# Describe catkin project
catkin_package(CATKIN_DEPENDS
    roscpp roslib tf std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge eigen_conversions
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 2.4.3 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif()
endif()

# Include directories of headers
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

##################################################
# Make binary for the main library
##################################################
add_library(${PROJECT_NAME} SHARED
  src/rvio/System.cc
  src/rvio/Updater.cc
  src/rvio/Tracker.cc
  src/rvio/PreIntegrator.cc
  src/rvio/Ransac.cc
  src/rvio/InputBuffer.cc
  src/rvio/FeatureDetector.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
)

##################################################
# Make binary for main method ROS
##################################################
add_executable(rvio_mono src/rvio_mono.cc)
target_link_libraries(rvio_mono
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
