cmake_minimum_required(VERSION 2.8.3)
project(ecn_visualodom)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE DEBUG)
find_package(VISP REQUIRED COMPONENTS ar core gui imgproc)

find_package(OpenCV)
find_package(Boost COMPONENTS thread date_time REQUIRED)

###########

## Specify additional locations of header files
include_directories(
  include
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Visual Odometry
add_executable(visualodom vo.cpp
                include/ecn_visualodom/ar_display.h
                include/ecn_visualodom/video_loop.h
                include/ecn_visualodom/visual_odom.h src/visual_odom.cpp
                )
target_link_libraries(visualodom ${VISP_LIBRARIES}  ${OpenCV_LIBS} Boost::date_time Boost::thread)
