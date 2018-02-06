cmake_minimum_required(VERSION 2.8.3)
project(normal_vis)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            pcl_ros
            visualization_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(normal_vis
            src/normal_vis.h
            src/normal_vis.cpp)

find_package( OpenCV REQUIRED )

target_link_libraries(normal_vis ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(normal_pub src/normal_pub.cpp)
target_link_libraries(normal_pub ${catkin_LIBRARIES} ${OpenCV_LIBS} normal_vis)