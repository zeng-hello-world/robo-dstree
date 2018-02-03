cmake_minimum_required(VERSION 2.8.3)
project(ground_removal)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            pcl_ros)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(ground_removal
        src/grid_map_class/grid_map.h
        src/grid_map_class/grid_map.cpp)

find_package( OpenCV REQUIRED )

target_link_libraries(ground_removal ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(ros_pub src/ros_pub.cpp)
target_link_libraries(ros_pub ${catkin_LIBRARIES} ${OpenCV_LIBS} ground_removal)
