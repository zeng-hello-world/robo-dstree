cmake_minimum_required(VERSION 2.8.3)
project(bootstrapping)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             pcl_ros)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(bootstrapping
            src/bootstrapping_include.h
            src/bootstrapping.h
            src/bootstrapping.cpp)

find_package( OpenCV REQUIRED )

target_link_libraries(bootstrapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(talker src/talker.cpp)
#add_executable(talker src/DS_tree_talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${OpenCV_LIBS} bootstrapping)