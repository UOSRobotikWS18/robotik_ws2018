cmake_minimum_required(VERSION 2.8.3)
project(scanmatching)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  laser_geometry
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(scanmatching src/scanmatching.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(scanmatching
   ${catkin_LIBRARIES}
 )
