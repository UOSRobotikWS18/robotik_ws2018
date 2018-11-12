cmake_minimum_required(VERSION 2.8.3)
project(blatt2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(imu_cpp src/imu.cpp)

target_link_libraries(imu_cpp
   ${catkin_LIBRARIES}
)
