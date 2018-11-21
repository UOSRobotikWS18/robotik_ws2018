cmake_minimum_required(VERSION 2.8.3)
project(blatt5)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_gauss src/gauss.cpp)

target_link_libraries(${PROJECT_NAME}_gauss
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

