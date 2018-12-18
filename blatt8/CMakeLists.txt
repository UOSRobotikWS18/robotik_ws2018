cmake_minimum_required(VERSION 2.8.3)
project(blatt8)

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  laser_geometry
  nav_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  tf
)

find_package(Eigen REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(grid_mapper src/grid_mapper.cpp)
target_link_libraries(grid_mapper ${catkin_LIBRARIES})


