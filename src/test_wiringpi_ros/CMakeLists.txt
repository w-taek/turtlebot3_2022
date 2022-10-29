cmake_minimum_required(VERSION 3.0.2)
project(test_wiringpi_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_wiringpi_ros src/test_wiringpi_ros.cpp)
target_link_libraries(test_wiringpi_ros ${catkin_LIBRARIES} -lwiringPi)