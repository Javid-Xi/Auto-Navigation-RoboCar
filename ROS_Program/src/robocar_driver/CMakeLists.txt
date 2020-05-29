cmake_minimum_required(VERSION 2.8.3)
project(robocar_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable( robocar_driver
  include/driver.h
  src/robocar_driver.cpp)

target_link_libraries( robocar_driver
  ${catkin_LIBRARIES}
)

