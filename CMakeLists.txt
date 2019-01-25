cmake_minimum_required(VERSION 2.8.3)
project(temoto)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
 CATKIN_DEPENDS roscpp rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


