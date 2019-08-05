cmake_minimum_required(VERSION 2.8.3)
project(temoto)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  file_template_parser
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy roslib file_template_parser
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_ws_generator 
  src/temoto_ws_generator.cpp 
)
add_dependencies(temoto_ws_generator
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_ws_generator
  ${catkin_LIBRARIES}
)