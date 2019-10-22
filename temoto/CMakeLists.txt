cmake_minimum_required(VERSION 2.8.3)
project(temoto)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  temoto_core
  temoto_utils
  temoto_action_engine
  temoto_er_manager
  temoto_component_manager
)

catkin_package(
 CATKIN_DEPENDS temoto_core temoto_utils temoto_action_engine temoto_er_manager temoto_component_manager
)
