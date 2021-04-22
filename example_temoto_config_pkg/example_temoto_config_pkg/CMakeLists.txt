cmake_minimum_required(VERSION 2.8.3)
project(example_temoto_config_pkg)

# use c++11 standard
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  temoto_er_manager
  temoto_component_manager
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build external resource manager client
add_executable(er_manager_client
  src/er_manager_client.cpp
)

add_dependencies(er_manager_client 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(er_manager_client 
  ${catkin_LIBRARIES}
)

# Build component manager client
add_executable(component_manager_client
  src/component_manager_client.cpp
)

add_dependencies(component_manager_client 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(component_manager_client 
  ${catkin_LIBRARIES}
)
