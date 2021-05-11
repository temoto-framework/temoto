cmake_minimum_required(VERSION 2.8.3)
project(example_temoto_config_pkg)

# use c++11 standard
add_compile_options(-std=c++14)

option(TEMOTO_ENABLE_TRACING_ "Use tracer" OFF)
if(TEMOTO_ENABLE_TRACING_)
  add_compile_options(-Dtemoto_enable_tracing)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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

# component_string_0
add_executable(component_string_0 src/dummy_components/component_string_0.cpp)
add_dependencies(component_string_0 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(component_string_0 ${catkin_LIBRARIES} )

# component_string_1
add_executable(component_string_1 src/dummy_components/component_string_1.cpp)
add_dependencies(component_string_1 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(component_string_1 ${catkin_LIBRARIES} )

# component_string_2
add_executable(component_string_2 src/dummy_components/component_string_2.cpp)
add_dependencies(component_string_2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(component_string_2 ${catkin_LIBRARIES} )