cmake_minimum_required(VERSION 2.8.3)
project(ta_dummy_navigation)

# use c++11 standard
add_compile_options(-std=c++11)
option(TEMOTO_ENABLE_TRACING "Use tracer" OFF)

if(TEMOTO_ENABLE_TRACING)
  add_compile_options(-Denable_tracing)
endif()

set(ACTION_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/)

find_package(catkin REQUIRED COMPONENTS
  class_loader
  roscpp
  rospy
  temoto_action_engine
  temoto_core
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

file(MAKE_DIRECTORY ${ACTION_LIB_DIR})

set_target_properties(${PROJECT_NAME}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ACTION_LIB_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${class_loader_LIBRARIES}
  ${catkin_LIBRARIES}
)

class_loader_hide_library_symbols(${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})