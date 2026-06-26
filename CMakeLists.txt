cmake_minimum_required(VERSION 3.10)
project(spring_joint_plugin VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Match ROS 2 Jazzy / Gazebo Harmonic version mappings exactly
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(spring_joint_plugin SHARED 
  src/spring_plugin.cpp
)

target_include_directories(spring_joint_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(spring_joint_plugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
)

set_target_properties(spring_joint_plugin PROPERTIES
  OUTPUT_NAME "spring_joint_plugin"
  PREFIX "lib"
)