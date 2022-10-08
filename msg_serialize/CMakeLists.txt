cmake_minimum_required(VERSION 3.0.2)
project(msg_serialize)

add_compile_options(-std=c++11 -fpermissive -Wfatal-errors)

find_package(catkin REQUIRED COMPONENTS
  nlink_parser
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS nlink_parser roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_serialization src/twist_serialization.cpp)
target_link_libraries(twist_serialization ${catkin_LIBRARIES})
add_dependencies(twist_serialization ${catkin_EXPORTED_TARGETS})

add_executable(twist_deserialization src/twist_deserialization.cpp)
target_link_libraries(twist_deserialization ${catkin_LIBRARIES})
add_dependencies(twist_deserialization ${catkin_EXPORTED_TARGETS})
