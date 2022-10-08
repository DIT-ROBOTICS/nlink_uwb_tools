cmake_minimum_required(VERSION 3.0.2)
project(nlink_uwb)

add_compile_options(-std=c++11 -fpermissive -Wfatal-errors)

find_package(catkin REQUIRED COMPONENTS
  nlink_parser
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS nlink_parser roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nlink_tagframe_converter src/nlink_tagframe_converter.cpp)
target_link_libraries(nlink_tagframe_converter ${catkin_LIBRARIES})
add_dependencies(nlink_tagframe_converter ${catkin_EXPORTED_TARGETS})

