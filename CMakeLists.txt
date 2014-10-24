set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_explorer)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs s8_msgs s8_common_node s8_turner)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp s8_msgs geometry_msgs s8_common_node s8_turner
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s8_explorer_node src/explorer_node.cpp)
target_link_libraries(s8_explorer_node ${catkin_LIBRARIES})
