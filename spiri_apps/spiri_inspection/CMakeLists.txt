
cmake_minimum_required(VERSION 2.8.3)
project(spiri_inspection)
find_package(catkin REQUIRED COMPONENTS spiri_api)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(inspect inspect.cpp)
target_link_libraries(inspect ${catkin_LIBRARIES})
