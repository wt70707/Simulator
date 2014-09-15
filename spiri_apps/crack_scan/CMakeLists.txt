
cmake_minimum_required(VERSION 2.8.3)
project(crack_scan)
find_package(catkin REQUIRED COMPONENTS spiri_api)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(crack_scan crack_scan.cpp window.cpp)
target_link_libraries(crack_scan ${catkin_LIBRARIES})
