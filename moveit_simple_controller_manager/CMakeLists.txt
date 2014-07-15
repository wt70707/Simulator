cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_controller_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(Boost REQUIRED COMPONENTS system thread)

#include_directories(
#  SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#)

#link_directories(${Boost_LIBRARY_DIRS})


find_package(catkin REQUIRED COMPONENTS
  action_controller
  actionlib
  control_msgs
  moveit_core
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES moveit_simple_controller_manager
  CATKIN_DEPENDS action_controller actionlib control_msgs moveit_core pluginlib roscpp
  DEPENDS moveit_core
)


include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a cpp library
 add_library(moveit_simple_controller_manager
   src/moveit_simple_controller_manager.cpp
 )



 target_link_libraries(moveit_simple_controller_manager
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
 )

add_dependencies(moveit_simple_controller_manager action_controller_generate_messages_cpp)
#############
## Install ##
#############
install(TARGETS moveit_simple_controller_manager LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_simple_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )

