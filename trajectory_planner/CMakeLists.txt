cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  control_msgs
  actionlib
  tf
  visualization_msgs
  apc_msgs
  move_base_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #cirular_trajectoryLib
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs sensor_msgs control_msgs actionlib tf visualization_msgs apc_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

## Executables
add_executable(         cirular_trajectory       src/circle_trajectory.cpp)
target_link_libraries(  cirular_trajectory   	 ${catkin_LIBRARIES})
add_dependencies(       cirular_trajectory       ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS cirular_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
