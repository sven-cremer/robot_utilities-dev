cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generator
  CATKIN_DEPENDS geometry_msgs #roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(trajectory_generator
  src/trajectoryGenerator.cpp
)

## Add cmake target dependencies of the library
# add_dependencies(trajectory_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_generator
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_generator_node
  trajectory_generator
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS trajectory_generator trajectory_generator_node
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
