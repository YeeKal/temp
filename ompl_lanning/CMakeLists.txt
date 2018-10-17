cmake_minimum_required(VERSION 2.8.3)
project(ompl_lanning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(ompl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
		EIGEN3
		ompl
)
include_directories(
	include ${catkin_include_dirs}
	"include/ompl_lanning"
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories( "/usr/include/eigen3" )
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(ur5_planning_ompl src/ur5_planning_ompl.cpp)
target_link_libraries(ur5_planning_ompl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})
install(TARGETS ur5_planning_ompl DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(ur5_planning_ompl_constraint src/ur5_planning_ompl_constraint.cpp)
target_link_libraries(ur5_planning_ompl_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})
install(TARGETS ur5_planning_ompl_constraint DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


