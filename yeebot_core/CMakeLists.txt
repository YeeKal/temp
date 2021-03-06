cmake_minimum_required(VERSION 2.8.3)
project(yeebot_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  srdfdom
  pluginlib
  kdl_parser
)  

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(orocos_kdl REQUIRED)
find_package(ompl REQUIRED)

#set(ompl_include_dirs /home/yee/ompl_moveit/devel/include/ompl
#)
#set(ompl_libraries /home/yee/ompl_moveit/build/ompl/libompl.so
#)
catkin_package(
  CATKIN_DEPENDS
    roscpp
    urdf
    srdfdom
    pluginlib
    kdl_parser   
                       #important
  
  LIBRARIES 
    ${PROJECT_NAME} 
    ${orocos_kdl_LIBRARIES}  
   # ${ompl_libraries}#important
  INCLUDE_DIRS  #which is neede to indicate the include path
    include
   # ${OMPL_INCLUDE_DIRS}
  DEPENDS
    EIGEN3
    orocos_kdl
   # ompl
)

include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIRS}
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
  #SYSTEM ${OMPL_INCLUDE_DIRS}
)
#link_directories(BEFORE ${OMPL_LIBRARY_DIRS})




add_library(${PROJECT_NAME}
  src/kine_kdl.cpp
  src/pose_constraint.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
#ompl should be OMPL

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

