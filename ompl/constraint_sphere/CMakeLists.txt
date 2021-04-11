cmake_minimum_required(VERSION 2.8)
project( sphere_constrained_planning )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

set(OMPL_LIBRARIES /usr/local/lib/libompl.so)


#list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

add_executable( sphere_constrained_planning sphere_constrained_planning.cpp )

include_directories( "/usr/include/eigen3" )
find_package( ompl REQUIRED )
#find_package( omplapp REQUIRED )

include_directories(${OMPL_INCLUDE_DIRS})

#IMPORTANT
target_link_libraries( sphere_constrained_planning ${OMPL_LIBRARIES} )
