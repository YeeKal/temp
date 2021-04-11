cmake_minimum_required(VERSION 2.8)

PROJECT(planner_yee)

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")  
#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

#设定存放编译出来的库文件的目录
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 

ADD_SUBDIRECTORY(src/rrt_star)
ADD_SUBDIRECTORY(src/rrt_star_split)



