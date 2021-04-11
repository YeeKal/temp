cmake_minimum_required(VERSION 2.8) 
project(temp)  
find_package(OpenCV REQUIRED) 
find_package(Boost REQUIRED)
# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# find_package(nlopt REQUIRED)
 

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")  
include_directories("/usr/include/eigen3")

add_executable(temp temp.cpp )  
target_link_libraries(temp ${OpenCV_LIBS})
target_link_libraries (temp ${Boost_LIBRARIES})
target_link_libraries(temp ${CERES_LIBRARIES})

add_executable(corner_detection corner_detection.cpp )  
target_link_libraries(corner_detection ${OpenCV_LIBS})

add_executable(hough_transform hough_transform.cpp )  
target_link_libraries(hough_transform ${OpenCV_LIBS})


add_executable(sift_match sift_match.cpp )  
target_link_libraries(sift_match ${OpenCV_LIBS})

add_executable(histogram_equalization histogram_equalization.cpp )  
target_link_libraries(histogram_equalization ${OpenCV_LIBS})

add_executable(hsv_seg hsv_seg.cpp )  
target_link_libraries(hsv_seg ${OpenCV_LIBS})

add_executable(nlopt_demo nlopt_demo.cpp )  
# target_link_libraries(nlopt_demo ${Nlopt_LIBRARIES})
target_link_libraries(nlopt_demo "/usr/local/lib/libnlopt.so")
