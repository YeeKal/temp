find_package(OpenCV REQUIRED) 

aux_source_directory(. SRC_RRT_STAR)

add_executable(rrt_demo rrt_demo.cpp rrt.cpp scene.cpp)  
target_link_libraries(rrt_demo ${OpenCV_LIBS})
add_executable(rrt_connect_demo rrt_connect_demo.cpp rrt_connect.cpp scene.cpp)  
target_link_libraries(rrt_connect_demo ${OpenCV_LIBS})
