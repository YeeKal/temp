find_package(OpenCV REQUIRED) 

aux_source_directory(. SRC_RRT_STAR)

add_executable(rrt_star ${SRC_RRT_STAR})  
target_link_libraries(rrt_star ${OpenCV_LIBS})
