project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()
add_executable(obstacle_detection_node Main.cpp)
target_link_libraries(obstacle_detection_node PUBLIC radar_driver ${catkin_LIBRARIES})
