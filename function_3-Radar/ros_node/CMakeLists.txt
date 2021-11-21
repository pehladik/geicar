project(radar_ros)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()
add_executable(radar_ros_node Main.cpp)
target_link_libraries(radar_ros_node PUBLIC radar_driver ${catkin_LIBRARIES})
