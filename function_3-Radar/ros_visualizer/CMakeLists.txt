project(ros_visualizer)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             radar_ros_msgs)

catkin_package(CATKIN_DEPENDS radar_ros_msgs)

add_executable(ros_visualizer_node Main.cpp)
target_link_libraries(ros_visualizer_node radar_visualizer_lib radar_msgs_translate ${catkin_LIBRARIES})
target_include_directories(ros_visualizer_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(ros_visualizer_node ${catkin_EXPORTED_TARGETS})
