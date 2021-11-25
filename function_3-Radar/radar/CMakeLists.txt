project(radar)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             radar_messages
             )

catkin_package(CATKIN_DEPENDS radar_messages)

add_executable(radar_node Main.cpp)
target_link_libraries(radar_node PUBLIC radar_driver ${catkin_LIBRARIES})
target_include_directories(radar_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(radar_node ${catkin_EXPORTED_TARGETS})
