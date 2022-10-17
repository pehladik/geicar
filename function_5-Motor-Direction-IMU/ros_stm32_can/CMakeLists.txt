project(ros_stm32_can)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             stm32_ros_msgs)

catkin_package(CATKIN_DEPENDS stm32_ros_msgs)

add_executable(ros_stm32_can_node Main.cpp)
target_link_libraries(ros_stm32_can_node PUBLIC ${catkin_LIBRARIES} simply_can)
target_include_directories(ros_stm32_can_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(ros_stm32_can_node ${catkin_EXPORTED_TARGETS})