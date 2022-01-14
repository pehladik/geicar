project(stm32_can_receiver)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             stm32_ros_msgs)

catkin_package(CATKIN_DEPENDS stm32_ros_msgs)

add_executable(stm32_can_receiver_node Main.cpp)
target_link_libraries(stm32_can_receiver_node PUBLIC ${catkin_LIBRARIES} simply_can)
target_include_directories(stm32_can_receiver_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(stm32_can_receiver_node ${catkin_EXPORTED_TARGETS})