project(stm32_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
             std_msgs
             message_generation)

add_service_files(
        FILES
        emergency_stop.srv
        brakes.srv
        motor.srv
        steering.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)

