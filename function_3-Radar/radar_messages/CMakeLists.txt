project(radar_messages)

find_package(catkin REQUIRED COMPONENTS
             std_msgs
             message_generation
             )

add_message_files(
        FILES
        object.msg
        frame.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)
