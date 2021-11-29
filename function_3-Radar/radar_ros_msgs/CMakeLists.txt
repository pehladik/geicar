project(radar_ros_msgs)

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

add_library(radar_msgs_translate src/radar_msgs_translate.cpp)
target_include_directories(radar_msgs_translate PUBLIC include)
target_link_libraries(radar_msgs_translate radar_msgs ${catkin_LIBRARIES})
add_dependencies(radar_msgs_translate ${${PROJECT_NAME}_EXPORTED_TARGETS})