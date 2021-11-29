add_library(radar_msgs
            src/Message.cpp
            src/Measure.cpp
            src/Config.cpp)

target_include_directories(radar_msgs PUBLIC include)
target_link_libraries(radar_msgs radar_util)