add_library(radar_driver
            src/Radar.cpp)
target_link_libraries(radar_driver PRIVATE simply_can)
target_link_libraries(radar_driver PUBLIC radar_msgs radar_util)
target_include_directories(radar_driver PUBLIC include)
