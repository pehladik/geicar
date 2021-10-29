add_library(radar_driver
            driver/src/Message.cpp
            driver/src/Radar.cpp)
target_link_libraries(radar_driver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/simply_can_usb_driver/libsimplyCAN.a)
target_include_directories(radar_driver PRIVATE third_party/simply_can_usb_driver/include)
target_include_directories(radar_driver PUBLIC driver/include)

add_executable(radar_visualizer visualizer/Main.cpp)
target_link_libraries(radar_visualizer PUBLIC radar_driver)

add_executable(radar_test test/catch_test.cpp)
target_include_directories(radar_test PRIVATE driver/include)