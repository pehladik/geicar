add_executable(radar_test catch_test.cpp)
target_link_libraries(radar_test PRIVATE radar_driver)
target_include_directories(radar_test PRIVATE ../third_party/catch)