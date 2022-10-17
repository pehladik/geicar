add_executable(radar_test catch_test.cpp)
target_link_libraries(radar_test PRIVATE radar_driver catch)
