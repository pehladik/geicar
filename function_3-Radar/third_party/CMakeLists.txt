add_subdirectory(piksel)

add_library(catch INTERFACE)
target_include_directories(catch INTERFACE catch)

add_library(simply_can STATIC IMPORTED GLOBAL)
set_target_properties(simply_can
                      PROPERTIES
                      IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/simply_can_usb_driver/libsimplyCAN.a
                      INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/simply_can_usb_driver/include)