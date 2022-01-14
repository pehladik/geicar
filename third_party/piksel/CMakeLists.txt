cmake_minimum_required(VERSION 3.12)
project(piksel)
set(CMAKE_CXX_STANDARD 11)

set(MSDFGEN_SOURCES
    lib/msdfgen/msdfgen/core/Bitmap.cpp
    lib/msdfgen/msdfgen/core/Contour.cpp
    lib/msdfgen/msdfgen/core/edge-coloring.cpp
    lib/msdfgen/msdfgen/core/edge-segments.cpp
    lib/msdfgen/msdfgen/core/EdgeHolder.cpp
    lib/msdfgen/msdfgen/core/equation-solver.cpp
    lib/msdfgen/msdfgen/core/msdfgen.cpp
    lib/msdfgen/msdfgen/core/render-sdf.cpp
    lib/msdfgen/msdfgen/core/save-bmp.cpp
    lib/msdfgen/msdfgen/core/shape-description.cpp
    lib/msdfgen/msdfgen/core/Shape.cpp
    lib/msdfgen/msdfgen/core/SignedDistance.cpp
    lib/msdfgen/msdfgen/core/Vector2.cpp
    )

add_library(msdfgen STATIC ${MSDFGEN_SOURCES})
set_property(TARGET msdfgen PROPERTY POSITION_INDEPENDENT_CODE TRUE)

set(BASE64_SOURCES
    lib/cpp-base64/cpp-base64/base64.cpp
    )

add_library(cpp-base64 STATIC ${BASE64_SOURCES})
set_property(TARGET cpp-base64 PROPERTY POSITION_INDEPENDENT_CODE TRUE)

set(JSON11_SOURCES
    lib/json11/json11/json11.cpp
    )

add_library(json11 STATIC ${JSON11_SOURCES})
set_property(TARGET json11 PROPERTY POSITION_INDEPENDENT_CODE TRUE)

set(SOURCES
    src/piksel/baseapp.cpp
    src/piksel/font.cpp
    src/piksel/graphics.cpp
    src/piksel/image.cpp
    src/piksel/shader.cpp
    src/piksel/shape.cpp
    src/piksel/ext/rng.cpp
    src/piksel/ext/sprite.cpp
    src/piksel/ext/spritesheet.cpp
    src/piksel/ext/animation.cpp
    src/piksel/ext/timer.cpp
    )
add_library(piksel ${SOURCES})
set_property(TARGET piksel PROPERTY POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(piksel PUBLIC
                           src/piksel
                           lib/glm
                           lib/stb/include
                           lib/msdfgen
                           lib/glad/include
                           lib/cpp-base64
                           lib/json11
                           )

if (EMSCRIPTEN)
    target_link_libraries(piksel
                          msdfgen
                          cpp-base64
                          json11
                          )
else ()
    add_library(glad STATIC lib/glad/src/glad.c)
    set_property(TARGET glad PROPERTY POSITION_INDEPENDENT_CODE TRUE)
    target_include_directories(glad PRIVATE lib/glad/include)
    target_link_libraries(glad ${CMAKE_DL_LIBS})

    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory(lib/glfw)

    target_link_libraries(piksel
                          glad
                          glfw
                          msdfgen
                          cpp-base64
                          json11
                          )
endif (EMSCRIPTEN)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/shell.html ${CMAKE_BINARY_DIR}/shell.html COPYONLY)

set(DATA_SOURCE_DIR ${CMAKE_SOURCE_DIR}/data)
set(DATA_BINARY_DIR ${CMAKE_BINARY_DIR}/data)
add_custom_target(make_data_directory ALL
                  COMMAND ${CMAKE_COMMAND} -E make_directory ${DATA_SOURCE_DIR}
                  )
add_custom_target(touch_dummy ALL
                  COMMAND ${CMAKE_COMMAND} -E touch ${DATA_SOURCE_DIR}/dummy
                  DEPENDS make_data_directory
                  )
add_custom_target(copy_data_directory ALL
                  COMMAND ${CMAKE_COMMAND} -E copy_directory ${DATA_SOURCE_DIR} ${DATA_BINARY_DIR}
                  DEPENDS touch_dummy
                  )
add_dependencies(piksel copy_data_directory)
