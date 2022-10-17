cmake_minimum_required(VERSION 3.10)
project(geiflix)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party)
add_subdirectory(function_3-Radar)

set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
find_package(catkin QUIET
             NO_POLICY_SCOPE
             PATHS ${catkin_search_path})
unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

if (NOT catkin_FOUND)
    message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
endif ()

catkin_workspace()