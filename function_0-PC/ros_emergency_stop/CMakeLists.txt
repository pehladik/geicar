project(ros_emergency_stop)

find_package(catkin REQUIRED COMPONENTS
             rospy
             stm32_ros_msgs)

catkin_package(CATKIN_DEPENDS stm32_ros_msgs)

catkin_install_python(PROGRAMS scripts/ros_emergency_stop.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
