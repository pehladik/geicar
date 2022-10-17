project(ros_dashboard)

find_package(catkin REQUIRED COMPONENTS
             rospy
             stm32_ros_msgs)

catkin_package(CATKIN_DEPENDS stm32_ros_msgs)

catkin_install_python(PROGRAMS scripts/ros_dashboard.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})