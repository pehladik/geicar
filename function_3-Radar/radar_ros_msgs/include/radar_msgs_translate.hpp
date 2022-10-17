#ifndef GEIFLIX_RADAR_MSGS_TRANSLATE_HPP
#define GEIFLIX_RADAR_MSGS_TRANSLATE_HPP

#include "Measure.hpp"
#include "radar_ros_msgs/frame.h"

Measure ros_msg_to_measure(const radar_ros_msgs::frame &frame);

#endif //GEIFLIX_RADAR_MSGS_TRANSLATE_HPP
