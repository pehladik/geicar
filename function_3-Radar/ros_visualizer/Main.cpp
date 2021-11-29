#include <ros/ros.h>
#include "radar_ros_msgs/frame.h"
#include "radar_msgs_translate.hpp"
#include "RadarVisualizer.hpp"

void process() {
	ros::spinOnce();
}

int main(int argc, char **argv) {
	Measure measure;

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	auto sub = n.subscribe<radar_ros_msgs::frame>("radar_frames", 1000,
	                                              [&measure](const radar_ros_msgs::frameConstPtr &frame) {
		                                              measure = ros_msg_to_measure(*frame);
	                                              });

	auto process_function = [] {process();};
	auto measure_getter = [&measure]() {return measure;};

	RadarVisualizer app{process_function, measure_getter};
	app.start();
}