#include <ros/ros.h>
#include "radar_ros_msgs/frame.h"
#include "Radar.hpp"
#include "Config.hpp"

void print_usage_and_exit(std::string_view exe_name) {
	std::ostringstream ss;
	ss << "Usage: " << exe_name << " [-dump <file>] path\n";
	ss << "-- Options --\n";
	ss << "   -dump <file> : dump the messages to <file>\n";
	ss << "   path         : path to the dump file or to the USB converter\n";
	ROS_ERROR("%s", ss.str().c_str());
	std::exit(1);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ros_radar_node");

	// Parse arguments
	std::optional<std::string> path{};
	std::optional<std::string> dump_file_path{};

	for (int i = 1; i < argc; ++i) {
		std::string_view arg = argv[i];
		if (arg == "-dump") {
			i++;
			if (i >= argc || dump_file_path.has_value()) {
				print_usage_and_exit(argv[0]);
			}
			dump_file_path = argv[i];
		} else {
			if (path.has_value()) {
				print_usage_and_exit(argv[0]);
			}
			path = argv[i];
		}
	}
	if (!path.has_value()) print_usage_and_exit(argv[0]);

	const bool simulate = path->substr(path->size() - 4) == ".txt";

	std::unique_ptr<Radar> radar;
	try {
		if (simulate) {
			radar = std::make_unique<SimulatedRadar>(*path, dump_file_path);
		} else {
			radar = std::make_unique<RealRadar>(*path, dump_file_path);
		}
	} catch (const std::runtime_error &err) {
		ROS_ERROR("%s", err.what());
		exit(2);
	}

	ros::NodeHandle n;

	ros::Publisher objects_publisher = n.advertise<radar_ros_msgs::frame>("radar_frames", 1000);

	ros::Rate loop_rate(10);

	RadarConfiguration config{};
	config.outputType = OutputType::OBJECTS;
	radar->send_config(config);

	while (ros::ok()) {
		radar->process();

		if (radar->measure.has_value()) {
			auto &objects = radar->measure->objects;
			radar_ros_msgs::frame frame;
			frame.time = ros::Time{radar->measure->timestamp / 1000u, 1000 * (radar->measure->timestamp % 1000u)};

			for (auto &&object: objects) {
				frame.objects.emplace_back();
				frame.objects.back().id = object.id;
				frame.objects.back().distance_long = object.distance_long;
				frame.objects.back().distance_lat = object.distance_lat;
				frame.objects.back().velocity_long = object.relative_velocity_long;
				frame.objects.back().velocity_lat = object.relative_velocity_lat;
				frame.objects.back().radar_cross_section = object.radar_cross_section;
			}

			objects_publisher.publish(frame);

			ros::spinOnce();
		}

		loop_rate.sleep();
	}

	return 0;
}