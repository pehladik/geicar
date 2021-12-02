#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Radar.hpp"
#include "Config.hpp"

void print_usage_and_exit(std::string_view exe_name) {
	std::ostringstream ss;
	ss << "Usage: " << exe_name << " [-dump <file>] path\n";
	ss << "-- Options --\n";
	ss << "   -dump <file> : dump the messages to <file>\n";
	ss << "   path         : path to the dump file or to the USB converter\n";
	std::cout << ss.str();
	std::exit(1);
}

bool is_obstacle_dangerous(Object obj) {
	return obj.distance_long < 2 &&
			obj.distance_lat < 1 &&
			obj.distance_lat > -1;
}

int main(int argc, char **argv) {
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
	if (simulate) {
		radar = std::make_unique<SimulatedRadar>(*path, dump_file_path);
	} else {
		radar = std::make_unique<RealRadar>(*path, dump_file_path);
	}

	ros::init(argc, argv, "obstacle_detection_node");

	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<std_msgs::String>("obstacle_ahead", 1000);

	ros::Rate loop_rate(10);

	RadarConfiguration config{};
	config.outputType = OutputType::OBJECTS;
	radar->send_config(config);

	while (ros::ok()) {
		radar->process();

		if (radar->measure.has_value()) {
			auto &objects = radar->measure->objects;
			bool danger = std::any_of(objects.begin(), objects.end(), is_obstacle_dangerous);

			std_msgs::String msg;

			std::stringstream ss;
			ss << danger;
			msg.data = ss.str();

			ROS_INFO("Publishing %s", msg.data.c_str());

			publisher.publish(msg);

			ros::spinOnce();
		}

		loop_rate.sleep();
	}


	return 0;

}