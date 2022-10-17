#include "radar_msgs_translate.hpp"

Measure ros_msg_to_measure(const radar_ros_msgs::frame &frame) {
	Measure measure{};
	measure.timestamp = frame.time.toNSec() / 1000;
	for (auto &&object: frame.objects) {
		measure.objects.emplace_back();
		auto &o = measure.objects.back();
		o.distance_long = object.distance_long;
		o.distance_lat = object.distance_lat;
		o.relative_velocity_long = object.velocity_long;
		o.relative_velocity_lat = object.velocity_lat;
		o.radar_cross_section = object.radar_cross_section;
	}
	return measure;
}
