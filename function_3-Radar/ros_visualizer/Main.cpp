#include <ros/ros.h>
#include "radar_ros_msgs/frame.h"
#include "radar_msgs_translate.hpp"
#include <std_msgs/Float32.h>
#include "RadarVisualizer.hpp"

using radar_frame = radar_ros_msgs::frame;
using ultronic_measure = std_msgs::Float32;

class RosVisualizer : public RadarVisualizer {
protected:
	void process() override {
		ros::spinOnce();
	}

	std::optional<Measure> get_radar_measure() override {
		return radar_measure;
	}

	std::optional<float> get_ultrasonic_measure() override {
		return us_measure;
	}

	void send_config(const RadarConfiguration &configuration) override {}

	std::optional<RadarState> get_state() override {return {};}

	glm::vec2 get_warning_region_size() override {
		glm::vec2 size{};
		size.x = n.param("trigger_dist_long", 4.f);
		size.y = n.param("trigger_dist_lat", 2.f);
		return size;
	}

	void send_warning_region_size(glm::vec2 size) override {
		n.setParam("trigger_dist_long", size.x);
		n.setParam("trigger_dist_lat", size.y);
	}

	ros::NodeHandle n{};
	std::optional<Measure> radar_measure{};
	std::optional<float> us_measure{};
	ros::Subscriber sub_radar{n.subscribe<radar_frame>("radar_frames", 50,
	                                                   [this](const radar_frame::ConstPtr &frame) {
		                                                   radar_measure = ros_msg_to_measure(*frame);
	                                                   })};
	ros::Subscriber sub_us{n.subscribe<ultronic_measure>("stm32/ultrasound", 50,
	                                                     [this](const ultronic_measure::ConstPtr &measure) {
		                                                     us_measure = measure->data;
	                                                     })};
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");

	RosVisualizer visualizer{};
	visualizer.start();
}