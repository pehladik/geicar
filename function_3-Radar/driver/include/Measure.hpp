#ifndef GEIFLIX_MEASURE_HPP
#define GEIFLIX_MEASURE_HPP

#include <vector>
#include <optional>
#include <ostream>
#include "Message.hpp"


struct QualityInformation {
	QualityInformation(const message::ObjectQualityInfo &quality_info_msg);
	QualityInformation(const message::ClusterQualityInfo &quality_info_msg);
	static constexpr std::array<double, 8> PROB_OF_EXISTENCE = {0.00, 0.25, 0.5, 0.75, 0.90, 0.99, 0.999, 1.0};
	double probability_of_existence;
};

enum struct ObjectClass {
	point = 0x0,
	car = 0x1,
	truck = 0x2,
	pedestrian = 0x3,
	motorcycle = 0x4,
	bicycle = 0x5,
	wide = 0x6,
	reserved = 0x7
};

struct ExtendedInformation {
	ExtendedInformation(const message::ObjectExtInfo &extended_info_msg);
	unsigned id;
	ObjectClass object_class;

	static constexpr double AREL_RES = 0.01;
	static constexpr double AREL_LONG_MIN = -10.0;
	static constexpr double AREL_LAT_MIN = -2.5;
	static constexpr double ORIENTATION_ANGLE_MIN = -180.0;
	static constexpr double ORIENTATION_ANGLE_RES = 0.4;
	static constexpr double WIDTH_RES = 0.2;
	static constexpr double LENGTH_RES = 0.2;
};

struct CollisionDetectionWarning {

};

struct Object {
	Object(const message::ObjectGeneralInfo &general_info_msg,
	       const std::optional<message::ObjectQualityInfo> &quality_info_msg,
	       const std::optional<message::ObjectExtInfo> &extended_info_msg);
	Object(const message::ClusterGeneralInfo &general_info_msg,
	       const std::optional<message::ClusterQualityInfo> &quality_info_msg);
	enum struct DynamicProperty {
		MOVING = 0x0,
		STATIONARY = 0x1,
		ONCOMING = 0x2,
		STATIONARY_CANDIDATE = 0x3,
		UNKNOWN = 0x4,
		CROSSING_STATIONARY = 0x5,
		CROSSING_MOVING = 0x6,
		STOPPED = 0x7
	};
	unsigned id;
	double distance_long; // meters
	double distance_lat; // meters
	double relative_velocity_long; // m/s
	double relative_velocity_lat; // m/s
	DynamicProperty dynamic_property;
	double radar_cross_section; //dBm2

	std::optional<QualityInformation> quality_info;
	std::optional<ExtendedInformation> extended_info;
	std::optional<CollisionDetectionWarning> collision_detection_warning;
	friend std::ostream &operator<<(std::ostream &os, const Object &object);
	static constexpr double DIST_RES = 0.2;
	static constexpr double DIST_LONG_MIN = -500;
	static constexpr double DIST_LONG_MAX = 1138.2;
	static constexpr double DIST_LAT_MIN_OBJECTS = -204.6;
	static constexpr double DIST_LAT_MIN_CLUSTERS = -102.3;
	static constexpr double DIST_LAT_MAX = 204.8;
	static constexpr double VREL_RES = 0.25;
	static constexpr double VREL_LONG_MIN = -128.0;
	static constexpr double VREL_LAT_MIN = -64.0;
	static constexpr double RCS_RES = 0.5;
	static constexpr double RCS_MIN = -64.0;
};

struct Measure {
	Measure(const message::ObjectListStatus &list_status_msg,
	        const std::array<std::optional<message::ObjectGeneralInfo>, 256> &general_info_msg,
	        const std::array<std::optional<message::ObjectQualityInfo>, 256> &quality_info_msg,
	        const std::array<std::optional<message::ObjectExtInfo>, 256> &extended_info_msg);
	Measure(const message::ClusterListStatus &list_status_msg,
	        const std::array<std::optional<message::ClusterGeneralInfo>, 256> &general_info_msg,
	        const std::array<std::optional<message::ClusterQualityInfo>, 256> &quality_info_msg);
	unsigned counter;
	std::vector<Object> objects;
	friend std::ostream &operator<<(std::ostream &os, const Measure &measure);
};

#endif //GEIFLIX_MEASURE_HPP
