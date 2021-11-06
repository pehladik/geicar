#ifndef GEIFLIX_MEASURE_HPP
#define GEIFLIX_MEASURE_HPP

#include <vector>
#include <optional>
#include "Message.hpp"


struct QualityInformation {
	static constexpr std::array<double, 8> PROB_OF_EXISTENCE = {0.00, 0.25, 0.5, 0.75, 0.90, 0.99, 0.999, 1.0};
};

struct ExtendedInformation {
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
	Object(const ObjectGeneralInfo &general_info);
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

	static constexpr double DIST_RES = 0.2;
	static constexpr double DIST_LONG_MIN = -500;
	static constexpr double DIST_LONG_MAX = 1138.2;
	static constexpr double DIST_LAT_MIN = -204.6;
	static constexpr double DIST_LAT_MAX = 204.8;
	static constexpr double VREL_RES = 0.25;
	static constexpr double VREL_LONG_MIN = -128.0;
	static constexpr double VREL_LAT_MIN = -64.0;
	static constexpr double RCS_RES = 0.5;
	static constexpr double RCS_MIN = -64.0;
};

struct Measure {
	Measure(const ObjectListStatus &object_list_status,
	        const std::vector<ObjectGeneralInfo> &general_info,
	        const std::vector<ObjectQualityInfo> &quality_info,
	        const std::vector<ObjectExtInfo> &extended_info);
	int counter;
	std::vector<Object> objects;
};

#endif //GEIFLIX_MEASURE_HPP