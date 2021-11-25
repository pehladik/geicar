#include <iostream>
#include "Measure.hpp"

using namespace message;

Measure::Measure(const ClusterListStatus &list_status_msg,
                 const std::array<std::optional<ClusterGeneralInfo>, 256> &general_info_msg,
                 const std::array<std::optional<ClusterQualityInfo>, 256> &quality_info_msg) :
		counter(list_status_msg.measCounter.to_ulong()),
		timestamp{list_status_msg.timestamp} {
	objects.reserve(std::min(250ul, list_status_msg.nofTargetsFar.to_ulong() + list_status_msg.nofTargetsNear.to_ulong()));
	for (unsigned i = 0; i < general_info_msg.size(); ++i) {
		if (general_info_msg[i].has_value()) {
			objects.emplace_back(*general_info_msg[i], quality_info_msg[i]);
		}
	}
}

Measure::Measure(const message::ObjectListStatus &list_status_msg,
                 const std::array<std::optional<ObjectGeneralInfo>, 256> &general_info_msg,
                 const std::array<std::optional<ObjectQualityInfo>, 256> &quality_info_msg,
                 const std::array<std::optional<ObjectExtInfo>, 256> &extended_info_msg) :
		counter(list_status_msg.measCounter.to_ulong()),
		timestamp{list_status_msg.timestamp} {
	objects.reserve(std::min(100ul, list_status_msg.nofObjects.to_ulong()));
	for (unsigned i = 0; i < general_info_msg.size(); ++i) {
		if (general_info_msg[i].has_value()) {
			objects.emplace_back(*general_info_msg[i], quality_info_msg[i], extended_info_msg[i]);
		}
	}
}

std::ostream &operator<<(std::ostream &os, const Measure &measure) {
	os << "Measure:\n" <<
	   "  counter: " << measure.counter << '\n' <<
	   "  objects:\n";
	for (auto &&obj: measure.objects) {
		os << "   -> " << obj << '\n';
	}
	return os;
}


Object::Object(const message::ObjectGeneralInfo &general_info_msg,
               const std::optional<message::ObjectQualityInfo> &quality_info_msg,
               const std::optional<message::ObjectExtInfo> &extended_info_msg) :
		extended_info{extended_info_msg},
		quality_info{quality_info_msg} {
	id = general_info_msg.id.to_ulong();
	distance_long = DIST_LONG_MIN + static_cast<double>(general_info_msg.distLong.to_ulong()) * DIST_RES;
	distance_lat = DIST_LAT_MIN_OBJECTS + static_cast<double>(general_info_msg.distLat.to_ulong()) * DIST_RES;
	radar_cross_section = RCS_MIN + static_cast<double>(general_info_msg.rcs.to_ulong()) * RCS_RES;
	relative_velocity_long = VREL_LONG_MIN + static_cast<double>(general_info_msg.vrelLong.to_ulong()) * VREL_RES;
	relative_velocity_lat = VREL_LAT_MIN + static_cast<double>(general_info_msg.vrelLat.to_ulong()) * VREL_RES;
}

std::ostream &operator<<(std::ostream &os, const Object &object) {
	os << "id: " << object.id <<
	   " distance_long: " << object.distance_long <<
	   " distance_lat: " << object.distance_lat;
	return os;
}

Object::Object(const ClusterGeneralInfo &general_info_msg,
               const std::optional<message::ClusterQualityInfo> &quality_info_msg) :
		quality_info{quality_info_msg} {
	id = general_info_msg.id.to_ulong();
	distance_long = DIST_LONG_MIN + static_cast<double>(general_info_msg.distLong.to_ulong()) * DIST_RES;
	distance_lat = DIST_LAT_MIN_CLUSTERS + static_cast<double>(general_info_msg.distLat.to_ulong()) * DIST_RES;
	radar_cross_section = RCS_MIN + static_cast<double>(general_info_msg.rcs.to_ulong()) * RCS_RES;
}

ExtendedInformation::ExtendedInformation(const ObjectExtInfo &extended_info_msg) :
		id{static_cast<unsigned>(extended_info_msg.id.to_ulong())},
		object_class{static_cast<ObjectClass>(extended_info_msg.objectClass.to_ulong())} {}

QualityInformation::QualityInformation(const ClusterQualityInfo &quality_info_msg) {
	unsigned prob = quality_info_msg.pdh0.to_ulong();
	probability_of_existence = PROB_OF_EXISTENCE[prob];
}

QualityInformation::QualityInformation(const ObjectQualityInfo &quality_info_msg) {
	unsigned prob = quality_info_msg.probOfExist.to_ulong();
	probability_of_existence = PROB_OF_EXISTENCE[prob];
}

