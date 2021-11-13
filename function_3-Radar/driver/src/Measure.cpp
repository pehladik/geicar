#include <iostream>
#include "Measure.hpp"

using namespace message;

Measure::Measure(const ObjectListStatus &list_status_msg,
                 const std::vector<ObjectGeneralInfo> &general_info_msg,
                 const std::vector<ObjectQualityInfo> &quality_info_msg,
                 const std::vector<ObjectExtInfo> &extended_info_msg) :
		counter(list_status_msg.measCounter.to_ulong()) {
//	std::cout << object_list_status;
//	unsigned nObjects = object_list_status.nofObjects.to_ulong();
	std::size_t nObjects = std::min(general_info_msg.size(), std::min(extended_info_msg.size(), quality_info_msg.size()));
//	std::cout << nObjects << " objects detected...\n";
	for (unsigned i = 0; i < nObjects; ++i) {
		objects.emplace_back(general_info_msg.at(i), extended_info_msg.at(i), quality_info_msg.at(i));
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

Object::Object(const ObjectGeneralInfo &general_info_msg) :
		id{static_cast<unsigned int>(general_info_msg.id.to_ulong())},
		distance_long{DIST_LONG_MIN + static_cast<double>(general_info_msg.distLong.to_ulong()) * DIST_RES},
		distance_lat{DIST_LAT_MIN + static_cast<double>(general_info_msg.distLat.to_ulong()) * DIST_RES},
		radar_cross_section{RCS_MIN + static_cast<double>(general_info_msg.rcs.to_ulong()) * RCS_RES} {}

Object::Object(const ObjectGeneralInfo &general_info_msg, const ObjectExtInfo &extended_info_msg) {
	id = general_info_msg.id.to_ulong();
	distance_long = DIST_LONG_MIN + static_cast<double>(general_info_msg.distLong.to_ulong()) * DIST_RES;
	distance_lat = DIST_LAT_MIN + static_cast<double>(general_info_msg.distLat.to_ulong()) * DIST_RES;
	radar_cross_section = RCS_MIN + static_cast<double>(general_info_msg.rcs.to_ulong()) * RCS_RES;

	extended_info = ExtendedInformation(extended_info_msg);
}

Object::Object(const ObjectGeneralInfo &general_info_msg, const ObjectExtInfo &extended_info_msg,
               const ObjectQualityInfo &quality_info_msg) {
	id = general_info_msg.id.to_ulong();
	distance_long = DIST_LONG_MIN + static_cast<double>(general_info_msg.distLong.to_ulong()) * DIST_RES;
	distance_lat = DIST_LAT_MIN + static_cast<double>(general_info_msg.distLat.to_ulong()) * DIST_RES;
	radar_cross_section = RCS_MIN + static_cast<double>(general_info_msg.rcs.to_ulong()) * RCS_RES;

	extended_info = ExtendedInformation(extended_info_msg);
	quality_info = QualityInformation(quality_info_msg);
}

std::ostream &operator<<(std::ostream &os, const Object &object) {
	os << "id: " << object.id <<
	   " distance_long: " << object.distance_long <<
	   " distance_lat: " << object.distance_lat;
	return os;
}

ExtendedInformation::ExtendedInformation(const ObjectExtInfo &extended_info_msg) :
		id{static_cast<unsigned>(extended_info_msg.id.to_ulong())},
		object_class{static_cast<ObjectClass>(extended_info_msg.objectClass.to_ulong())} {}

QualityInformation::QualityInformation(const ObjectQualityInfo &quality_info_msg) {
	unsigned prob = quality_info_msg.probOfExist.to_ulong();
	switch (prob) {
		case 0:
			probability_of_existence = 0.;
			break;
		case 1:
			probability_of_existence = 0.25;
			break;
		case 2:
			probability_of_existence = 0.5;
			break;
		case 3:
			probability_of_existence = 0.75;
			break;
		case 4:
			probability_of_existence = 0.9;
			break;
		case 5:
			probability_of_existence = 0.99;
			break;
		case 6:
			probability_of_existence = 0.999;
			break;
		case 7:
			probability_of_existence = 1;
			break;
		default:
			probability_of_existence = 0.;

	}

}
