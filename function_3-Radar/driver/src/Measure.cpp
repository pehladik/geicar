#include <iostream>
#include "Measure.hpp"

Measure::Measure(const ObjectListStatus &object_list_status,
                 const std::vector<ObjectGeneralInfo> &general_info,
                 const std::vector<ObjectQualityInfo> &quality_info,
                 const std::vector<ObjectExtInfo> &extended_info) {
//	std::cout << object_list_status;
//	unsigned nObjects = object_list_status.nofObjects.to_ulong();
	std::size_t nObjects = std::min(general_info.size(), extended_info.size());
//	std::cout << nObjects << " objects detected...\n";
	for (unsigned i = 0; i < nObjects; ++i) {
		objects.emplace_back(general_info.at(i), extended_info.at(i));
	}
}

Object::Object(const ObjectGeneralInfo &general_info) :
		id{static_cast<unsigned int>(general_info.id.to_ulong())},
		distance_long{DIST_LONG_MIN + static_cast<double>(general_info.distLong.to_ulong()) * DIST_RES},
		distance_lat{DIST_LAT_MIN + static_cast<double>(general_info.distLat.to_ulong()) * DIST_RES},
		radar_cross_section{RCS_MIN + static_cast<double>(general_info.rcs.to_ulong()) * RCS_RES} {}

Object::Object(const ObjectGeneralInfo &general_info, const ObjectExtInfo &ext_info_message) {
	id = general_info.id.to_ulong();
	distance_long = DIST_LONG_MIN + static_cast<double>(general_info.distLong.to_ulong()) * DIST_RES;
	distance_lat = DIST_LAT_MIN + static_cast<double>(general_info.distLat.to_ulong()) * DIST_RES;
	radar_cross_section = RCS_MIN + static_cast<double>(general_info.rcs.to_ulong()) * RCS_RES;

	extended_info = ExtendedInformation(ext_info_message);
}

ExtendedInformation::ExtendedInformation(const ObjectExtInfo &ext_info) :
		id{static_cast<unsigned>(ext_info.id.to_ulong())},
		object_class{static_cast<ObjectClass>(ext_info.objectClass.to_ulong())} {}
