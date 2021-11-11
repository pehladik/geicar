#include <iostream>
#include "Measure.hpp"

Measure::Measure(const ObjectListStatus &object_list_status,
                 const std::vector<ObjectGeneralInfo> &general_info,
                 const std::vector<ObjectQualityInfo> &quality_info,
                 const std::vector<ObjectExtInfo> &extended_info) :
		counter(object_list_status.measCounter.to_ulong()) {
//	std::cout << object_list_status;
//	unsigned nObjects = object_list_status.nofObjects.to_ulong();
	std::size_t nObjects = std::min(general_info.size(), std::min(extended_info.size(), quality_info.size()));
//	std::cout << nObjects << " objects detected...\n";
	for (unsigned i = 0; i < nObjects; ++i) {
		objects.emplace_back(general_info.at(i), extended_info.at(i), quality_info.at(i));
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

Object::Object(const ObjectGeneralInfo &general_info, const ObjectExtInfo &ext_info_message,
               const ObjectQualityInfo &qual_info_message) {
	id = general_info.id.to_ulong();
	distance_long = DIST_LONG_MIN + static_cast<double>(general_info.distLong.to_ulong()) * DIST_RES;
	distance_lat = DIST_LAT_MIN + static_cast<double>(general_info.distLat.to_ulong()) * DIST_RES;
	radar_cross_section = RCS_MIN + static_cast<double>(general_info.rcs.to_ulong()) * RCS_RES;

	extended_info = ExtendedInformation(ext_info_message);
	quality_info = QualityInformation(qual_info_message);
}

ExtendedInformation::ExtendedInformation(const ObjectExtInfo &ext_info) :
		id{static_cast<unsigned>(ext_info.id.to_ulong())},
		object_class{static_cast<ObjectClass>(ext_info.objectClass.to_ulong())} {}

QualityInformation::QualityInformation(const ObjectQualityInfo &qualityInfo) {
	int prob = static_cast<double>(qualityInfo.probOfExist.to_ulong());
	switch (prob) {
		case 0:
			prob = 0.;
			break;
		case 1:
			prob = 0.25;
			break;
		case 2:
			prob = 0.5;
			break;
		case 3:
			prob = 0.75;
			break;
		case 4:
			prob = 0.9;
			break;
		case 5:
			prob = 0.99;
			break;
		case 6:
			prob = 0.999;
			break;
		case 7:
			prob = 1;
			break;
		default:
			prob = 0.;

	}

}
