#include <stdexcept>
#include "Message.hpp"
#include "util.hpp"

std::unique_ptr<message> message::parse(std::uint32_t id, const std::bitset<64> &payload) {
	switch (id) {
		case 0x60A:
			return std::make_unique<object_list_status>(payload);
		case 0x60B:
			return std::make_unique<object_general_info>(payload);
		case 0x60C:
			return std::make_unique<object_quality_info>(payload);
		case 0x60D:
			return std::make_unique<object_ext_info>(payload);
	}
	throw std::runtime_error{"Unknown message id" + std::to_string(id)};
}

std::ostream &operator<<(std::ostream &os, const message &msg) {
	msg.print(os);
	return os;
}


object_list_status::object_list_status(const std::bitset<64> &payload) {
	nofObjects = slice_bitset<0, 8>(payload);
	measCounter = slice_bitset<16, 16>(payload);
	interfaceVersion = slice_bitset<28, 4>(payload);
}

void object_list_status::print(std::ostream &os) const {
	os << "object_list_status:\n" <<
	   "  nofObjects: " << nofObjects.to_ulong() << '\n' <<
	   "  measCounter: " << measCounter.to_ulong() << '\n' <<
	   "  interfaceVersion: " << interfaceVersion.to_ulong() << '\n';
}


std::bitset<64> object_list_status::to_payload() {
	return std::bitset<64>{nofObjects.to_string() +
	                       measCounter.to_string() +
	                       interfaceVersion.to_string()};
}


object_general_info::object_general_info(const std::bitset<64> &payload) {
	id = slice_bitset<0, 8>(payload);
	distLong = slice_bitset<8, 13>(payload);    //désordre
	distLat = slice_bitset<21, 11>(payload);    //désordre
	vrelLong = slice_bitset<32, 10>(payload);   //désordre
	dynProp = slice_bitset<42, 3>(payload);      //désordre
	vrelLat = slice_bitset<45, 9>(payload);     //désordre
	rcs = slice_bitset<56, 8>(payload);
}

void object_general_info::print(std::ostream &os) const {
	os << "object_general_info:\n" <<
	   "  id: " << id.to_ulong() << '\n' <<
	   "  distLong: " << distLong.to_ulong() << '\n' <<
	   "  distLat: " << distLat.to_ulong() << '\n' <<
	   "  vrelLong: " << vrelLong.to_ulong() << '\n' <<
	   "  vrelLat: " << vrelLat.to_ulong() << '\n' <<
	   "  dynProp: " << dynProp.to_ulong() << '\n' <<
	   "  rcs: " << rcs.to_ulong() << '\n';
}

std::bitset<64> object_general_info::to_payload() {
	return std::bitset<64>{id.to_string() +
	                       distLong.to_string() +
	                       distLat.to_string() +
	                       vrelLong.to_string() +
	                       dynProp.to_string() +
	                       vrelLat.to_string() +
	                       rcs.to_string()};
}

object_quality_info::object_quality_info(const std::bitset<64> &payload) {
	id = slice_bitset<0, 8>(payload);
	distLong_rms = slice_bitset<8, 5>(payload);
	distLat_rms = slice_bitset<13, 5>(payload);     //désordre
	vrelLong_rms = slice_bitset<18, 5>(payload);    //désordre
	vrelLat_rms = slice_bitset<23, 5>(payload);     //désordre
	arelLong_rms = slice_bitset<28, 5>(payload);    //désordre
	arelLat_rms = slice_bitset<33, 5>(payload);     //désordre
	orientation_rms = slice_bitset<38, 5>(payload); //désordre
	measState = slice_bitset<50, 3>(payload);
	probOfExist = slice_bitset<53, 3>(payload);
}

void object_quality_info::print(std::ostream &os) const {
	os << "object_quality_info:\n" <<
	   "  id: " << id.to_ulong() << '\n' <<
	   "  distLong_rms: " << distLong_rms.to_ulong() << '\n' <<
	   "  distLat_rms: " << distLat_rms.to_ulong() << '\n' <<
	   "  vrelLong_rms: " << vrelLong_rms.to_ulong() << '\n' <<
	   "  vrelLat_rms: " << vrelLat_rms.to_ulong() << '\n' <<
	   "  arelLat_rms: " << arelLat_rms.to_ulong() << '\n' <<
	   "  arelLong_rms: " << arelLong_rms.to_ulong() << '\n' <<
	   "  orientation_rms: " << orientation_rms.to_ulong() << '\n' <<
	   "  probOfExist: " << probOfExist.to_ulong() << '\n';
}

std::bitset<64> object_quality_info::to_payload() {
	return std::bitset<64>{id.to_string() +
	                       distLong_rms.to_string() +
	                       vrelLong_rms.to_string() +
	                       distLat_rms.to_string() +
	                       vrelLat_rms.to_string() +
	                       arelLat_rms.to_string() +
	                       arelLong_rms.to_string() +
	                       orientation_rms.to_string() +
	                       measState.to_string() +
	                       probOfExist.to_string()};
}

object_ext_info::object_ext_info(const std::bitset<64> &payload) {
	id = slice_bitset<0, 8>(payload);
	arelLong = slice_bitset<8, 11>(payload);
	arelLat = slice_bitset<19, 9>(payload);
	objectClass = slice_bitset<24, 3>(payload);
	orientationAngle = slice_bitset<32, 10>(payload);
	length = slice_bitset<48, 8>(payload);
	width = slice_bitset<56, 8>(payload);
}

void object_ext_info::print(std::ostream &os) const {
	os << "object_ext_info:\n" <<
	   "  id: " << id.to_ulong() << '\n' <<
	   "  arelLong: " << arelLong.to_ulong() << '\n' <<
	   "  arelLat: " << arelLat.to_ulong() << '\n' <<
	   "  objectClass: " << objectClass.to_ulong() << '\n' <<
	   "  orientationAngle: " << orientationAngle.to_ulong() << '\n' <<
	   "  length: " << length.to_ulong() << '\n' <<
	   "  width: " << width.to_ulong() << '\n';
}

std::bitset<64> object_ext_info::to_payload() {
	return std::bitset<64>{id.to_string() +
	                       arelLong.to_string() +
	                       objectClass.to_string() +
	                       arelLat.to_string() +
	                       orientationAngle.to_string() +
	                       length.to_string() +
	                       width.to_string()};
}

void radar_config::print(std::ostream &os) const {
	os << "object_ext_info:\n" <<
	   "  maxDistance_valid: " << maxDistance_valid.to_ulong() << '\n' <<
	   "  sensorID_valid: " << sensorID_valid.to_ulong() << '\n' <<
	   "  radarPower_valid: " << radarPower_valid.to_ulong() << '\n' <<
	   "  outputType_valid: " << outputType_valid.to_ulong() << '\n' <<
	   "  sendQuality_valid: " << sendQuality_valid.to_ulong() << '\n' <<
	   "  sendExtInfo_valid: " << sendExtInfo_valid.to_ulong() << '\n' <<
	   "  sortIndex_valid: " << sortIndex_valid.to_ulong() << '\n' <<
	   "  storeInNVM_valid: " << storeInNVM_valid.to_ulong() << '\n' <<
	   "  maxDistance: " << maxDistance.to_ulong() << '\n' <<
	   "  sensorID: " << sensorID.to_ulong() << '\n' <<
	   "  outputType: " << outputType.to_ulong() << '\n' <<
	   "  radarPower: " << radarPower.to_ulong() << '\n' <<
	   "  ctrlRelay_valid: " << ctrlRelay_valid.to_ulong() << '\n' <<
	   "  ctrlRelay: " << ctrlRelay.to_ulong() << '\n' <<
	   "  sendQuality: " << sendQuality.to_ulong() << '\n' <<
	   "  sendExtInfo: " << sendExtInfo.to_ulong() << '\n' <<
	   "  sortIndex: " << sortIndex.to_ulong() << '\n' <<
	   "  storeInNVM: " << storeInNVM.to_ulong() << '\n' <<
	   "  rcsThreshold_valid: " << rcsThreshold_valid.to_ulong() << '\n' <<
	   "  rcsThreshold: " << rcsThreshold.to_ulong() << '\n';
}

std::bitset<64> radar_config::to_payload() {
	return std::bitset<64>{maxDistance_valid.to_string() +
	                       sensorID_valid.to_string() +
	                       radarPower_valid.to_string() +
	                       outputType_valid.to_string() +
	                       sendQuality_valid.to_string() +
	                       sendExtInfo_valid.to_string() +
	                       sortIndex_valid.to_string() +
	                       storeInNVM_valid.to_string() +
	                       maxDistance.to_string() +
	                       sensorID.to_string() +
	                       outputType.to_string() +
	                       radarPower.to_string() +
	                       ctrlRelay_valid.to_string() +
	                       ctrlRelay.to_string() +
	                       sendQuality.to_string() +
	                       sendExtInfo.to_string() +
	                       sortIndex.to_string() +
	                       storeInNVM.to_string() +
	                       rcsThreshold_valid.to_string() +
	                       rcsThreshold.to_string()};
}
