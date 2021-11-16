#include <stdexcept>
#include <sstream>
#include "Message.hpp"
#include "util.hpp"
#include <iostream>

namespace message {

	std::unique_ptr<MessageBase> MessageBase::parse(std::uint32_t id, const std::bitset<64> &payload) {
		switch (id) {
			case 0x60A:
				return std::make_unique<ObjectListStatus>(payload);
			case 0x60B:
				return std::make_unique<ObjectGeneralInfo>(payload);
			case 0x60C:
				return std::make_unique<ObjectQualityInfo>(payload);
			case 0x60D:
				return std::make_unique<ObjectExtInfo>(payload);
			case 0x200:
				return std::make_unique<RadarConfig>(payload);
			case 0x201:
				return std::make_unique<RadarState>(payload);
			case 0x202:
				return std::make_unique<FilterConfig>(payload);
		}
		std::ostringstream ss{};
		ss << "Unknown message id: 0x" << std::hex << id;
		throw std::runtime_error{ss.str()};
	}

	std::ostream &operator<<(std::ostream &os, const MessageBase &msg) {
		msg.print(os);
		return os;
	}

	ObjectListStatus::ObjectListStatus(const std::bitset<64> &payload) :
			nofObjects{reverse_slice<0, 8>(payload)},
			measCounter{reverse_slice<8, 16>(payload)},
			interfaceVersion{reverse_slice<24, 4>(payload)} {
	}

	void ObjectListStatus::print(std::ostream &os) const {
		os << "Object_list_status:\n";
		print_bitset64(os, to_payload());
		os << "  nofObjects: " << nofObjects.to_ulong() << '\n' <<
		   "  measCounter: " << measCounter.to_ulong() << '\n' <<
		   "  interfaceVersion: " << interfaceVersion.to_ulong() << '\n';
	}


	std::bitset<64> ObjectListStatus::to_payload() const {
		return concat_bitsets(reverse(nofObjects),
		                      reverse(measCounter),
		                      reverse(interfaceVersion),
		                      std::bitset<36>{});
	}

	uint32_t ObjectListStatus::get_id() {
		return 0x60A;
	}


	ObjectGeneralInfo::ObjectGeneralInfo(const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			distLong{reverse_slice<8, 13>(payload)},
			distLat{reverse_slice<21, 11>(payload)},
			vrelLong{reverse_slice<32, 10>(payload)},
			vrelLat{reverse_slice<42, 9>(payload)},
			dynProp{reverse_slice<53, 3>(payload)},
			rcs{reverse_slice<56, 8>(payload)} {}


	void ObjectGeneralInfo::print(std::ostream &os) const {
		os << "Object_general_info:\n";
		print_bitset64(os, to_payload());
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  distLong: " << distLong.to_ulong() << '\n' <<
		   "  distLat: " << distLat.to_ulong() << '\n' <<
		   "  vrelLong: " << vrelLong.to_ulong() << '\n' <<
		   "  vrelLat: " << vrelLat.to_ulong() << '\n' <<
		   "  dynProp: " << dynProp.to_ulong() << '\n' <<
		   "  rcs: " << rcs.to_ulong() << '\n';
	}

	std::bitset<64> ObjectGeneralInfo::to_payload() const {
		return concat_bitsets(reverse(id),
		                      reverse(distLong),
		                      reverse(distLat),
		                      reverse(vrelLong),
		                      reverse(vrelLat),
		                      std::bitset<2>{},
		                      reverse(dynProp),
		                      reverse(rcs));
	}

	uint32_t ObjectGeneralInfo::get_id() {
		return 0x60B;
	}

	ObjectQualityInfo::ObjectQualityInfo(const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			distLong_rms{reverse_slice<8, 5>(payload)},
			distLat_rms{reverse_slice<13, 5>(payload)},
			vrelLong_rms{reverse_slice<18, 5>(payload)},
			vrelLat_rms{reverse_slice<23, 5>(payload)},
			arelLong_rms{reverse_slice<28, 5>(payload)},
			arelLat_rms{reverse_slice<33, 5>(payload)},
			orientation_rms{reverse_slice<38, 5>(payload)},
			probOfExist{reverse_slice<48, 3>(payload)},
			measState{reverse_slice<51, 3>(payload)} {}

	void ObjectQualityInfo::print(std::ostream &os) const {
		os << "Object_quality_info:\n";
		print_bitset64(os, to_payload());
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  distLong_rms: " << distLong_rms.to_ulong() << '\n' <<
		   "  distLat_rms: " << distLat_rms.to_ulong() << '\n' <<
		   "  vrelLong_rms: " << vrelLong_rms.to_ulong() << '\n' <<
		   "  vrelLat_rms: " << vrelLat_rms.to_ulong() << '\n' <<
		   "  arelLat_rms: " << arelLat_rms.to_ulong() << '\n' <<
		   "  arelLong_rms: " << arelLong_rms.to_ulong() << '\n' <<
		   "  orientation_rms: " << orientation_rms.to_ulong() << '\n' <<
		   "  probOfExist: " << probOfExist.to_ulong() << '\n';
	}

	std::bitset<64> ObjectQualityInfo::to_payload() const {
		return concat_bitsets(reverse(id),
		                      reverse(distLong_rms),
		                      reverse(distLat_rms),
		                      reverse(vrelLong_rms),
		                      reverse(vrelLat_rms),
		                      reverse(arelLong_rms),
		                      reverse(arelLat_rms),
		                      reverse(orientation_rms),
		                      std::bitset<5>{},
		                      reverse(probOfExist),
		                      reverse(measState),
		                      std::bitset<10>{});
	}

	uint32_t ObjectQualityInfo::get_id() {
		return 0x60C;
	}

	ObjectExtInfo::ObjectExtInfo(
			const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			arelLong{reverse_slice<8, 11>(payload)},
			arelLat{reverse_slice<19, 9>(payload)},
			objectClass{reverse_slice<29, 3>(payload)},
			orientationAngle{reverse_slice<32, 10>(payload)},
			length{reverse_slice<48, 8>(payload)},
			width{reverse_slice<56, 8>(payload)} {}

	void ObjectExtInfo::print(std::ostream &os) const {
		os << "Object_ext_info:\n";
		print_bitset64(os, to_payload());
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  arelLong: " << arelLong.to_ulong() << '\n' <<
		   "  arelLat: " << arelLat.to_ulong() << '\n' <<
		   "  objectClass: " << objectClass.to_ulong() << '\n' <<
		   "  orientationAngle: " << orientationAngle.to_ulong() << '\n' <<
		   "  length: " << length.to_ulong() << '\n' <<
		   "  width: " << width.to_ulong() << '\n';
	}

	std::bitset<64> ObjectExtInfo::to_payload() const {
		return concat_bitsets(reverse(id),
		                      reverse(arelLong),
		                      reverse(arelLat),
		                      std::bitset<1>{},
		                      reverse(objectClass),
		                      reverse(orientationAngle),
		                      std::bitset<6>{},
		                      reverse(length),
		                      reverse(width));
	}

	uint32_t ObjectExtInfo::get_id() {
		return 0x60D;
	}

	void RadarConfig::print(std::ostream &os) const {
		os << "Radar_config:\n";
		print_bitset64(os, to_payload());
		os << "  maxDistance_valid: " << maxDistance_valid.to_ulong() << '\n' <<
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

	RadarConfig::RadarConfig(
			const std::bitset<64> &payload) :
			storeInNVM_valid{reverse_slice<0, 1>(payload)},
			sortIndex_valid{reverse_slice<1, 1>(payload)},
			sendExtInfo_valid{reverse_slice<2, 1>(payload)},
			sendQuality_valid{reverse_slice<3, 1>(payload)},
			outputType_valid{reverse_slice<4, 1>(payload)},
			radarPower_valid{reverse_slice<5, 1>(payload)},
			sensorID_valid{reverse_slice<6, 1>(payload)},
			maxDistance_valid{reverse_slice<7, 1>(payload)},
			maxDistance{reverse_slice<8, 10>(payload)},
			radarPower{reverse_slice<32, 3>(payload)},
			outputType{reverse_slice<35, 2>(payload)},
			sensorID{reverse_slice<37, 3>(payload)},
			storeInNVM{reverse_slice<40, 1>(payload)},
			sortIndex{reverse_slice<41, 3>(payload)},
			sendExtInfo{reverse_slice<44, 1>(payload)},
			sendQuality{reverse_slice<45, 1>(payload)},
			ctrlRelay{reverse_slice<46, 1>(payload)},
			ctrlRelay_valid{reverse_slice<47, 1>(payload)},
			rcsThreshold{reverse_slice<52, 3>(payload)},
			rcsThreshold_valid{reverse_slice<55, 1>(payload)} {}

	std::bitset<64> RadarConfig::to_payload() const {
		return concat_bitsets(storeInNVM_valid,
		                      sortIndex_valid,
		                      sendExtInfo_valid,
		                      sendQuality_valid,
		                      outputType_valid,
		                      radarPower_valid,
		                      sensorID_valid,
		                      maxDistance_valid,
		                      reverse(maxDistance),
		                      std::bitset<14>{},
		                      reverse(radarPower),
		                      reverse(outputType),
		                      reverse(sensorID),
		                      storeInNVM,
		                      reverse(sortIndex),
		                      sendExtInfo,
		                      sendQuality,
		                      ctrlRelay,
		                      ctrlRelay_valid,
		                      std::bitset<4>{},
		                      reverse(rcsThreshold),
		                      rcsThreshold_valid,
		                      std::bitset<8>{});
	}

	uint32_t RadarConfig::get_id() {
		return 0x200;
	}

	FilterConfig::FilterConfig(const std::bitset<64> &payload) :
			type{reverse_slice<0, 1>(payload)},
			index{reverse_slice<1, 4>(payload)},
			active{reverse_slice<5, 1>(payload)},
			valid{reverse_slice<6, 1>(payload)},
			minDistance{reverse_slice<12, 12>(payload)},
			maxDistance{reverse_slice<28, 12>(payload)} {}

	std::bitset<64> FilterConfig::to_payload() const {
		return concat_bitsets(type,
		                      index,
		                      active,
		                      valid,
		                      std::bitset<5>{},
		                      reverse(minDistance),
		                      std::bitset<4>{},
		                      reverse(maxDistance),
		                      std::bitset<24>{});
	}

	void FilterConfig::print(std::ostream &os) const {
		os << "Filter_config:\n";
		print_bitset64(os, to_payload());
		os << "  type: " << type.to_ulong() << '\n' <<
		   "  index: " << index.to_ulong() << '\n' <<
		   "  active: " << active.to_ulong() << '\n' <<
		   "  valid: " << valid.to_ulong() << '\n' <<
		   "  min_distance: " << minDistance.to_ulong() << '\n' <<
		   "  max_distance: " << maxDistance.to_ulong() << '\n';
	}

	uint32_t FilterConfig::get_id() {
		return 0x202;
	}

	std::bitset<64> RadarState::to_payload() const {
		return concat_bitsets(
				NvmWriteStatus,
				NvmReadStatus,
				std::bitset<6>{},
				maxDistanceCfg,
				persistentError,
				interference,
				temperatureError,
				temporaryError,
				voltageError,
				std::bitset<7>{},
				radarPowerCfg,
				sortIndex,
				std::bitset<1>{},
				sensorId,
				motionRxState,
				sendExtInfoCfg,
				sendQualityCfg,
				outputTypeCfg,
				controlRelayCfg,
				std::bitset<12>{},
				rcsThreshold,
				std::bitset<2>{}
		);
	}

	RadarState::RadarState(const std::bitset<64> &payload) :
			NvmWriteStatus{reverse_slice<0, 1>(payload)},
			NvmReadStatus{reverse_slice<1, 1>(payload)},
			maxDistanceCfg{reverse_slice<8, 10>(payload)},
			persistentError{reverse_slice<18, 1>(payload)},
			interference{reverse_slice<19, 1>(payload)},
			temperatureError{reverse_slice<20, 1>(payload)},
			temporaryError{reverse_slice<21, 1>(payload)},
			voltageError{reverse_slice<22, 1>(payload)},
			radarPowerCfg{reverse_slice<30, 3>(payload)},
			sortIndex{reverse_slice<33, 3>(payload)},
			sensorId{reverse_slice<37, 3>(payload)},
			motionRxState{reverse_slice<40, 2>(payload)},
			sendExtInfoCfg{reverse_slice<42, 1>(payload)},
			sendQualityCfg{reverse_slice<43, 1>(payload)},
			outputTypeCfg{reverse_slice<44, 2>(payload)},
			controlRelayCfg{reverse_slice<46, 1>(payload)},
			rcsThreshold{reverse_slice<59, 3>(payload)} {}

	void RadarState::print(std::ostream &os) const {
		os << "Radar_State:\n";
		print_bitset64(os, to_payload());
		os << "  NvmWriteStatus: " << NvmWriteStatus.to_ulong() << '\n' <<
		   "  NvmReadStatus: " << NvmReadStatus.to_ulong() << '\n' <<
		   "  maxDistanceCfg: " << maxDistanceCfg.to_ulong() << '\n' <<
		   "  persistentError: " << persistentError.to_ulong() << '\n' <<
		   "  interference: " << interference.to_ulong() << '\n' <<
		   "  temperatureError: " << temperatureError.to_ulong() << '\n' <<
		   "  temporaryError: " << temporaryError.to_ulong() << '\n' <<
		   "  voltageError: " << voltageError.to_ulong() << '\n' <<
		   "  radarPowerCfg: " << radarPowerCfg.to_ulong() << '\n' <<
		   "  sortIndex: " << sortIndex.to_ulong() << '\n' <<
		   "  sensorId: " << sensorId.to_ulong() << '\n' <<
		   "  motionRxState: " << motionRxState.to_ulong() << '\n' <<
		   "  sendExtInfoCfg: " << sendExtInfoCfg.to_ulong() << '\n' <<
		   "  sendQualityCfg: " << sendQualityCfg.to_ulong() << '\n' <<
		   "  outputTypeCfg: " << outputTypeCfg.to_ulong() << '\n' <<
		   "  controlRelayCfg: " << controlRelayCfg.to_ulong() << '\n';

	}

	uint32_t RadarState::get_id() {
		return 0x201;
	}

}