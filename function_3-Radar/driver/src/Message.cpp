#include <stdexcept>
#include <sstream>
#include "Message.hpp"
#include "util.hpp"
#include <iostream>

namespace message {

	std::unique_ptr<MessageIn> MessageIn::parse(uint32_t timestamp, std::uint32_t id, const std::bitset<64> &payload) {
		std::unique_ptr<MessageIn> msg;
		switch (id) {
			case 0x60A:
				msg = std::make_unique<ObjectListStatus>(payload);
				break;
			case 0x60B:
				msg = std::make_unique<ObjectGeneralInfo>(payload);
				break;
			case 0x60C:
				msg = std::make_unique<ObjectQualityInfo>(payload);
				break;
			case 0x60D:
				msg = std::make_unique<ObjectExtInfo>(payload);
				break;
			case 0x600:
				msg = std::make_unique<ClusterListStatus>(payload);
				break;
			case 0x701:
				msg = std::make_unique<ClusterGeneralInfo>(payload);
				break;
			case 0x702:
				msg = std::make_unique<ClusterQualityInfo>(payload);
				break;
			case 0x201:
				msg = std::make_unique<RadarState>(payload);
				break;
			case 0x203:
				msg = std::make_unique<FilterStateHeader>(payload);
				break;
			case 0x204:
				msg = std::make_unique<FilterStateConfig>(payload);
				break;
		}
		if (msg != nullptr) {
			msg->timestamp = timestamp;
			return msg;
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
			interfaceVersion{reverse_slice<24, 4>(payload)} {}

	uint32_t ObjectListStatus::get_id() {
		return 0x60A;
	}

	void ObjectListStatus::print(std::ostream &os) const {
		os << "Object_list_status:\n";
		os << "  nofObjects: " << nofObjects.to_ulong() << '\n' <<
		   "  measCounter: " << measCounter.to_ulong() << '\n' <<
		   "  interfaceVersion: " << interfaceVersion.to_ulong() << '\n';
	}


	ObjectGeneralInfo::ObjectGeneralInfo(const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			distLong{reverse_slice<8, 13>(payload)},
			distLat{reverse_slice<21, 11>(payload)},
			vrelLong{reverse_slice<32, 10>(payload)},
			vrelLat{reverse_slice<42, 9>(payload)},
			dynProp{reverse_slice<53, 3>(payload)},
			rcs{reverse_slice<56, 8>(payload)} {}


	uint32_t ObjectGeneralInfo::get_id() {
		return 0x60B;
	}

	void ObjectGeneralInfo::print(std::ostream &os) const {
		os << "Object_general_info:\n";
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  distLong: " << distLong.to_ulong() << '\n' <<
		   "  distLat: " << distLat.to_ulong() << '\n' <<
		   "  vrelLong: " << vrelLong.to_ulong() << '\n' <<
		   "  vrelLat: " << vrelLat.to_ulong() << '\n' <<
		   "  dynProp: " << dynProp.to_ulong() << '\n' <<
		   "  rcs: " << rcs.to_ulong() << '\n';
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

	uint32_t ObjectQualityInfo::get_id() {
		return 0x60C;
	}

	void ObjectQualityInfo::print(std::ostream &os) const {
		os << "Object_quality_info:\n";
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

	ObjectExtInfo::ObjectExtInfo(const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			arelLong{reverse_slice<8, 11>(payload)},
			arelLat{reverse_slice<19, 9>(payload)},
			objectClass{reverse_slice<29, 3>(payload)},
			orientationAngle{reverse_slice<32, 10>(payload)},
			length{reverse_slice<48, 8>(payload)},
			width{reverse_slice<56, 8>(payload)} {}

	uint32_t ObjectExtInfo::get_id() {
		return 0x60D;
	}

	void ObjectExtInfo::print(std::ostream &os) const {
		os << "Object_ext_info:\n";
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  arelLong: " << arelLong.to_ulong() << '\n' <<
		   "  arelLat: " << arelLat.to_ulong() << '\n' <<
		   "  objectClass: " << objectClass.to_ulong() << '\n' <<
		   "  orientationAngle: " << orientationAngle.to_ulong() << '\n' <<
		   "  length: " << length.to_ulong() << '\n' <<
		   "  width: " << width.to_ulong() << '\n';
	}

	ClusterListStatus::ClusterListStatus(const std::bitset<64> &payload) :
			nofTargetsNear{reverse_slice<0, 8>(payload)},
			nofTargetsFar{reverse_slice<0, 8>(payload)},
			measCounter{reverse_slice<16, 16>(payload)},
			interfaceVersion{reverse_slice<32, 4>(payload)} {}

	uint32_t ClusterListStatus::get_id() {
		return 0x600;
	}

	void ClusterListStatus::print(std::ostream &os) const {
		os << "ClusterListStatus:\n";
		os << "  nofTargetsNear: " << nofTargetsNear.to_ulong() << '\n' <<
		   "  nofTargetsFar: " << nofTargetsFar.to_ulong() << '\n' <<
		   "  measCounter: " << measCounter.to_ulong() << '\n' <<
		   "  interfaceVersion: " << interfaceVersion.to_ulong() << '\n';
	}

	ClusterGeneralInfo::ClusterGeneralInfo(const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			distLong{reverse_slice<8, 13>(payload)},
			distLat{reverse_slice<22, 10>(payload)},
			vrelLong{reverse_slice<32, 10>(payload)},
			vrelLat{reverse_slice<42, 9>(payload)},
			dynProp{reverse_slice<53, 3>(payload)},
			rcs{reverse_slice<56, 8>(payload)} {}


	uint32_t ClusterGeneralInfo::get_id() {
		return 0x701;
	}

	void ClusterGeneralInfo::print(std::ostream &os) const {
		os << "ClusterGeneralInfo:\n";
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  distLong: " << distLong.to_ulong() << '\n' <<
		   "  distLat: " << distLat.to_ulong() << '\n' <<
		   "  vrelLong: " << vrelLong.to_ulong() << '\n' <<
		   "  vrelLat: " << vrelLat.to_ulong() << '\n' <<
		   "  dynProp: " << dynProp.to_ulong() << '\n' <<
		   "  rcs: " << rcs.to_ulong() << '\n';
	}


	ClusterQualityInfo::ClusterQualityInfo(const std::bitset<64> &payload) :
			id{reverse_slice<0, 8>(payload)},
			distLong_rms{reverse_slice<8, 5>(payload)},
			distLat_rms{reverse_slice<13, 5>(payload)},
			vrelLong_rms{reverse_slice<18, 5>(payload)},
			vrelLat_rms{reverse_slice<23, 5>(payload)},
			pdh0{reverse_slice<29, 3>(payload)},
			invalidState{reverse_slice<32, 5>(payload)},
			ambigState{reverse_slice<37, 3>(payload)} {}

	uint32_t ClusterQualityInfo::get_id() {
		return 0x702;
	}

	void ClusterQualityInfo::print(std::ostream &os) const {
		os << "ClusterQualityInfo:\n";
		os << "  id: " << id.to_ulong() << '\n' <<
		   "  distLong_rms: " << distLong_rms.to_ulong() << '\n' <<
		   "  distLat_rms: " << distLat_rms.to_ulong() << '\n' <<
		   "  vrelLong_rms: " << vrelLong_rms.to_ulong() << '\n' <<
		   "  vrelLat_rms: " << vrelLat_rms.to_ulong() << '\n' <<
		   "  pdh0: " << pdh0.to_ulong() << '\n' <<
		   "  invalidState: " << invalidState.to_ulong() << '\n' <<
		   "  ambigState: " << ambigState.to_ulong() << '\n';

	}

	uint32_t RadarConfig::get_id() {
		return 0x200;
	}

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

	std::bitset<64> FilterConfig::to_payload() const {
		return concat_bitsets(reverse(type),
		                      reverse(index),
		                      reverse(active),
		                      reverse(valid),
		                      std::bitset<5>{},
		                      reverse(minDistance),
		                      std::bitset<4>{},
		                      reverse(maxDistance),
		                      std::bitset<24>{});
	}

	uint32_t FilterConfig::get_id() {
		return 0x202;
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

	uint32_t RadarState::get_id() {
		return 0x201;
	}

	void RadarState::print(std::ostream &os) const {
		os << "Radar_State:\n";
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

	uint32_t SpeedInformation::get_id() {
		return 0x300;
	}

	std::bitset<64> SpeedInformation::to_payload() const {
		return concat_bitsets(reverse(speedDirection),
		                      std::bitset<1>{},
		                      reverse(speed),
		                      std::bitset<48>{});
	}

	void SpeedInformation::print(std::ostream &os) const {
		os << "SpeedInformation:\n";
		print_bitset64(os, to_payload());
		os << "  speedDirection: " << speedDirection.to_ulong() << '\n' <<
		   "  speed: " << speed.to_ulong() << '\n';
	}

	uint32_t YawRateInformation::get_id() {
		return 0x301;
	}

	std::bitset<64> YawRateInformation::to_payload() const {
		return concat_bitsets(reverse(yawRate), std::bitset<48>{});
	}

	void YawRateInformation::print(std::ostream &os) const {
		os << "YawRateInformation:\n";
		print_bitset64(os, to_payload());
		os << "  yawRate: " << yawRate.to_ulong() << '\n';
	}

	FilterStateHeader::FilterStateHeader(const std::bitset<64> &payload) :
			nOfClusterFilterCfg{reverse_slice<0, 5>(payload)},
			nOfObjectFilterCfg{reverse_slice<8, 5>(payload)} {}

	uint32_t FilterStateHeader::get_id() {
		return 0x203;
	}

	void FilterStateHeader::print(std::ostream &os) const {
		os << "YawRateInformation:\n";
		os << "  nOfClusterFilterCfg: " << nOfClusterFilterCfg.to_ulong() << '\n' <<
		   "  nOfObjectFilterCfg: " << nOfObjectFilterCfg.to_ulong() << '\n';
	}

	FilterStateConfig::FilterStateConfig(const std::bitset<64> &payload) :
			type{reverse_slice<0, 1>(payload)},
			index{reverse_slice<1, 4>(payload)},
			active{reverse_slice<5, 1>(payload)} {
		if (type.to_ulong() == 0xA) {
			min = reverse_slice<11, 13>(payload);
			max = reverse_slice<27, 13>(payload);
		} else {
			min = reverse_slice<12, 12>(payload);
			max = reverse_slice<28, 12>(payload);
		}
	}

	uint32_t FilterStateConfig::get_id() {
		return 0x204;
	}

	void FilterStateConfig::print(std::ostream &os) const {
		os << "FilterStateConfig:\n";
		os << "  type: " << type.to_ulong() << '\n' <<
		   "  index: " << index.to_ulong() << '\n' <<
		   "  active: " << active.to_ulong() << '\n';
		if (type.to_ullong() == 0xA) {
			os << "  min: " << std::get<1>(min).to_ulong() << '\n' <<
			   "  max: " << std::get<1>(max).to_ulong() << '\n';
		} else {
			os << "  min: " << std::get<0>(min).to_ulong() << '\n' <<
			   "  max: " << std::get<0>(max).to_ulong() << '\n';
		}
	}
}