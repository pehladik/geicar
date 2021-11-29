#include <map>
#include "Config.hpp"

using namespace message;

static std::map<OutputType, const char *> output_type_string{
		{OutputType::NONE,     "NONE"},
		{OutputType::OBJECTS,  "OBJECTS"},
		{OutputType::CLUSTERS, "CLUSTERS"},
};

std::ostream &operator<<(std::ostream &os, OutputType value) {
	os << output_type_string[value];
	return os;
}

static std::map<RadarPower, const char *> radar_power_string{
		{RadarPower::STANDARD,  "STANDARD"},
		{RadarPower::MINUS_3DB, "MINUS_3DB"},
		{RadarPower::MINUS_6DB, "MINUS_6DB"},
		{RadarPower::MINUS_9DB, "MINUS_9DB"},
};

std::ostream &operator<<(std::ostream &os, RadarPower value) {
	os << radar_power_string[value];
	return os;
}

static std::map<SortIndex, const char *> sort_index_string{
		{SortIndex::NO_SORTING, "NO_SORTING"},
		{SortIndex::RANGE,      "RANGE"},
		{SortIndex::RCS,        "RCS"},
};

std::ostream &operator<<(std::ostream &os, SortIndex value) {
	os << sort_index_string[value];
	return os;
}

static std::map<RcsThreshold, const char *> rcs_threshold_string{
		{RcsThreshold::STANDARD,         "STANDARD"},
		{RcsThreshold::HIGH_SENSITIVITY, "HIGH_SENSITIVITY"},
};

std::ostream &operator<<(std::ostream &os, RcsThreshold value) {
	os << rcs_threshold_string[value];
	return os;
}

static std::map<::RadarState::Status, const char *> status_string{
		{::RadarState::Status::FAIL,    "FAIL"},
		{::RadarState::Status::SUCCESS, "SUCCESS"},
};

std::ostream &operator<<(std::ostream &os, ::RadarState::Status value) {
	os << status_string[value];
	return os;
}

static std::map<::RadarState::MotionRxState, const char *> motion_rx_state_string{
		{::RadarState::MotionRxState::OK,                "OK"},
		{::RadarState::MotionRxState::SPEED_MISSING,     "SPEED_MISSING"},
		{::RadarState::MotionRxState::YAW_MISSING,       "YAW_MISSING"},
		{::RadarState::MotionRxState::SPEED_YAW_MISSING, "SPEED_YAW_MISSING"},
};

std::ostream &operator<<(std::ostream &os, ::RadarState::MotionRxState value) {
	os << motion_rx_state_string[value];
	return os;
}

RadarConfig RadarConfiguration::to_message() const {
	RadarConfig msg{};
	msg.sensorID_valid = sensorID.has_value();
	msg.sensorID = std::bitset<3>{sensorID.value_or(0)};
	msg.maxDistance_valid = maxDistance.has_value();
	msg.maxDistance = std::bitset<10>{static_cast<unsigned long long>(maxDistance.value_or(0) / MAX_DISTANCE_RES)};
	msg.radarPower_valid = radarPower.has_value();
	msg.radarPower = std::bitset<3>{static_cast<unsigned>(radarPower.value_or(RadarPower::STANDARD))};
	msg.outputType_valid = outputType.has_value();
	msg.outputType = std::bitset<2>{static_cast<unsigned>(outputType.value_or(OutputType::NONE))};
	msg.sendQuality_valid = sendQuality.has_value();
	msg.sendQuality = std::bitset<1>{sendQuality.value_or(0)};
	msg.sendExtInfo_valid = sendExtInfo.has_value();
	msg.sendExtInfo = std::bitset<1>{sendExtInfo.value_or(0)};
	msg.sortIndex_valid = sortIndex.has_value();
	msg.sortIndex = std::bitset<3>{static_cast<unsigned>(sortIndex.value_or(SortIndex::NO_SORTING))};
	msg.ctrlRelay_valid = ctrlRelay.has_value();
	msg.ctrlRelay = std::bitset<1>{ctrlRelay.value_or(0)};
	msg.rcsThreshold_valid = rcsThreshold.has_value();
	msg.rcsThreshold = std::bitset<3>{static_cast<unsigned>(rcsThreshold.value_or(RcsThreshold::STANDARD))};
	msg.storeInNVM_valid = storeInNVM.has_value();
	msg.storeInNVM = std::bitset<1>{storeInNVM.value_or(0)};
	return msg;
}

::RadarState::RadarState(const message::RadarState &status_msg) :
		NvmWriteStatus{static_cast<Status>(status_msg.NvmWriteStatus.to_ullong())},
		NvmReadStatus{static_cast<Status>(status_msg.NvmReadStatus.to_ullong())},
		maxDistanceCfg{static_cast<std::uint16_t>(status_msg.maxDistanceCfg.to_ullong() * MAX_DISTANCE_RES)},
		persistentError{static_cast<bool>(status_msg.persistentError.to_ullong())},
		interference{static_cast<bool>(status_msg.interference.to_ullong())},
		temperatureError{static_cast<bool>(status_msg.temperatureError.to_ullong())},
		temporaryError{static_cast<bool>(status_msg.temporaryError.to_ullong())},
		voltageError{static_cast<bool>(status_msg.voltageError.to_ullong())},
		radarPowerCfg{static_cast<RadarPower>(status_msg.radarPowerCfg.to_ullong())},
		sortIndex{static_cast<SortIndex>(status_msg.sortIndex.to_ullong())},
		sensorId{static_cast<std::uint8_t>(status_msg.sensorId.to_ullong())},
		motionRxState{static_cast<MotionRxState>(status_msg.motionRxState.to_ullong())},
		sendExtInfoCfg{static_cast<bool>(status_msg.sendExtInfoCfg.to_ullong())},
		sendQualityCfg{static_cast<bool>(status_msg.sendQualityCfg.to_ullong())},
		outputTypeCfg{static_cast<OutputType>(status_msg.outputTypeCfg.to_ullong())},
		controlRelayCfg{static_cast<bool>(status_msg.controlRelayCfg.to_ullong())},
		rcsThreshold{static_cast<RcsThreshold>(status_msg.rcsThreshold.to_ullong())} {}

std::ostream &operator<<(std::ostream &os, const ::RadarState &state) {
	os << "RadarState" << '\n' <<
	   "  NvmWriteStatus: " << state.NvmWriteStatus << '\n' <<
	   "  NvmReadStatus: " << state.NvmReadStatus << '\n' <<
	   "  maxDistanceCfg: " << state.maxDistanceCfg << '\n' <<
	   "  persistentError: " << state.persistentError << '\n' <<
	   "  interference: " << state.interference << '\n' <<
	   "  temperatureError: " << state.temperatureError << '\n' <<
	   "  temporaryError: " << state.temporaryError << '\n' <<
	   "  voltageError: " << state.voltageError << '\n' <<
	   "  radarPowerCfg: " << state.radarPowerCfg << '\n' <<
	   "  sortIndex: " << state.sortIndex << '\n' <<
	   "  sensorId: " << static_cast<unsigned>(state.sensorId) << '\n' <<
	   "  motionRxState: " << state.motionRxState << '\n' <<
	   "  sendExtInfoCfg: " << state.sendExtInfoCfg << '\n' <<
	   "  sendQualityCfg: " << state.sendQualityCfg << '\n' <<
	   "  outputTypeCfg: " << state.outputTypeCfg << '\n' <<
	   "  controlRelayCfg: " << state.controlRelayCfg << '\n' <<
	   "  rcsThreshold: " << state.rcsThreshold << '\n';
	return os;
}
