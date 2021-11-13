#ifndef GEIFLIX_CONFIG_HPP
#define GEIFLIX_CONFIG_HPP

#include <optional>
#include <ostream>
#include "Message.hpp"

enum struct OutputType {
	NONE = 0,
	OBJECTS = 1,
	CLUSTERS = 2
};

enum struct RadarPower {
	STANDARD = 0,
	MINUS_3DB = 1,
	MINUS_6DB = 2,
	MINUS_9DB = 3
};

enum struct SortIndex {
	NO_SORTING = 0,
	RANGE = 1,
	RCS = 2,
};

enum struct RcsThreshold {
	STANDARD = 0,
	HIGH_SENSITIVITY = 1
};

struct RadarConfiguration {
	RadarConfiguration(const message::RadarConfig &config_msg);
	message::RadarConfig to_message() const;
	std::optional<std::uint8_t> sensorID;
	std::optional<std::uint16_t> maxDistance;
	std::optional<RadarPower> radarPower;
	std::optional<OutputType> outputType;
	std::optional<bool> sendQuality;
	std::optional<bool> sendExtInfo;
	std::optional<SortIndex> sortIndex;
	std::optional<bool> ctrlRelay;
	std::optional<RcsThreshold> rcsThreshold;
	std::optional<bool> storeInNVM;

	static constexpr int MAX_DISTANCE_RES = 2;
};

struct RadarState {
	RadarState(const message::RadarState &status_msg);

	enum struct Status {
		FAIL = 0,
		SUCCESS = 1
	};
	enum struct MotionRxState {
		OK,
		SPEED_MISSING,
		YAW_MISSING,
		SPEED_YAW_MISSING
	};

	Status NvmWriteStatus;
	Status NvmReadStatus;
	std::uint16_t maxDistanceCfg;
	bool persistentError;
	bool interference;
	bool temperatureError;
	bool temporaryError;
	bool voltageError;
	RadarPower radarPowerCfg;
	SortIndex sortIndex;
	std::uint8_t sensorId;
	MotionRxState motionRxState;
	bool sendExtInfoCfg;
	bool sendQualityCfg;
	OutputType outputTypeCfg;
	bool controlRelayCfg;
	RcsThreshold rcsThreshold;

	friend std::ostream &operator<<(std::ostream &os, const RadarState &state);
};

std::ostream &operator<<(std::ostream &os, OutputType value);
std::ostream &operator<<(std::ostream &os, RadarPower value);
std::ostream &operator<<(std::ostream &os, SortIndex value);
std::ostream &operator<<(std::ostream &os, RcsThreshold value);
std::ostream &operator<<(std::ostream &os, RadarState::Status value);
std::ostream &operator<<(std::ostream &os, RadarState::MotionRxState value);


#endif //GEIFLIX_CONFIG_HPP
