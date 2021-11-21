#ifndef GEIFLIX_MESSAGE_HPP
#define GEIFLIX_MESSAGE_HPP

#include <bitset>
#include <memory>
#include <ostream>
#include <variant>

namespace message {

	struct MessageBase {
		virtual std::uint32_t get_id() = 0;
		virtual void print(std::ostream &os) const = 0;
		friend std::ostream &operator<<(std::ostream &os, const MessageBase &msg);
		virtual ~MessageBase() = default;
	};

	struct MessageIn : public MessageBase {
		static std::unique_ptr<MessageIn> parse(std::uint32_t id, const std::bitset<64> &payload);
	};

	struct MessageOut : public MessageBase {
		virtual std::bitset<64> to_payload() const = 0;
	};

    // ------------------------------OBJECT LIST-----------------------------------------
    //0x60A : Object list status
	struct ObjectListStatus : public MessageIn {
		explicit ObjectListStatus(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> nofObjects;          //number of objects
		std::bitset<16> measCounter;        //measurement cycle counter
		std::bitset<4> interfaceVersion;    //object list CAN interface version
		void print(std::ostream &os) const override;
		virtual ~ObjectListStatus() = default;
	};

    //0x60B : Object General information
	struct ObjectGeneralInfo : public MessageIn {
		explicit ObjectGeneralInfo(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> id;                  //Object ID
		std::bitset<13> distLong;           //Longitudinal coordinate (x)
		std::bitset<11> distLat;            //Lateral coordinate (y)
		std::bitset<10> vrelLong;           //Relative velocity in longitudinal direction (x)
		std::bitset<3> dynProp;             //Dynamic property of the object
		std::bitset<9> vrelLat;             //Relative velocity in lateral direction (y)
		std::bitset<8> rcs;                 //Radar cross-section
		void print(std::ostream &os) const override;
		virtual ~ObjectGeneralInfo() = default;
	};

    //0x60C : Object quality information
	struct ObjectQualityInfo : public MessageIn {
		explicit ObjectQualityInfo(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> id;                  //Object ID
		std::bitset<5> distLong_rms;        //Standard deviation of longitudinal distance
		std::bitset<5> distLat_rms;         //Standard deviation of lateral distance
		std::bitset<5> vrelLong_rms;        //Standard deviation of longitudinal relative velocity
		std::bitset<5> vrelLat_rms;         //Standard deviation of lateral relative velocity
		std::bitset<5> arelLong_rms;        //Standard deviation of longitudinal relative acceleration
		std::bitset<5> arelLat_rms;         //Standard deviation of lateral relative acceleration
		std::bitset<5> orientation_rms;     //Standard deviation of orientation angle
		std::bitset<3> measState;           //Measurement state (if the object is valid)
		std::bitset<3> probOfExist;         //Probability of existence
		void print(std::ostream &os) const override;
		virtual ~ObjectQualityInfo() = default;
	};

    //0x60D : Object Extended Information
	struct ObjectExtInfo : public MessageIn {
		explicit ObjectExtInfo(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> id;                  //Object ID
		std::bitset<11> arelLong;           //Relative acceleration in longitudinal direction
		std::bitset<9> arelLat;             //Relative acceleration in lateral direction
		std::bitset<3> objectClass;         //Class of the object (car, pedestrian, bicycle...)
		std::bitset<10> orientationAngle;   //Orientation angle of the object
		std::bitset<8> length;              //Length of the tracked object
		std::bitset<8> width;               //Width of the tracked object
		void print(std::ostream &os) const override;
		virtual ~ObjectExtInfo() = default;
	};

    // ------------------------------CLUSTER LIST-----------------------------------------
    //0x600 : Cluster list status
	struct ClusterListStatus : public MessageIn {
		explicit ClusterListStatus(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> nofTargetsNear;
		std::bitset<8> nofTargetsFar;
		std::bitset<16> measCounter;
		std::bitset<4> interfaceVersion;
		void print(std::ostream &os) const override;
		virtual ~ClusterListStatus() = default;
	};

    //0x701 : Cluster General information
	struct ClusterGeneralInfo : public MessageIn {
		explicit ClusterGeneralInfo(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> id;
		std::bitset<13> distLong;
		std::bitset<10> distLat;
		std::bitset<10> vrelLong;
		std::bitset<9> vrelLat;
		std::bitset<3> dynProp;
		std::bitset<8> rcs;
		void print(std::ostream &os) const override;
		virtual ~ClusterGeneralInfo() = default;
	};

    //0x702 : Object quality information
	struct ClusterQualityInfo : public MessageIn {
		explicit ClusterQualityInfo(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<8> id;
		std::bitset<5> distLong_rms;
		std::bitset<5> distLat_rms;
		std::bitset<5> vrelLong_rms;
		std::bitset<5> vrelLat_rms;
		std::bitset<3> pdh0;
		std::bitset<5> invalidState;
		std::bitset<3> ambigState;
		void print(std::ostream &os) const override;
		virtual ~ClusterQualityInfo() = default;
	};

    // ------------------------------STATE OUTPUT-----------------------------------------
	//0x201 : Radar Configuration
	struct RadarState : public MessageIn {
		explicit RadarState(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<1> NvmWriteStatus;
		std::bitset<1> NvmReadStatus;
		std::bitset<10> maxDistanceCfg;
		std::bitset<1> persistentError;
		std::bitset<1> interference;
		std::bitset<1> temperatureError;
		std::bitset<1> temporaryError;
		std::bitset<1> voltageError;
		std::bitset<3> radarPowerCfg;
		std::bitset<3> sortIndex;
		std::bitset<3> sensorId;
		std::bitset<2> motionRxState;
		std::bitset<1> sendExtInfoCfg;
		std::bitset<1> sendQualityCfg;
		std::bitset<2> outputTypeCfg;
		std::bitset<1> controlRelayCfg;
		std::bitset<3> rcsThreshold;
		void print(std::ostream &os) const override;
		virtual ~RadarState() = default;
	};

	// 0x203 : Filter Configuration State Header
	struct FilterStateHeader : public MessageIn {
		explicit FilterStateHeader(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<5> nOfClusterFilterCfg;
		std::bitset<5> nOfObjectFilterCfg;
		void print(std::ostream &os) const override;
		virtual ~FilterStateHeader() = default;
	};

	// 0x204 : Filter Configuration State
	struct FilterStateConfig : public MessageIn {
		explicit FilterStateConfig(const std::bitset<64> &payload);
		uint32_t get_id() override;
		std::bitset<1> type;
		std::bitset<4> index;
		std::bitset<1> active;
		std::variant<std::bitset<12>, std::bitset<13>> min;
		std::variant<std::bitset<12>, std::bitset<13>> max;
		void print(std::ostream &os) const override;
		virtual ~FilterStateConfig() = default;
	};

	// 0x408 : Collision Detection State
	struct CollisionDetectionState : public MessageIn {
        explicit CollisionDetectionState(const std::bitset<64> &payload);
        uint32_t get_id() override;
        std::bitset<4> nOfRegions;
        std::bitset<1> active;
        std::bitset<8> minDetectTime;
        std::bitset<16> measCounter;
        void print(std::ostream &os) const override;
        virtual ~CollisionDetectionState() = default;
	};

	// 0x402 : Collision Detection Region State
	struct CollisionDetectionRegionState : public MessageIn {
        explicit CollisionDetectionRegionState(const std::bitset<64> &payload);
        uint32_t get_id() override;
        std::bitset<3> regionID;
        std::bitset<2> warningLevel;
        std::bitset<13> point1x;
        std::bitset<11> point1y;
        std::bitset<13> point2x;
        std::bitset<11> point2y;
        std::bitset<8> nOfObjects;
        void print(std::ostream &os) const override;
        virtual ~CollisionDetectionRegionState() = default;
	};

	// 0x700 : Software Version ID
	struct VersionId : public MessageIn {
        explicit VersionId(const std::bitset<64> &payload);
        uint32_t get_id() override;
        std::bitset<8> majorRelease;
        std::bitset<8> minorRelease;
        std::bitset<8> patchLevel;
        std::bitset<1> extendedRange;
        std::bitset<1> countryCode;
        void print(std::ostream &os) const override;
        virtual ~VersionId() = default;
	};

    // ------------------------------CONFIGURATION MESSAGES-----------------------------------------
	//0x200 : Radar Configuration
	struct RadarConfig : public MessageOut {
		RadarConfig() = default;
		uint32_t get_id() override;
		std::bitset<64> to_payload() const override;
		std::bitset<1> maxDistance_valid;
		std::bitset<1> sensorID_valid;
		std::bitset<1> radarPower_valid;
		std::bitset<1> outputType_valid;
		std::bitset<1> sendQuality_valid;
		std::bitset<1> sendExtInfo_valid;
		std::bitset<1> sortIndex_valid;
		std::bitset<1> storeInNVM_valid;
		std::bitset<10> maxDistance;
		std::bitset<3> sensorID;
		std::bitset<2> outputType;
		std::bitset<3> radarPower;
		std::bitset<1> ctrlRelay_valid;
		std::bitset<1> ctrlRelay;
		std::bitset<1> sendQuality;
		std::bitset<1> sendExtInfo;
		std::bitset<3> sortIndex;
		std::bitset<1> storeInNVM;
		std::bitset<1> rcsThreshold_valid;
		std::bitset<3> rcsThreshold;
		void print(std::ostream &os) const override;
		virtual ~RadarConfig() = default;
	};

    //0x202 : Cluster and Object Filter Config
	struct FilterConfig : public MessageOut {
		uint32_t get_id() override;
		std::bitset<64> to_payload() const override;
		std::bitset<1> valid;
		std::bitset<1> active;
		std::bitset<4> index;
		std::bitset<1> type;
		std::bitset<12> minDistance;
		std::bitset<12> maxDistance;
		void print(std::ostream &os) const override;
		virtual ~FilterConfig() = default;
	};

    //0x400 : Collision Detection Configuration
    struct CollisionDetectionCfg : public MessageOut {
        uint32_t get_id() override;
        std::bitset<64> to_payload() const override;
        std::bitset<1> warningReset;
        std::bitset<1> activate;
        std::bitset<1> minTimeValid;
        std::bitset<1> clearRegions;
        std::bitset<8> minDetectTime;
        void print(std::ostream &os) const override;
        virtual ~CollisionDetectionCfg() = default;
    };

    //0x401 : Collision Detection Region Configuration
    struct CollisionDetectionRegionCfg : public MessageOut {
        uint32_t get_id() override;
        std::bitset<64> to_payload() const override;
        std::bitset<1> activate;
        std::bitset<1> coordinatesValid;
        std::bitset<3> regionId;
        std::bitset<13> point1x;
        std::bitset<11> point1y;
        std::bitset<13> point2x;
        std::bitset<11> point2y;
        void print(std::ostream &os) const override;
        virtual ~CollisionDetectionRegionCfg() = default;
    };

    // ---------------------------MOTION INPUT SIGNALS-----------------------------------------
	//0x300 : Speed Information
	struct SpeedInformation : public MessageOut {
		SpeedInformation() = default;
		uint32_t get_id() override;
		std::bitset<64> to_payload() const override;
		std::bitset<2> speedDirection;
		std::bitset<13> speed;
		void print(std::ostream &os) const override;
		virtual ~SpeedInformation() = default;
	};

	//0x301 : Yaw Rate Information
	struct YawRateInformation : public MessageOut {
		YawRateInformation() = default;
		uint32_t get_id() override;
		std::bitset<64> to_payload() const override;
		std::bitset<16> yawRate;
		void print(std::ostream &os) const override;
		virtual ~YawRateInformation() = default;
	};

}

#endif //GEIFLIX_MESSAGE_HPP
