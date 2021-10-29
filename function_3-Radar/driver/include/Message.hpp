#ifndef GEIFLIX_MESSAGE_HPP
#define GEIFLIX_MESSAGE_HPP

#include <bitset>
#include <memory>
#include <ostream>

struct message {
	static std::unique_ptr<message> parse(std::uint32_t id, const std::bitset<64> &payload);
	virtual std::bitset<64> to_payload() = 0;
	virtual void print(std::ostream &os) const = 0;
	friend std::ostream &operator<<(std::ostream &os, const message &msg);
};

//0x60A : Object list status
struct object_list_status : message {
	explicit object_list_status(const std::bitset<64> &payload);
	std::bitset<64> to_payload() override;
	std::bitset<8> nofObjects;          //number of objects
	std::bitset<16> measCounter;        //measurement cycle counter
	std::bitset<4> interfaceVersion;    //objectf list CAN interface version
	void print(std::ostream &os) const override;
};

//0x60B : Object General information
struct object_general_info : message {
	explicit object_general_info(const std::bitset<64> &payload);
	std::bitset<64> to_payload() override;
	std::bitset<8> id;                  //Object ID
	std::bitset<13> distLong;           //Longitudinal coordinate (x)
	std::bitset<11> distLat;            //Lateral coordinate (y)
	std::bitset<10> vrelLong;           //Relative velocity in longitudinal direction (x)
	std::bitset<3> dynProp;             //Dynamic property of the object
	std::bitset<9> vrelLat;             //Relative velocity in lateral direction (y)
	std::bitset<8> rcs;                 //Radar cross-section
	void print(std::ostream &os) const override;
};

//0x60C : Object quality information
struct object_quality_info : message {
	explicit object_quality_info(const std::bitset<64> &payload);
	std::bitset<64> to_payload() override;
	std::bitset<8> id;                  //Object ID
	std::bitset<5> distLong_rms;        //Standard deviation of longitudinal distance
	std::bitset<5> vrelLong_rms;        //Standard deviation of longitudinal relative velocity
	std::bitset<5> distLat_rms;         //Standard deviation of lateral distance
	std::bitset<5> vrelLat_rms;         //Standard deviation of lateral relative velocity
	std::bitset<5> arelLat_rms;         //Standard deviation of lateral relative acceleration
	std::bitset<5> arelLong_rms;        //Standard deviation of longitudinal relative acceleration
	std::bitset<5> orientation_rms;     //Standard deviation of orientation angle
	std::bitset<3> measState;           //Measurement state (if the object is valid)
	std::bitset<3> probOfExist;         //Probability of existence
	void print(std::ostream &os) const override;
};

//0x60D : Object Extended Information
struct object_ext_info : message {
	explicit object_ext_info(const std::bitset<64> &payload);
	std::bitset<64> to_payload() override;
	std::bitset<8> id;                  //Object ID
	std::bitset<11> arelLong;           //Relative acceleration in longitudinal direction
	std::bitset<3> objectClass;         //Class of the object (car, pedestrian, bicycle...)
	std::bitset<9> arelLat;             //Relative acceleration in lateral direction
	std::bitset<10> orientationAngle;   //Orientation angle of the object
	std::bitset<8> length;              //Length of the tracked object
	std::bitset<8> width;               //Width of the tracked object
	void print(std::ostream &os) const override;
};

//0x200 : Radar Configuration
struct radar_config : message {
	explicit radar_config(const std::bitset<64> &payload);
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
	std::bitset<64> to_payload() override;
};

//0x202 : Object Filter Config
struct filter_config : message {
	explicit filter_config(const std::bitset<64> &payload);
	std::bitset<1> valid;
	std::bitset<1> active;
	std::bitset<4> index;
	std::bitset<1> type;
	std::bitset<12> min;
	std::bitset<12> max;
	void print(std::ostream &os) const override;
};

#endif //GEIFLIX_MESSAGE_HPP
