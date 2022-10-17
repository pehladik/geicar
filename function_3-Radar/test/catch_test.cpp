#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include <iostream>
#include "util.hpp"
#include "Radar.hpp"

TEST_CASE("test_bitset_slice") {
	std::bitset<11> b{"10011101110"};
	REQUIRE(slice_bitset<2, 4>(b) == std::bitset<4>{"1011"});

	std::bitset<64> b2{"0000000000000000000000000000000001000000000000000000000001111111"};
	REQUIRE(slice_bitset<0, 8>(b2) == std::bitset<8>{"01111111"});
	REQUIRE(slice_bitset<8, 16>(b2) == std::bitset<16>{0});
	REQUIRE(slice_bitset<28, 4>(b2) == std::bitset<4>{"0100"});
}

TEST_CASE("test_bitset_concat") {
	std::bitset<3> b1{"100"};
	std::bitset<2> b2{"11"};
	std::bitset<4> b3{"0010"};
	REQUIRE(concat_bitsets(b1, b2, b3) == std::bitset<9>{"001011100"});
}

TEST_CASE("test_bitset_reverse") {
	std::bitset<3> b1{"100"};
	std::bitset<1> b2{"1"};
	std::bitset<20> b3{"00100110011110001011"};
	REQUIRE(reverse(b1) == std::bitset<3>{"001"});
	REQUIRE(reverse(b2) == std::bitset<1>{"1"});
	REQUIRE(reverse(b3) == std::bitset<20>{"11010001111001100100"});
}

TEST_CASE("test_parse_message") {
	SECTION("Object list status") {
		std::uint32_t id = 0x60A;
		std::uint8_t data[] = {0b00001110,
		                       0b00000000,
		                       0b00100011,
		                       0b00101111,
		                       0b11111111,
		                       0b11111111,
		                       0b11111111,
		                       0b11111111};

		auto msg = Radar::parse_message(0, id, data);
		auto obj_list_status = dynamic_cast<message::ObjectListStatus *>(msg.get());
		CAPTURE(*msg);
		REQUIRE(obj_list_status != nullptr);
		REQUIRE(obj_list_status->nofObjects.to_ulong() == 14);
		REQUIRE(obj_list_status->measCounter.to_ulong() == 35);
		REQUIRE(obj_list_status->interfaceVersion.to_ulong() == 2);
	}

	SECTION("Radar config object") {
		RadarConfiguration config;
		config.outputType = OutputType::OBJECTS;
		config.storeInNVM = true;
		config.sortIndex = SortIndex::RANGE;
		config.sendExtInfo = true;
		config.sendQuality = true;
		auto msg = config.to_message();
		CAPTURE(msg);
		REQUIRE(msg.outputType_valid.to_ulong() == 1);
		REQUIRE(msg.outputType.to_ulong() == 1);
		REQUIRE(msg.storeInNVM_valid.to_ulong() == 1);
		REQUIRE(msg.storeInNVM.to_ulong() == 1);
		REQUIRE(msg.sortIndex_valid.to_ulong() == 1);
		REQUIRE(msg.sortIndex.to_ulong() == 1);
		REQUIRE(msg.sendExtInfo_valid.to_ulong() == 1);
		REQUIRE(msg.sendExtInfo.to_ulong() == 1);
		REQUIRE(msg.sendQuality_valid.to_ulong() == 1);
		REQUIRE(msg.sendQuality.to_ulong() == 1);
		REQUIRE(msg.ctrlRelay_valid.to_ulong() == 0);
		REQUIRE(msg.maxDistance_valid.to_ulong() == 0);
		REQUIRE(msg.rcsThreshold_valid.to_ulong() == 0);
		REQUIRE(msg.sensorID_valid.to_ulong() == 0);
		CAPTURE(msg.to_payload());
		REQUIRE(msg.get_id() == 0x200);
		REQUIRE(reverse(msg.to_payload()) == std::bitset<64>{0xF8000000089C0000});
	}
}