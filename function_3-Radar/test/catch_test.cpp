#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "util.hpp"

TEST_CASE("test_bitset_slice") {
	std::bitset<11> b{"10011001110"};
	REQUIRE(slice_bitset<2, 4>(b) == std::bitset<4>{"0011"});
}

TEST_CASE("test_bitset_concat") {
	std::bitset<3> b1{"100"};
	std::bitset<2> b2{"11"};
	std::bitset<4> b3{"0010"};
	REQUIRE(concat_bitsets(b1, b2, b3) == std::bitset<9>{"100110010"});
}
