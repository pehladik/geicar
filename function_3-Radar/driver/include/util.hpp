#ifndef GEIFLIX_UTIL_HPP
#define GEIFLIX_UTIL_HPP

#include <bitset>
#include <cmath>

template<std::size_t first, std::size_t length, std::size_t N>
std::bitset<length> slice_bitset(const std::bitset<N> &src) {
	uint64_t mask = std::pow(2, length) - 1;
	auto ret = src.to_ullong() >> first & mask;
	return std::bitset<length>{ret};
}

template<std::size_t N>
std::bitset<N> concat_bitsets(const std::bitset<N> &bitset) {
	return bitset;
}

template<std::size_t N1, std::size_t... Ns>
std::bitset<N1 + (Ns + ...)> concat_bitsets(const std::bitset<N1> &a, const std::bitset<Ns> &...bs) {
	auto b = concat_bitsets(bs...);
	return (a.to_ullong() << b.size()) + b.to_ullong();
}


#endif //GEIFLIX_UTIL_HPP
