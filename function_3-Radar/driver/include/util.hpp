#ifndef GEIFLIX_UTIL_HPP
#define GEIFLIX_UTIL_HPP

#include <bitset>
#include <cmath>
#include <algorithm>

template<std::size_t N>
std::bitset<N> reverse(std::bitset<N> b) {
	for (std::size_t i = 0; i < N / 2; ++i) {
		bool t = b[i];
		b[i] = b[N - i - 1];
		b[N - i - 1] = t;
	}
	return b;
}

template<std::size_t first, std::size_t length, std::size_t N>
std::bitset<length> slice_bitset(const std::bitset<N> &src) {
	uint64_t mask = std::pow(2, length) - 1;
	auto ret = src.to_ullong() >> first & mask;
	return std::bitset<length>(ret);
}

template<std::size_t first, std::size_t length, std::size_t N>
std::bitset<length> reverse_slice(std::bitset<N> b) {
	return reverse(slice_bitset<first, length>(b));
}

template<std::size_t N>
std::bitset<N> concat_bitsets(const std::bitset<N> &bitset) {
	return bitset;
}

template<std::size_t N1, std::size_t... Ns>
std::bitset<N1 + (Ns + ...)> concat_bitsets(const std::bitset<N1> &a, const std::bitset<Ns> &...bs) {
	auto b = concat_bitsets(bs...);
	return (b.to_ullong() << a.size()) + a.to_ullong();
}

void print_bitset64(std::ostream &os, const std::bitset<64> &msg);

std::uint8_t reverse_byte(std::uint8_t x);

#endif //GEIFLIX_UTIL_HPP
