#include <iostream>
#include "Radar.hpp"

int main() {
	Radar radar{"/dev/ttyACM1"};
	while (true) {
		try {
			auto msg = radar.receive();
			std::cout << *msg;
		} catch (const std::runtime_error &e) {
			std::cout << e.what() << '\n';
		}
	}
}