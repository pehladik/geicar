#include <iostream>
#include "Radar.hpp"

int main() {
	Radar radar{"/dev/ttyACM1"};
	while (true) {
		try {
			auto msg = radar.receive();
			std::cout << *msg;
			auto object_list = dynamic_cast<Object_list_status *>(msg.get());
			if (object_list) {
				std::cout << object_list->nofObjects.to_ulong() << std::endl;
			}
		} catch (const std::runtime_error &e) {
			std::cout << e.what() << '\n';
		}
	}
}