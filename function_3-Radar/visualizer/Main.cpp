#include <iostream>
#include "Radar.hpp"

int main(int argc, char **argv) {
//	RadarVisualizer app{"/dev/ttyACM1"};
//	app.start();
//	Radar radar{"/dev/ttyACM1"};
	if (argc != 2) {
		throw std::runtime_error{"One argument expected: path to the dump file"};
	}
	SimulatedRadar sim_radar{argv[1]};
	std::unique_ptr<Message> msg;
	msg = sim_radar.receive();

	while (msg != nullptr) {
		try {
			std::cout << *msg;
			auto object_list = dynamic_cast<Object_list_status *>(msg.get());
			if (object_list) {
				std::cout << object_list->nofObjects.to_ulong() << std::endl;
			}
			msg = sim_radar.receive();
		} catch (const std::runtime_error &e) {
			std::cout << e.what() << '\n';
		}
	}
}