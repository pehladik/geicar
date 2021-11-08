#include <iostream>
#include "RadarVisualizer.hpp"

int main(int argc, char **argv) {
	if (argc != 2) {
		throw std::runtime_error{"One argument expected: path to the dump file or to the USB to CAN device"};
	}
	const std::string arg = argv[1];
	const bool simulate = arg.substr(arg.size() - 4) == ".txt";
	RadarVisualizer app{arg, simulate};
	app.start();

//	SimulatedRadar sim_radar{argv[1]};
//	std::unique_ptr<Message> msg;
//
//	while (true) {
//		try {
//			msg = sim_radar.receive();
//			if (msg == nullptr) break;
//			std::cout << *msg << '\n';
//			auto object_list = dynamic_cast<Object_list_status *>(msg.get());
//			if (object_list) {
//				std::cout << object_list->nofObjects.to_ulong() << std::endl;
//			}
//		} catch (const std::runtime_error &e) {
//			std::cout << e.what() << '\n';
//		}
//	}
}