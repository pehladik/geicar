#include <iostream>
#include <optional>
#include <thread>
#include <Radar.hpp>
#include <sstream>

using namespace std::chrono_literals;

void print_usage_and_exit(std::string_view exe_name) {
	std::ostringstream ss;
	ss << "Usage: " << exe_name << " [-dump <file>] path\n";
	ss << "-- Options --\n";
	ss << "   -dump <file> : dump the messages to <file>\n";
	ss << "   path         : path to the dump file or to the USB converter\n";
	std::cout << ss.str();
	std::exit(1);
}

int main(int argc, char **argv) {
	// Parse arguments
	std::optional<std::string> path{};
	std::optional<std::string> dump_file_path{};

	for (int i = 1; i < argc; ++i) {
		std::string_view arg = argv[i];
		if (arg == "-dump") {
			i++;
			if (i >= argc || dump_file_path.has_value()) {
				print_usage_and_exit(argv[0]);
			}
			dump_file_path = argv[i];
		} else {
			if (path.has_value()) {
				print_usage_and_exit(argv[0]);
			}
			path = argv[i];
		}
	}
	if (!path.has_value()) print_usage_and_exit(argv[0]);

	const bool simulate = path->substr(path->size() - 4) == ".txt";

	std::unique_ptr<Radar> radar;
	if (simulate) {
		radar = std::make_unique<SimulatedRadar>(*path, dump_file_path);
	} else {
		radar = std::make_unique<RealRadar>(*path, dump_file_path);
	}

	std::ofstream out{"obstacles.txt"};

	for (int i = 0; i < 100; ++i) {
		radar->process();

		if (radar->state.has_value()) {
//			std::cout << radar->state.value() << '\n';
		}
		if (radar->measure.has_value()) {
//			std::cout << radar->measure.value() << '\n';
			for (auto &&object: radar->measure->objects) {
				out << object.radar_cross_section << ","
				    << object.distance_long << "," << object.distance_lat << "," <<
				    object.relative_velocity_long << "," << object.relative_velocity_lat << " ";
			}
			out << '\n';
		}

		std::this_thread::sleep_for(100ms);
	}
}
