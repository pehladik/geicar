#include <iostream>
#include "RadarVisualizer.hpp"

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
	std::optional<std::filesystem::path> dump_file_path{};

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

	RadarVisualizer app{*path, simulate, dump_file_path};
	app.start();
}