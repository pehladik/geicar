#include <iostream>
#include <sstream>
#include "RadarVisualizer.hpp"
#include "Radar.hpp"

void print_usage_and_exit(std::string_view exe_name) {
	std::ostringstream ss;
	ss << "Usage: " << exe_name << " [-dump <file>] path\n";
	ss << "-- Options --\n";
	ss << "   -dump <file> : dump the messages to <file>\n";
	ss << "   path         : path to the dump file or to the USB converter\n";
	std::cout << ss.str();
	std::exit(1);
}


class RadarVisualizerImpl : public RadarVisualizer {
public:
	RadarVisualizerImpl(const std::string &path,
	                    const std::optional<std::string> &dump_file_path) : RadarVisualizer() {
		const bool simulate = path.substr(0, 4) != "/dev";
		if (simulate)
			radar = std::make_unique<SimulatedRadar>(path, dump_file_path);
		else
			radar = std::make_unique<RealRadar>(path, dump_file_path);
	}

protected:
	void process() override {
		radar->process();
	}

	std::optional<Measure> get_radar_measure() override {
		return radar->measure;
	}

	std::optional<float> get_ultrasonic_measure() override {
		return std::nullopt;
	}

	void send_config(const RadarConfiguration &configuration) override {
		radar->send_config(configuration);
	}

	std::optional<RadarState> get_state() override {
		return radar->state;
	}

	glm::vec2 get_warning_region_size() override {
		return warning_region_size;
	}

	void send_warning_region_size(glm::vec2 size) override {
		warning_region_size = size;
	}

private:
	std::unique_ptr<Radar> radar{};
	glm::vec2 warning_region_size{5, 2};

};


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

	RadarVisualizerImpl visualizer{path.value(), dump_file_path};
	visualizer.start();
}