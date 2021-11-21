#include <iostream>
#include <sstream>
#include "RadarVisualizer.hpp"

RadarVisualizer::RadarVisualizer(const std::filesystem::path &path,
                                 bool simulate,
                                 const std::optional<std::filesystem::path> &dump_file_path)
		: piksel::BaseApp(1920, 1000) {
	if (simulate) {
		radar = std::make_unique<SimulatedRadar>(path, dump_file_path);
	} else {
		radar = std::make_unique<RealRadar>(path, dump_file_path);
	}
}

void RadarVisualizer::setup() {}

constexpr double deg2rad(double a) {
	return a * (M_PI / 180);
}

void RadarVisualizer::draw(piksel::Graphics &g) {
	radar->process();

	const glm::vec4 black{0, 0, 0, 1};
	const glm::vec4 red{1, 0, 0, 1};
	const glm::vec4 green{0, 0.8, 0, 1};
	const glm::vec4 blue{0, 0, 1, 1};
	const glm::vec4 violet{1, 0, 1, 1};
	const glm::vec4 yellow{.8, .8, 0, 1};
	const glm::vec4 cyan{0, 1, 1, 1};
	const glm::vec4 gray{0.75, 0.75, 0.75, 1};

	g.ellipseMode(piksel::DrawMode::CENTER);
	g.ellipseMode(piksel::DrawMode::RADIUS);
	g.rectMode(piksel::DrawMode::CENTER);
	g.strokeWeight(2);
	g.stroke(black);
	g.noFill();
	g.textSize(20);

	g.background({1, 1, 1, 1});

	g.push();
	g.noStroke();
	g.fill(red);
	g.text("car", 20, 70);
	g.fill(green);
	g.text("pedestrians", 20, 100);
	g.fill(blue);
	g.text("points", 20, 130);
	g.fill(violet);
	g.text("bicycle", 20, 160);
	g.fill(yellow);
	g.text("motorcycle", 20, 190);
	g.fill(cyan);
	g.text("truck", 20, 220);
	g.fill(gray);
	g.text("wide", 20, 250);
	g.fill(black);
	g.text("other", 20, 280);

	if (radar->state.has_value()) {
		g.textSize(11);
		std::stringstream ss;
		ss << radar->state.value();
		g.text(ss.str(), 20, 2 * height / 3);
	}

	g.textSize(18);
	std::stringstream ss_key;
	ss_key << (char) m_key;
	g.text(ss_key.str(), width - 40, 20);
	g.text(std::to_string(m_key), width - 40, 45);
	g.pop();

	// radar
	const auto radar_coord = radar_to_screen_coord(0, 0);
	g.push();
	g.fill(red);
	g.noStroke();
	g.rect(radar_coord.x, radar_coord.y, 8, 30);
	g.pop();

	// range
	const std::array<glm::tvec2<float>, 14> range{
			radar_coord,
			radar_to_screen_coord(20 * cos(deg2rad(60)), 20 * sin(deg2rad(60))),
			radar_to_screen_coord(80 * cos(deg2rad(40)), 80 * sin(deg2rad(40))),
			radar_to_screen_coord(80 * cos(deg2rad(20)), 80 * sin(deg2rad(20))),
			radar_to_screen_coord(80 * cos(deg2rad(9)), 80 * sin(deg2rad(9))),
			radar_to_screen_coord(160 * cos(deg2rad(9)), 160 * sin(deg2rad(9))),
			radar_to_screen_coord(250 * cos(deg2rad(4)), 250 * sin(deg2rad(4))),
			radar_to_screen_coord(250 * cos(deg2rad(4)), -250 * sin(deg2rad(4))),
			radar_to_screen_coord(160 * cos(deg2rad(9)), -160 * sin(deg2rad(9))),
			radar_to_screen_coord(80 * cos(deg2rad(9)), -80 * sin(deg2rad(9))),
			radar_to_screen_coord(80 * cos(deg2rad(20)), -80 * sin(deg2rad(20))),
			radar_to_screen_coord(80 * cos(deg2rad(40)), -80 * sin(deg2rad(40))),
			radar_to_screen_coord(20 * cos(deg2rad(60)), -20 * sin(deg2rad(60))),
			radar_coord
	};
	g.push();
	g.stroke(gray);
	g.strokeWeight(1);
	draw_polygon(g, range);
	g.pop();

	if (radar->measure.has_value()) {
		auto &measure = radar->measure.value();
		unsigned nObjects = measure.objects.size();

		g.push();
		g.fill(black);
		g.noStroke();
		g.text(std::to_string(nObjects) + " objects", 20, 40);
		g.pop();

		for (int i = 0; i < nObjects; i++) {
			const Object &object = measure.objects[i];

			const auto screen_coord = radar_to_screen_coord(object.distance_long, object.distance_lat);
			const float x = screen_coord.x;
			const float y = screen_coord.y;
			const auto dist = sqrt(pow(object.distance_lat, 2) + pow(object.distance_long, 2));

			glm::tvec4<float> transparency{0, 0, 0, 0};
			if (object.quality_info.has_value()) {
				transparency = {0, 0, 0, 1 - object.quality_info->probability_of_existence};
			}

			g.push();
			g.strokeWeight(std::max(3., object.radar_cross_section + 20));
			g.stroke(black - transparency);
			if (object.extended_info.has_value()) {
				if (object.extended_info->object_class == ObjectClass::car) {
					g.stroke(red - transparency);
				} else if (object.extended_info->object_class == ObjectClass::pedestrian) {
					g.stroke(green - transparency);
				} else if (object.extended_info->object_class == ObjectClass::point) {
					g.stroke(blue - transparency);
				} else if (object.extended_info->object_class == ObjectClass::bicycle) {
					g.stroke(violet - transparency);
				} else if (object.extended_info->object_class == ObjectClass::motorcycle) {
					g.stroke(yellow - transparency);
				} else if (object.extended_info->object_class == ObjectClass::truck) {
					g.stroke(cyan - transparency);
				} else if (object.extended_info->object_class == ObjectClass::wide) {
					g.stroke(gray - transparency);
				}
			}
			g.point(x, y);
			g.pop();

			if (display_speed) {
				g.push();
				g.strokeWeight(1);
				auto speed_vector_end = radar_to_screen_coord(object.distance_long + object.relative_velocity_long,
				                                              object.distance_lat + object.relative_velocity_lat);
				g.line(x, y, speed_vector_end.x, speed_vector_end.y);
				g.pop();
			}

			if (display_distance) {
				std::stringstream ss_dist;
				ss_dist << std::fixed << std::setprecision(1) << dist << "m";
				g.push();
				g.textSize(15);
				g.strokeWeight(1);
				g.noStroke();
				g.fill(gray - transparency);
				g.text(ss_dist.str(), x, y);
				g.pop();
			}
		}
	}
}

void RadarVisualizer::keyReleased(int key) {
	m_key = key;
	RadarConfiguration config{};
	std::cout << "key = '" << (char) key << "' => " << key << std::endl;
	if (key == 'O') {
		if (radar->state.has_value()) {
			config.outputType = radar->state->outputTypeCfg == OutputType::CLUSTERS ?
			                    OutputType::OBJECTS : OutputType::CLUSTERS;
		} else {
			config.outputType = OutputType::OBJECTS;
		}
	}
	if (key == 'P') {
		if (radar->state.has_value()) {
			config.radarPower = static_cast<RadarPower>((static_cast<int>(radar->state->radarPowerCfg) + 1) % 4);
		} else {
			config.radarPower = RadarPower::STANDARD;
		}
	}
	if (key == 'T') {
		if (radar->state.has_value()) {
			config.rcsThreshold = radar->state->rcsThreshold == RcsThreshold::STANDARD ?
			                      RcsThreshold::HIGH_SENSITIVITY : RcsThreshold::STANDARD;
		} else {
			config.rcsThreshold = RcsThreshold::STANDARD;
		}
	}
	if (key == 'D') {
		if (radar->state.has_value()) {
			config.maxDistance = 196 + ((radar->state->maxDistanceCfg - 196 + 10) % 64);
		} else {
			config.maxDistance = 196;
		}
	}
	if (key == 59) { // 'M' on azerty keyboards
		display_distance = !display_distance;
	}
	if (key == 'V') { // 'M' on azerty keyboards
		display_speed = !display_speed;
	}
	radar->send_config(config);
}

glm::tvec2<float> RadarVisualizer::radar_to_screen_coord(double lon, double lat) const {
	return {static_cast<float>(lon - Object::DIST_LAT_MIN_OBJECTS + offset.x) * zoom,
	        static_cast<float>(lat - Object::DIST_LONG_MIN + offset.y) * zoom};
}

void RadarVisualizer::mouseWheel(int nb) {
	float last_zoom = zoom;
	zoom *= (1 + 0.15f * static_cast<float>(nb));
	auto new_mouse = mouse_position * float(zoom / last_zoom);
	auto delta = mouse_position - new_mouse;
	offset += delta / zoom;
}

void RadarVisualizer::mouseMoved(int x, int y) {
	const auto delta = glm::vec2{x, y} - mouse_position;
	mouse_position = {x, y};
	if (clicking) {
		offset += delta / zoom;
	}
}

void RadarVisualizer::mousePressed(int button) {
	if (button == 0) {
		clicking = true;
	}
}

void RadarVisualizer::mouseReleased(int button) {
	if (button == 0) {
		clicking = false;
	}
}

