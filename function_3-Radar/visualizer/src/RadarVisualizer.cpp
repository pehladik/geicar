#include <iostream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include "RadarVisualizer.hpp"

RadarVisualizer::RadarVisualizer(std::function<void()> process,
                                 std::function<std::optional<Measure>()> measure_getter,
                                 std::function<void(const RadarConfiguration &)> config_sender,
                                 std::function<std::optional<RadarState>()> state_getter) :
		piksel::BaseApp(1920, 1000),
		process{std::move(process)},
		get_measure{std::move(measure_getter)},
		get_state{std::move(state_getter)},
		send_config{std::move(config_sender)} {}

void RadarVisualizer::setup() {}

constexpr double deg2rad(double a) {
	return a * (M_PI / 180);
}

bool RadarVisualizer::is_obstacle_dangerous(const Object &obj) const {
	return obj.distance_long < warning_region_size.x &&
	       obj.distance_lat < warning_region_size.y / 2 &&
	       obj.distance_lat > -warning_region_size.y / 2;
}

void RadarVisualizer::draw(piksel::Graphics &g) {
	process();

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

	auto state = get_state();
	if (state.has_value()) {
		g.textSize(11);
		std::stringstream ss;
		ss << state.value();
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

	auto measure = get_measure();
	if (measure.has_value()) {
		unsigned nObjects = measure->objects.size();

		if (display_warning) {
			g.push();
			auto warning_box1 = radar_to_screen_coord(0, -warning_region_size.y / 2);
			auto warning_box2 = radar_to_screen_coord(warning_region_size.x, warning_region_size.y / 2);
			g.noFill();
			g.strokeWeight(1);
			g.stroke(red);
			g.rectMode(piksel::DrawMode::CORNERS);
			g.rect(warning_box1.x, warning_box1.y, warning_box2.x, warning_box2.y);
			std::stringstream ss;
			g.fill(red);
			g.noStroke();
			ss << std::setprecision(2) << warning_region_size.x << "*" << warning_region_size.y;
			g.text(ss.str(), width - 130, height - 40);
			g.pop();
			bool danger = std::any_of(measure->objects.begin(), measure->objects.end(),
			                          [this](const Object &o) {return is_obstacle_dangerous(o);});
			if (danger) {
				g.push();
				g.textSize(100);
				g.noStroke();
				g.fill(red);
				g.text("obstacle", width - 630, 130);
				g.pop();
			}
		}

		g.push();
		g.fill(black);
		g.noStroke();
		g.text(std::to_string(nObjects) + " objects", 20, 40);
		g.pop();

		for (int i = 0; i < nObjects; i++) {
			const Object &object = measure->objects[i];

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
	auto state = get_state();
	if (key == 'O') {
		if (state.has_value()) {
			config.outputType = state->outputTypeCfg == OutputType::CLUSTERS ?
			                    OutputType::OBJECTS : OutputType::CLUSTERS;
		} else {
			config.outputType = OutputType::OBJECTS;
		}
	}
	if (key == 'P') {
		if (state.has_value()) {
			config.radarPower = static_cast<RadarPower>((static_cast<int>(state->radarPowerCfg) + 1) % 4);
		} else {
			config.radarPower = RadarPower::STANDARD;
		}
	}
	if (key == 'T') {
		if (state.has_value()) {
			config.rcsThreshold = state->rcsThreshold == RcsThreshold::STANDARD ?
			                      RcsThreshold::HIGH_SENSITIVITY : RcsThreshold::STANDARD;
		} else {
			config.rcsThreshold = RcsThreshold::STANDARD;
		}
	}
	if (key == 'D') {
		if (state.has_value()) {
			config.maxDistance = 196 + ((state->maxDistanceCfg - 196 + 10) % 64);
		} else {
			config.maxDistance = 196;
		}
	}
	if (key == 59) { // 'M' on azerty keyboards
		display_distance = !display_distance;
	}
	if (key == 'V') {
		display_speed = !display_speed;
	}
	if (key == 'Z') { // 'W' on azerty keyboards
		display_warning = !display_warning;
	}
	send_config(config);
}

glm::tvec2<float> RadarVisualizer::radar_to_screen_coord(double lon, double lat) const {
	return {(lon + offset.x) * zoom,
	        (lat + offset.y) * zoom};
}

glm::vec2 RadarVisualizer::screen_to_radar_coord(const glm::vec2 &coord) const {
	return {coord.x / zoom - offset.x,
	        coord.y / zoom - offset.y};
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
	if (left_clicking) {
		offset += delta / zoom;
	}
	if (right_clicking) {
		warning_region_size = screen_to_radar_coord(mouse_position);
		warning_region_size.y *= 2;
		warning_region_size.y = abs(warning_region_size.y);
	}
}

void RadarVisualizer::mousePressed(int button) {
	if (button == 0) {
		left_clicking = true;
	}
	if (button == 1) {
		right_clicking = true;
	}
}

void RadarVisualizer::mouseReleased(int button) {
	if (button == 0) {
		left_clicking = false;
	}
	if (button == 1) {
		right_clicking = false;
	}
}

