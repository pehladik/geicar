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


void RadarVisualizer::setup() {
}

constexpr double deg2rad(double a) {
	return a * (M_PI / 180);
}

struct Coord {
	float x;
	float y;
};

constexpr Coord radar_to_screen_coord(double lon, double lat) {
	return {static_cast<float>((lat - Object::DIST_LAT_MIN_OBJECTS - 195.) * 18.),
	        static_cast<float>((lon - Object::DIST_LONG_MIN - 468.) * 18.)};
}

template<std::size_t N>
void draw_polygon(piksel::Graphics &g, const std::array<Coord, N> &points) {
	for (int i = 0; i < points.size() - 1; ++i) {
		g.line(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
	}
}

void RadarVisualizer::draw(piksel::Graphics &g) {
	radar->process();

	const glm::vec4 black{0, 0, 0, 1};
	const glm::vec4 red{1, 0, 0, 1};
	const glm::vec4 green{0, 0.8, 0, 1};
	const glm::vec4 blue{0, 0, 1, 1};
	const glm::vec4 violet{1, 0, 1, 1};
	const glm::vec4 gray{0.75, 0.75, 0.75, 1};

	g.ellipseMode(piksel::DrawMode::CENTER);
	g.ellipseMode(piksel::DrawMode::RADIUS);
	g.rectMode(piksel::DrawMode::CENTER);
	g.strokeWeight(2);
	g.stroke(black);
	g.noFill();
	g.textSize(30);

	g.background({1, 1, 1, 1});

	g.push();
	g.noStroke();
	g.fill(red);
	g.text("car", 50, 100);
	g.fill(green);
	g.text("pedestrians", 50, 150);
	g.fill(blue);
	g.text("points", 50, 200);
	g.fill(violet);
	g.text("bicycle", 50, 250);
	g.fill(black);
	g.text("other", 50, 300);
	if (radar->state.has_value()) {
		g.textSize(11);
		std::stringstream ss;
		ss << radar->state.value();
		g.text(ss.str(), 0, 2 * height / 3);
	}
	g.textSize(18);
	std::stringstream ss_key;
	ss_key << (char) m_key;
	g.text(ss_key.str(), width - 40, 20);
	g.text(std::to_string(m_key), width - 40, 45);
	g.pop();


	// radar
	const Coord radar_coord = radar_to_screen_coord(0, 0);
	g.push();
	g.fill(red);
	g.noStroke();
	g.rect(radar_coord.x, radar_coord.y, 8, 30);
	g.pop();

	// range
	const std::array<Coord, 14> range{
			radar_coord,
			radar_to_screen_coord(20 * sin(deg2rad(60)), 20 * cos(deg2rad(60))),
			radar_to_screen_coord(80 * sin(deg2rad(40)), 80 * cos(deg2rad(40))),
			radar_to_screen_coord(80 * sin(deg2rad(20)), 80 * cos(deg2rad(20))),
			radar_to_screen_coord(80 * sin(deg2rad(9)), 80 * cos(deg2rad(9))),
			radar_to_screen_coord(160 * sin(deg2rad(9)), 160 * cos(deg2rad(9))),
			radar_to_screen_coord(250 * sin(deg2rad(4)), 250 * cos(deg2rad(4))),
			radar_to_screen_coord(-250 * sin(deg2rad(4)), 250 * cos(deg2rad(4))),
			radar_to_screen_coord(-160 * sin(deg2rad(9)), 160 * cos(deg2rad(9))),
			radar_to_screen_coord(-80 * sin(deg2rad(9)), 80 * cos(deg2rad(9))),
			radar_to_screen_coord(-80 * sin(deg2rad(20)), 80 * cos(deg2rad(20))),
			radar_to_screen_coord(-80 * sin(deg2rad(40)), 80 * cos(deg2rad(40))),
			radar_to_screen_coord(-20 * sin(deg2rad(60)), 20 * cos(deg2rad(60))),
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
		g.text(std::to_string(nObjects) + " objects", 50, 50);
		g.pop();

		for (int i = 0; i < nObjects; i++) {
			const Object &object = measure.objects[i];

			const Coord screen_coord = radar_to_screen_coord(object.distance_lat, object.distance_long);
			const float x = screen_coord.x;
			const float y = screen_coord.y;
			const auto dist = sqrt(pow(object.distance_lat, 2) + pow(object.distance_long, 2));
			std::stringstream ss_dist;
			ss_dist << std::fixed << std::setprecision(1) << dist << "m";

			glm::tvec4<float> transparency{0, 0, 0, 0};
			if (object.quality_info.has_value()) {
				transparency = {0, 0, 0, 1 - object.quality_info->probability_of_existence};
			}

			g.push();
			g.textSize(15);
			g.strokeWeight(1);
			g.noStroke();
			g.fill(gray - transparency);
			g.text(ss_dist.str(), x, y);
			g.pop();


			if (object.extended_info->object_class == ObjectClass::car) {
				g.push();
				g.stroke(red - transparency);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
				g.pop();
			} else if (object.extended_info->object_class == ObjectClass::pedestrian) {
				g.push();
				g.stroke(green - transparency);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
				g.pop();
			} else if (object.extended_info->object_class == ObjectClass::point) {
				g.push();
				g.stroke(blue - transparency);
				g.strokeWeight(5);
				g.point(x, y);
				g.pop();
			} else if (object.extended_info->object_class == ObjectClass::bicycle) {
				g.push();
				g.stroke(violet - transparency);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
				g.pop();
			} else {
				g.push();
				g.stroke(black - transparency);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
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
	radar->send_config(config);
}
