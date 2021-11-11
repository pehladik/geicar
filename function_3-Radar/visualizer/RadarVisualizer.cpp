#include <iostream>
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

void RadarVisualizer::draw(piksel::Graphics &g) {
	radar->process();

	const glm::vec4 black{0, 0, 0, 1};
	const glm::vec4 red{1, 0, 0, 1};
	const glm::vec4 green{0, 0.8, 0, 1};
	const glm::vec4 blue{0, 0, 1, 1};
	const glm::vec4 violet{1, 0, 1, 1};
	const glm::vec4 gray{0.7, 0.7, 0.7, 1};

	g.background({1, 1, 1, 1});
	g.textSize(30);
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
	g.strokeWeight(2);

	if (radar->measure.has_value()) {
		auto &measure = radar->measure.value();
//		std::cout << measure.counter << '\n';
		unsigned nObjects = measure.objects.size();
		g.text(std::to_string(nObjects), 50, 50);
		for (int i = 0; i < nObjects; i++) {
			const Object &object = measure.objects[i];
			if (object.quality_info->probability_of_existence < 0.999) {
				continue;
			}
			const double y = (object.distance_lat - Object::DIST_LAT_MIN - 160) * 7;
			const double x = (object.distance_long - Object::DIST_LONG_MIN - 450) * 7;
			const auto dist = std::to_string(int(sqrt(pow(object.distance_lat, 2) + pow(object.distance_long, 2)))) + "m";
			g.textSize(15);
			g.strokeWeight(1);
			g.noStroke();
			g.fill(gray);
			g.text(dist, x, y);
			g.strokeWeight(2);
			g.noFill();

			if (object.extended_info->object_class == ObjectClass::car) {
				g.stroke(red);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
			} else if (object.extended_info->object_class == ObjectClass::pedestrian) {
				g.stroke(green);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
			} else if (object.extended_info->object_class == ObjectClass::point) {
				g.stroke(blue);
				g.strokeWeight(object.quality_info->probability_of_existence * 5);
				g.point(x, y);
				g.strokeWeight(2);
			} else if (object.extended_info->object_class == ObjectClass::bicycle) {
				g.stroke(violet);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
			} else {
				g.stroke(black);
				g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
			}
//			std::cout << x << ", " << y << '\n';
		}
	}
}
