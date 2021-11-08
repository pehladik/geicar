#include <iostream>
#include "RadarVisualizer.hpp"

void RadarVisualizer::setup() {
}

void RadarVisualizer::draw(piksel::Graphics &g) {
	radar->process();

	g.background({1, 1, 1, 1});
	g.textSize(30);
	g.strokeWeight(2);
	g.stroke({0, 0, 0, 1});
	g.noFill();

	if (radar->measure.has_value()) {
		auto &measure = radar->measure.value();
		unsigned nObjects = measure.objects.size();
		g.text(std::to_string(nObjects), 50, 50);
		for (int i = 0; i < nObjects; i++) {
			const Object &object = measure.objects[i];
			const double x = (object.distance_lat - Object::DIST_LAT_MIN - 200) * 50;
			const double y = (object.distance_long - Object::DIST_LONG_MIN - 500) * 50;
//			std::cout << x << ", " << y << '\n';
			g.ellipse(x, y, 10 * sqrt(object.radar_cross_section), 10 * sqrt(object.radar_cross_section));
		}
	}

//	try {
//		auto msg = radar->receive();
//		std::cout << *msg;
//		if (auto object_list = dynamic_cast<ObjectListStatus *>(msg.get())) {
//			nObjects = object_list->nofObjects.to_ulong();
//			std::cout << object_list->nofObjects.to_ulong() << std::endl;
//		}
//	} catch (const std::runtime_error &e) {
//		std::cout << e.what() << '\n';
//	}
//	g.fill({0,0,0,1});
//	g.background({1, 1, 1, 1});
//	g.rect(50, 50, 500, 200);
//	g.stroke({1, 0, 0, 1});
//	g.strokeWeight(5);
//	g.line(50, 50, 550, 250);
}

RadarVisualizer::RadarVisualizer(const std::string &path, bool simulate)
		: piksel::BaseApp(1700, /*(Object::DIST_LAT_MAX - Object::DIST_LAT_MIN) / 2,*/
		                  800) { /*(Object::DIST_LONG_MAX - Object::DIST_LONG_MIN) / 2) {*/
	if (simulate) {
		radar = std::make_unique<SimulatedRadar>(path);
	} else {
		radar = std::make_unique<RealRadar>(path);
	}
}
