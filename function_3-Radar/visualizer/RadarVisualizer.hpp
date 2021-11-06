#ifndef GEIFLIX_RADARVISUALIZER_HPP
#define GEIFLIX_RADARVISUALIZER_HPP

#include <baseapp.hpp>
#include <Radar.hpp>


class RadarVisualizer : public piksel::BaseApp {
public:
	explicit RadarVisualizer(const std::string &path, bool simulate = false);
	void setup();
	void draw(piksel::Graphics &g);

private:
	std::unique_ptr<Radar> radar;
};


#endif //GEIFLIX_RADARVISUALIZER_HPP
