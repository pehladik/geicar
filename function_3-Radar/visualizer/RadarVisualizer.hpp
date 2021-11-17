#ifndef GEIFLIX_RADARVISUALIZER_HPP
#define GEIFLIX_RADARVISUALIZER_HPP

#include <baseapp.hpp>
#include <Radar.hpp>


class RadarVisualizer : public piksel::BaseApp {
public:
	explicit RadarVisualizer(const std::filesystem::path &path,
	                         bool simulate = false,
	                         const std::optional<std::filesystem::path> &dump_file_path = std::nullopt);
	void setup();
	void draw(piksel::Graphics &g);
	void keyReleased(int key) override;

private:
	int m_key;
	std::unique_ptr<Radar> radar;
};


#endif //GEIFLIX_RADARVISUALIZER_HPP
