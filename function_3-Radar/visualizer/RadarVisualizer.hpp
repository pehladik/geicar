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
	void mouseWheel(int delta) override;
	void mouseMoved(int x, int y) override;
	void mousePressed(int button) override;
	void mouseReleased(int button) override;

private:
	template<std::size_t N>
	void draw_polygon(piksel::Graphics &g, const std::array<glm::vec2, N> &points);
	glm::vec2 radar_to_screen_coord(double lon, double lat) const;
	glm::vec2 screen_to_radar_coord(const glm::vec2 &coord) const;
	bool is_obstacle_dangerous(const Object &obj) const;
	int m_key;
	std::unique_ptr<Radar> radar;
	glm::vec2 offset{-195. - Object::DIST_LAT_MIN_OBJECTS, -468 - Object::DIST_LONG_MIN};
	glm::vec2 mouse_position{0, 0};
	float zoom = 18;
	bool right_clicking = false;
	bool left_clicking = false;
	bool display_distance = true;
	bool display_speed = true;
	bool display_warning = true;
	glm::vec2 warning_region_size{2, 2};
};


template<std::size_t N>
void RadarVisualizer::draw_polygon(piksel::Graphics &g, const std::array<glm::vec2, N> &points) {
	for (int i = 0; i < points.size() - 1; ++i) {
		g.line(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
	}
}

#endif //GEIFLIX_RADARVISUALIZER_HPP
