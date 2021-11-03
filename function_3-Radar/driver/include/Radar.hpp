#ifndef GEIFLIX_RADAR_HPP
#define GEIFLIX_RADAR_HPP

#include <string>
#include <memory>
#include <filesystem>
#include <fstream>
#include "Message.hpp"

class Radar {
public:
	explicit Radar(const std::string &serial_port);
	std::unique_ptr<Message> receive();
	static std::unique_ptr<Message> parse_message(std::uint32_t id, std::uint8_t data[8]);
	std::string get_last_error();
	void dump_to_file(const std::filesystem::path &path, int nb_messages);
	virtual ~Radar();
};

class SimulatedRadar {
public:
	explicit SimulatedRadar(const std::filesystem::path &path);
	std::unique_ptr<Message> receive();
	virtual ~SimulatedRadar();
private:
	std::ifstream file;
};

#endif //GEIFLIX_RADAR_HPP
