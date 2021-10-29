#ifndef GEIFLIX_RADAR_HPP
#define GEIFLIX_RADAR_HPP

#include <string>
#include <memory>
#include "Message.hpp"

class Radar {
public:
	explicit Radar(const std::string &serial_port);
	std::unique_ptr<message> receive();
	std::string get_last_error();
	virtual ~Radar();
};

#endif //GEIFLIX_RADAR_HPP
