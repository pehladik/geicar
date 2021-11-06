#ifndef GEIFLIX_RADAR_HPP
#define GEIFLIX_RADAR_HPP

#include <string>
#include <memory>
#include <deque>
#include <filesystem>
#include <fstream>
#include "Message.hpp"
#include "Measure.hpp"

class Radar {
public:
	virtual std::unique_ptr<Message> receive() = 0;
	static std::unique_ptr<Message> parse_message(std::uint32_t id, std::uint8_t data[8]);
	virtual void process() = 0;
	virtual ~Radar() = default;
	std::optional<Measure> measure{std::nullopt};
protected:
	std::deque<std::unique_ptr<Message>> message_queue{};
	void generate_measure();
};

class RealRadar : public Radar {
public:
	explicit RealRadar(const std::string &serial_port);
	std::unique_ptr<Message> receive();
	void process() override;
	std::string get_last_error();
	void dump_to_file(const std::filesystem::path &path, int nb_messages);
	virtual ~RealRadar();
};

class SimulatedRadar : public Radar {
public:
	explicit SimulatedRadar(const std::filesystem::path &path);
	std::unique_ptr<Message> receive();
	void process() override;
	virtual ~SimulatedRadar();
private:
	std::ifstream file;
};

#endif //GEIFLIX_RADAR_HPP
