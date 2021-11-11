#ifndef GEIFLIX_RADAR_HPP
#define GEIFLIX_RADAR_HPP

#include <string>
#include <memory>
#include <deque>
#include <filesystem>
#include <fstream>
#include <chrono>
#include "Message.hpp"
#include "Measure.hpp"

class Radar {
public:
	Radar(const std::optional<std::filesystem::path> &dump_file_path);
	virtual std::unique_ptr<Message> receive() = 0;
	static std::unique_ptr<Message> parse_message(std::uint32_t id, std::uint8_t data[8]);
	virtual void process();
	virtual ~Radar() = default;
	std::optional<Measure> measure{std::nullopt};
protected:
	void generate_measure();
	void write_to_dump_file(uint32_t ident, const uint8_t data[8]);
	std::deque<std::unique_ptr<Message>> message_queue{};
	std::chrono::high_resolution_clock::time_point time_started{std::chrono::high_resolution_clock::now()};
	std::optional<std::ofstream> dump_file = std::nullopt;
};

class RealRadar : public Radar {
public:
	RealRadar(const std::string &serial_port,
	          const std::optional<std::filesystem::path> &dump_file_path = std::nullopt);
	std::unique_ptr<Message> receive() override;
	std::string get_last_error();
	virtual ~RealRadar();
private:
	void init_can(const std::string &serial_port);
};

class SimulatedRadar : public Radar {
public:
	explicit SimulatedRadar(const std::filesystem::path &path,
	                        const std::optional<std::filesystem::path> &dump_file_path = std::nullopt);
	std::unique_ptr<Message> receive() override;
	virtual ~SimulatedRadar() = default;
private:
	std::ifstream file;
};

#endif //GEIFLIX_RADAR_HPP
