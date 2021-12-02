#ifndef GEIFLIX_RADAR_HPP
#define GEIFLIX_RADAR_HPP

#include <string>
#include <memory>
#include <deque>
#include <fstream>
#include <chrono>
#include "Message.hpp"
#include "Measure.hpp"
#include "Config.hpp"
#include "CollisionDetection.hpp"

class Radar {
public:
	Radar(const std::optional<std::string> &dump_file_path);

	/**
	 * Read all messages newly received and update the state and measure fields
	 */
	void process();

	/**
	 * Sends a radar configuration message to the radar
	 * @param config The configuration to send. All optional attributes that are not set are not modified.
	 */
	void send_config(const RadarConfiguration &config);
	static std::unique_ptr<message::MessageIn> parse_message(uint32_t timestamp, std::uint32_t id, std::uint8_t data[8]);
	virtual ~Radar() = default;

	/**
	 * The last parsed measure, or nullopt if not yet received
	 */
	std::optional<Measure> measure{std::nullopt};

	/**
	 * The last parsed radar state, or nullopt if not yet received
	 */
	std::optional<::RadarState> state{std::nullopt};

protected:
	virtual std::unique_ptr<message::MessageIn> receive() = 0;
	virtual void send(std::unique_ptr<message::MessageOut> msg) = 0;
	void generate_measure();
	void write_to_dump_file(uint32_t timestamp, uint32_t ident, const uint8_t data[8]);
	std::deque<std::unique_ptr<message::MessageIn>> message_queue{};
	std::chrono::high_resolution_clock::time_point time_started{std::chrono::high_resolution_clock::now()};
	std::optional<std::ofstream> dump_file = std::nullopt;
};

class RealRadar : public Radar {
public:
	/**
	 * @param serial_port The port where the CAN/USB converter
	 *                    is connected to (example: /dev/ttyACM1)
	 * @param dump_file_path An optional path to create a dump file
	 *                       with all the messages received during the session
	 */
	RealRadar(const std::string &serial_port,
	          const std::optional<std::string> &dump_file_path = std::nullopt);

	/**
	 * @return the error string associated with the last SimplyCAN function call
	 */
	std::string get_last_error();

	virtual ~RealRadar();

private:
	void init_can(const std::string &serial_port);
	void send(std::unique_ptr<message::MessageOut> msg) override;
	std::unique_ptr<message::MessageIn> receive() override;
};

class SimulatedRadar : public Radar {
public:
	/**
	 * @param path The path to the dump file where the messages were previously saved
	 * @param dump_file_path An optional path to create a dump file
	 *                       with all the messages received during the session
	 */
	explicit SimulatedRadar(const std::string &path,
	                        const std::optional<std::string> &dump_file_path = std::nullopt);

	virtual ~SimulatedRadar() = default;

private:
	std::unique_ptr<message::MessageIn> receive() override;
	void send(std::unique_ptr<message::MessageOut> msg) override;
	std::ifstream file;
};

#endif //GEIFLIX_RADAR_HPP
