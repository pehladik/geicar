#include "Radar.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <util.hpp>
#include <algorithm>
#include "simply.h"
#include "Message.hpp"

using namespace message;

Radar::Radar(const std::optional<std::filesystem::path> &dump_file_path) : dump_file{dump_file_path} {
	if (dump_file.has_value() && !dump_file->good()) {
		throw std::runtime_error{"Failed to open the file " + dump_file_path->string()};
	}
}

void Radar::process() {
	while (true) {
		try {
			auto msg = receive();
			if (!msg)
				break;
//			std::cout << *msg;
			if (dynamic_cast<ObjectListStatus *>(msg.get())) {
				generate_measure();
			}
			if (auto config = dynamic_cast<message::RadarState *>(msg.get())) {
				state = ::RadarState(*config);
//				std::cout << *state << '\n';
			}
			message_queue.push_back(std::move(msg));
		} catch (const std::runtime_error &e) {
			std::cout << e.what() << '\n';
		}
	}
}

std::unique_ptr<MessageBase> Radar::parse_message(uint32_t id, std::uint8_t data[8]) {
	std::bitset<64> payload_reversed = 0;
	for (int i = 0; i < 8; ++i) {
		std::uint64_t reversed_payload_byte = reverse_byte(data[i]);
		payload_reversed |= reversed_payload_byte << (i * 8);
	}
	return MessageBase::parse(id, payload_reversed);
}

void Radar::generate_measure() {
	ObjectListStatus *object_list{};
	while (object_list == nullptr && !message_queue.empty()) {
		object_list = dynamic_cast<ObjectListStatus *>(message_queue.front().get());
		message_queue.pop_front();
	}

	if (object_list) {
		std::vector<ObjectGeneralInfo> general_info_list{};
		std::vector<ObjectQualityInfo> quality_info_list{};
		std::vector<ObjectExtInfo> ext_info_list{};
		for (auto &&msg: message_queue) {
			if (auto general_info = dynamic_cast<ObjectGeneralInfo *>(msg.get())) {
				general_info_list.emplace_back(*general_info);
			}
			if (auto quality_info = dynamic_cast<ObjectQualityInfo *>(msg.get())) {
				quality_info_list.emplace_back(*quality_info);
			}
			if (auto ext_info = dynamic_cast<ObjectExtInfo *>(msg.get())) {
				ext_info_list.emplace_back(*ext_info);
			}
		}
//		std::cout << "generating a new measure...\n";
		measure = Measure{*object_list, general_info_list, quality_info_list, ext_info_list};
	}

	message_queue.clear();
}

RealRadar::RealRadar(const std::string &serial_port,
                     const std::optional<std::filesystem::path> &dump_file_path) : Radar{dump_file_path} {
	init_can(serial_port);
}

void RealRadar::init_can(const std::string &serial_port) {
	char *serial_port_c = new char[serial_port.length() + 1];
	strcpy(serial_port_c, serial_port.c_str());
	if (!simply_open(serial_port_c))
		throw std::runtime_error{"Error opening the CAN bus: " + get_last_error()};
	if (!simply_initialize_can(500))
		throw std::runtime_error{"Error initializing the CAN bus: " + get_last_error()};
	if (!simply_start_can())
		throw std::runtime_error{"Error starting the CAN bus: " + get_last_error()};
}

std::unique_ptr<MessageBase> RealRadar::receive() {
	can_msg_t msg;
	std::int8_t received;
	received = simply_receive(&msg);
	if (received == -1)
		throw std::runtime_error{"Error receiving CAN message: " + get_last_error()};
	if (received == 0)
		return nullptr;
	if (dump_file.has_value()) {
		write_to_dump_file(msg.ident, msg.payload);
	}
	return parse_message(msg.ident, msg.payload);
}

std::string RealRadar::get_last_error() {
	int err = simply_get_last_error();
	switch (err) {
		case 0:
			return "No error occurred";
		case -1:
			return "Unable to open the serial port";
		case -2:
			return "Access on serial port denied";
		case -3:
			return "Serial communication port closed";
		case -4:
			return "Serial communication error";
		case -5:
			return "Command unknown on device";
		case -6:
			return "Command response timeout reached";
		case -7:
			return "Unexpected command response received";
		case -8:
			return "Command response error";
		case -9:
			return "Invalid simplyCAN protocol version";
		case -10:
			return "Invalid device firmware version";
		case -11:
			return "Invalid simplyCAN product string";
		case -12:
			return "Invalid CAN state";
		case -13:
			return "Invalid CAN baudrate";
		case -14:
			return "Message not sent, Tx is busy";
		case -15:
			return "API is busy";
		default:
			return "Unknown error";
	}
}

RealRadar::~RealRadar() {
	if (!simply_close())
		std::cout << "Error closing the CAN bus: " << get_last_error();
}

void RealRadar::send(std::unique_ptr<message::MessageBase> msg) {
	std::cout << "Sending message :\n" << *msg;
	can_msg_t can_msg{};
	can_msg.ident = msg->get_id();
	std::uint64_t payload = msg->to_payload().to_ullong();
	for (int i = 0; i < 8; ++i) {
		can_msg.payload[i] = (payload >> (i * 8) & 0xff);
	}
	can_msg.dlc = 8;
	simply_send(&can_msg);
}

void Radar::write_to_dump_file(uint32_t id, const std::uint8_t data[8]) {
	using namespace std::chrono;
	if (!dump_file.has_value()) {
		return;
	}
	auto now = high_resolution_clock::now();
	auto ms = duration_cast<milliseconds>(now - time_started).count();
	*dump_file << ms << " " << id << " ";
	for (int i = 0; i < 8; ++i) {
		*dump_file << static_cast<unsigned>(data[i]) << " ";
	}
	*dump_file << "\n";
}

void Radar::send_config(const RadarConfiguration &config) {
	send(std::make_unique<message::RadarConfig>(config.to_message()));
}


SimulatedRadar::SimulatedRadar(const std::filesystem::path &path,
                               const std::optional<std::filesystem::path> &dump_file_path) :
		file{path}, Radar{dump_file_path} {
	if (!file.good()) {
		throw std::runtime_error{"Failed to open the file " + path.string()};
	}
	// Skip waiting before the first message
	auto prev_position = file.tellg();
	int first_message_time;
	file >> first_message_time;
	if (file.good()) {
		time_started -= std::chrono::milliseconds(first_message_time);
		file.seekg(prev_position);
	}
}

std::unique_ptr<MessageBase> SimulatedRadar::receive() {
	using namespace std::chrono;
	std::uint8_t data[8];
	std::uint32_t id;
	long message_ms;
	auto prev_position = file.tellg();

	file >> message_ms;
	if (!file.good())
		return nullptr;

	auto now = high_resolution_clock::now();
	auto ms = duration_cast<milliseconds>(now - time_started).count();

	if (message_ms > ms) {
		file.seekg(prev_position);
		return nullptr;
	} else {
		file >> id;
		for (std::uint8_t &byte: data) {
			unsigned tmp;
			file >> tmp;
			byte = static_cast<std::uint8_t>(tmp);
		}
		if (file.good()) {
			if (dump_file.has_value()) {
				write_to_dump_file(id, data);
			}
			return Radar::parse_message(id, data);
		} else {
			return nullptr;
		}
	}
}

void SimulatedRadar::send(std::unique_ptr<message::MessageBase> msg) {
	std::cout << "Sending message :\n" << *msg;
	std::cout << "Simulated radar -> skipping\n";
}

