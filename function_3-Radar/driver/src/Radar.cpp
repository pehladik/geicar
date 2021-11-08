#include "Radar.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <util.hpp>
#include <algorithm>
#include <cassert>
#include "simply.h"
#include "Message.hpp"

RealRadar::RealRadar(const std::string &serial_port) {
	char *serial_port_c = new char[serial_port.length() + 1];
	strcpy(serial_port_c, serial_port.c_str());
	if (!simply_open(serial_port_c))
		throw std::runtime_error{"Error opening the CAN bus: " + get_last_error()};
	if (!simply_initialize_can(500))
		throw std::runtime_error{"Error initializing the CAN bus: " + get_last_error()};
	if (!simply_start_can())
		throw std::runtime_error{"Error starting the CAN bus: " + get_last_error()};
}

std::unique_ptr<Message> RealRadar::receive() {
	can_msg_t msg;
	std::int8_t received;
	received = simply_receive(&msg);
	if (received == -1)
		throw std::runtime_error{"Error receiving CAN message: " + get_last_error()};
	if (received == 0)
		return nullptr;
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

std::unique_ptr<Message> Radar::parse_message(uint32_t id, std::uint8_t data[8]) {
//	std::bitset<64> payload = 0;
	std::bitset<64> payload_reversed = 0;
	for (int i = 0; i < 8; ++i) {
//		std::uint64_t payload_byte = msg.payload[i];
		std::uint64_t reversed_payload_byte = reverse_byte(data[i]);
//		payload |= payload_byte << (i * 8);
		payload_reversed |= reversed_payload_byte << (i * 8);
//		std::cout << i << '\n';
//		std::cout << std::bitset<64>{payload_byte << (i * 8)} << "\n";
//		std::cout << std::bitset<64>{reversed_payload_byte << (i * 8)} << "\n";
//		std::cout << payload.to_string() << "\n";
//		std::cout << payload_reversed.to_string() << "\n";
	}
//	std::cout << "payload = \n";
//	print_bitset64(std::cout, payload);
//	std::cout << "payload reversed = \n";
//	print_bitset64(std::cout, payload_reversed);
	return Message::parse(id, payload_reversed);
}

void RealRadar::dump_to_file(const std::filesystem::path &path, int nb_messages) {
	can_msg_t msg;
	std::int8_t received;
	std::ofstream file{path};
	for (int i = 0; i < nb_messages; ++i) {
		do {
			received = simply_receive(&msg);
			if (received == -1)
				throw std::runtime_error{"Error receiving CAN message: " + get_last_error()};
		} while (!received);
		file << msg.ident << " ";
		for (std::uint8_t byte: msg.payload) {
			unsigned tmp = byte;
			file << tmp << " ";
		}
		file << "\n";
	}
	file.close();
}

void RealRadar::process() {
	while (true) {
		try {
			auto msg = receive();
			if (!msg)
				break;
//			std::cout << *msg;
			if (dynamic_cast<ObjectListStatus *>(msg.get())) {
				generate_measure();
			}
			message_queue.push_back(std::move(msg));
		} catch (const std::runtime_error &e) {
			std::cout << e.what() << '\n';
		}
	}
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

SimulatedRadar::SimulatedRadar(const std::filesystem::path &path) : file{path} {
	if (!file.good()) {
		throw std::runtime_error{"Failed to open the file " + path.string()};
	}
}

std::unique_ptr<Message> SimulatedRadar::receive() {
	std::uint8_t data[8];
	std::uint32_t id;
	file >> id;
	for (std::uint8_t &byte: data) {
		unsigned tmp;
		file >> tmp;
		byte = static_cast<std::uint8_t>(tmp);
	}
	if (file.good())
		return Radar::parse_message(id, data);
	else
		return nullptr;
}

void SimulatedRadar::process() {
	for (int i = 0; i < 1; ++i) {
		try {
			auto msg = receive();
			if (!msg)
				break;
//			std::cout << *msg;
			if (dynamic_cast<ObjectListStatus *>(msg.get())) {
				generate_measure();
			}
			message_queue.push_back(std::move(msg));
		} catch (const std::runtime_error &e) {
			std::cout << e.what() << '\n';
		}
	}
}

SimulatedRadar::~SimulatedRadar() {
	file.close();
}
