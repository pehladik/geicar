#include "Radar.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <util.hpp>
#include "simply.h"
#include "Message.hpp"

Radar::Radar(const std::string &serial_port) {
	char *serial_port_c = new char[serial_port.length() + 1];
	strcpy(serial_port_c, serial_port.c_str());
	if (!simply_open(serial_port_c))
		throw std::runtime_error{"Error opening the CAN bus: " + get_last_error()};
	if (!simply_initialize_can(500))
		throw std::runtime_error{"Error initializing the CAN bus: " + get_last_error()};
	if (!simply_start_can())
		throw std::runtime_error{"Error starting the CAN bus: " + get_last_error()};
}

std::unique_ptr<Message> Radar::receive() {
	can_msg_t msg;
	std::int8_t received;
	do {
		received = simply_receive(&msg);
		if (received == -1)
			throw std::runtime_error{"Error receiving CAN message: " + get_last_error()};
	} while (!received);
	return parse_message(msg.ident, msg.payload);
}

std::string Radar::get_last_error() {
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

Radar::~Radar() {
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

void Radar::dump_to_file(const std::filesystem::path &path, int nb_messages) {
	can_msg_t msg;
	std::int8_t received;
	std::ofstream file{path};
	for (int i = 0; i < nb_messages; ++i) {
		received = simply_receive(&msg);
		if (received == -1)
			throw std::runtime_error{"Error receiving CAN message: " + get_last_error()};
		file << msg.ident << " ";
		for (std::uint8_t byte: msg.payload) {
			unsigned tmp = byte;
			file << tmp << " ";
		}
		file << "\n";
	}
	file.close();
}

SimulatedRadar::SimulatedRadar(const std::filesystem::path &path) : file{path} {}

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

SimulatedRadar::~SimulatedRadar() {
	file.close();
}
