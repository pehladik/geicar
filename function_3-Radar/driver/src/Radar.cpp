#include "Radar.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
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

std::uint8_t reverse_bits(std::uint8_t b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

std::unique_ptr<message> Radar::receive() {
	can_msg_t msg;
	int8_t received = 0;
	do {
		received = simply_receive(&msg);
		if (received == -1)
			throw std::runtime_error{"Error receiving CAN message: " + get_last_error()};
	} while (!received);

	std::uint64_t payload_val = 0;
	for (int i = 0; i < 8; ++i) {
		payload_val += reverse_bits(msg.payload[i]) << (i * 8);
	}
	std::bitset<64> payload{payload_val};
	return message::parse(msg.ident, payload);
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
	}
}

Radar::~Radar() {
	if (!simply_close())
		std::cout << "Error closing the CAN bus: " << get_last_error();
}
