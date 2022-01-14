#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <simply.h>
#include "stm32_ros_msgs/emergency_stop.h"
#include "stm32_ros_msgs/brakes.h"
#include "stm32_ros_msgs/motor.h"
#include "stm32_ros_msgs/steering.h"


std::string get_last_error();

using namespace stm32_ros_msgs;

bool send_motor_power(motorRequest &request, motorResponse &response);
bool send_brakes(brakesRequest &request, brakesResponse &response);
bool send_emergency_stop(emergency_stopRequest &request, emergency_stopResponse &response);
bool send_steering_angle(steeringRequest &request, steeringResponse &response);


int main(int argc, char **argv) {
	ros::init(argc, argv, "stm32_can_receiver_node");

	if (argc != 2) {
		ROS_ERROR("One argument required: path to the CAN device");
		return 1;
	}
	char *serial_port = argv[1];

	ros::NodeHandle n;

	auto emergency_stop_srv = n.advertiseService("stm32/emergency_stop", send_emergency_stop);
	auto brakes_srv = n.advertiseService("stm32/brakes", send_brakes);
	auto motor_srv = n.advertiseService("stm32/motor", send_motor_power);
	auto steering_srv = n.advertiseService("stm32/steering", send_steering_angle);

	ros::Publisher publisher_init = n.advertise<std_msgs::Bool>("stm32/init", 1000);
	ros::Publisher publisher_us = n.advertise<std_msgs::Float32>("stm32/ultrasound", 1000);

	if (!simply_open(serial_port)) {
		ROS_ERROR("Unable to open the CAN serial port: %s", get_last_error().c_str());
//		return 2;
	}
	if (!simply_initialize_can(500)) {
		ROS_ERROR("Unable to initialize CAN: %s", get_last_error().c_str());
//		return 3;
	}
	if (!simply_start_can()) {
		ROS_ERROR("Unable to start CAN: %s", get_last_error().c_str());
//		return 4;
	}

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		can_msg_t can_msg;
		std::int8_t received;
		received = simply_receive(&can_msg);
		if (received == -1) {
//			ROS_ERROR("Error receiving CAN message: %s", get_last_error().c_str());
		} else if (received != 0) {
			switch (can_msg.ident) {
				case 0xf1: {
					std_msgs::Bool ros_msg;
					ros_msg.data = can_msg.payload[0];
					publisher_init.publish(ros_msg);
					break;
				}
				case 0xf3: {
					std_msgs::Float32 ros_msg;
					uint16_t dist_mm = (can_msg.payload[0] << 8) + can_msg.payload[1];
					ros_msg.data = float(dist_mm) / 10;
					publisher_init.publish(ros_msg);
					break;
				}
				default:
					ROS_INFO("Unknown message received: id %d\n", can_msg.ident);
					break;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

bool send_emergency_stop(emergency_stopRequest &request, emergency_stopResponse &response) {
	can_msg_t can_msg{};
	can_msg.ident = request.CAN_ID;
	can_msg.payload[0] = request.stop;
	can_msg.dlc = 1;
	bool success = simply_send(&can_msg);
	if (!success)
		ROS_ERROR("Unable to send emergency stop command: %s\n", get_last_error().c_str());
	return success;
}

bool send_brakes(brakesRequest &request, brakesResponse &response) {
	can_msg_t can_msg{};
	can_msg.ident = request.CAN_ID;
	can_msg.payload[0] = request.brakes_on;
	can_msg.dlc = 1;
	bool success = simply_send(&can_msg);
	if (!success)
		ROS_ERROR("Unable to send brakes command: %s\n", get_last_error().c_str());
	return success;
}

bool send_motor_power(motorRequest &request, motorResponse &response) {
	can_msg_t can_msg{};
	can_msg.ident = request.CAN_ID;
	can_msg.payload[0] = request.power;
	can_msg.dlc = 1;
	bool success = simply_send(&can_msg);
	if (!success)
		ROS_ERROR("Unable to send motor command: %s\n", get_last_error().c_str());
	return success;
}

bool send_steering_angle(steeringRequest &request, steeringResponse &response) {
	can_msg_t can_msg{};
	can_msg.ident = request.CAN_ID;
	can_msg.payload[0] = request.angle;
	can_msg.dlc = 1;
	bool success = simply_send(&can_msg);
	if (!success)
		ROS_ERROR("Unable to send steering command: %s\n", get_last_error().c_str());
	return success;
}


std::string get_last_error() {
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
