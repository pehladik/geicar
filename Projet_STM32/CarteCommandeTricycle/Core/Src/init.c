#include "init.h"
#include "main.h"
#include "steering.h"
#include "brakes.h"
#include "motor.h"
#include "can.h"

void Demarrage(void) {
	// mise en maintien ON de l'alimentation tricycle
	HAL_GPIO_WritePin(Out_MAINTIEN_GPIO_Port, Out_MAINTIEN_Pin, GPIO_PIN_SET); // PC4
	// TODO: tests de sécurité à faire pour conserver le maintien

	steering_init();
	brakes_init();
	motor_init();
}

void Main(void) {
	Demarrage();
	CAN_start(&hcan1);

	uint8_t data[] = {1};
	CAN_send(&hcan1, 3, data, 1);
}
