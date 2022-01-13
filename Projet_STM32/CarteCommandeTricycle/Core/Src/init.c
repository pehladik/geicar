#include <uart.h>
#include "init.h"
#include "main.h"
#include "steering.h"
#include "brakes.h"
#include "motor.h"
#include "can.h"
#include "ultrasound.h"

void init(void) {
	// mise en maintien ON de l'alimentation tricycle
	HAL_GPIO_WritePin(Out_MAINTIEN_GPIO_Port, Out_MAINTIEN_Pin, GPIO_PIN_SET); // PC4
	// TODO: tests de sécurité à faire pour conserver le maintien

	steering_init();
	brakes_init();
	motor_init();
}

_Noreturn void Main(void) {
	uart_print("Starting...\r\n");
	init();
	CAN_start(&hcan1);

	uint8_t data[] = {1};
	CAN_send(&hcan1, 0xf1, data, 1);

	while (1) {
		float distance = getDistance_TRIG_INPUT();
		uart_print("Distance = %.1f cm\r\n", distance);
		HAL_Delay(200);
	}
}
