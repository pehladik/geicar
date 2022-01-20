#include "brakes.h"
#include "main.h"
#include "motor.h"

/// état 1 si arret urgence
bool emergency_stop_state = 0;

/// Speed of the brake actuator: 0 -> 500 for 0 -> 100%
#define BRAKES_ACTUATOR_SPEED 500

void brakes_init(void) {
	// Activation de la commande BREAK par '0'
	HAL_GPIO_WritePin(Out_BREAK_GPIO_Port, Out_BREAK_Pin, GPIO_PIN_RESET); // PA7
	// start TIMER 3 PWM vérin du frein
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PC6
	// PWM du vérin de freinage off
	TIM3->CCR1 = 0; // 0 à 500 pour 0 à 100 %
	// Direction vérin rentré ou sorti
	HAL_GPIO_WritePin(Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, GPIO_PIN_SET); // PC9
}

void set_emergency_stop(bool state) {
	emergency_stop_state = state;
	if (state == true) {
		motor_set_power(0);
		brakes_set(true);
	} else {
		motor_set_power(1);
		brakes_set(false);
	}
}

void brakes_set(bool brake) {
	// Do not allow unbraking when emergency stop is active
	if (!brake && emergency_stop_state) {
		return;
	}

	// Activation du frein moteur (je crois ?) (activation à 0, desactivation a 1)
	HAL_GPIO_WritePin(Out_BREAK_GPIO_Port, Out_BREAK_Pin, !brake); //PA7

	// Direction du verin de freinage
	HAL_GPIO_WritePin(Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, brake); //PC9

	// Demarrage de la PWM pour le verin
	TIM3->CCR1 = BRAKES_ACTUATOR_SPEED;

	// TODO: prendre en compte capteurs de fin de course et prevoir timeout
	// Le code ci-dessous bloque jusqu'a ce que le capteur soit declenché, ce qui rend les démos fastidieuses

	// Attente du capteur de fin de course
//	GPIO_TypeDef *end_travel_sensor_port = brake ? Fin_de_course_in_GPIO_Port : Fin_de_course_out_GPIO_Port; // NOLINT(bugprone-branch-clone)
//	uint16_t end_travel_sensor_pin = brake ? Fin_de_course_in_Pin : Fin_de_course_out_Pin;
//	while (HAL_GPIO_ReadPin(end_travel_sensor_port, end_travel_sensor_pin)) {
//		// Direction du verin de freinage
//		HAL_GPIO_WritePin(Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, brake); //PC9
//
//		// Demarrage de la PWM pour le verin
//		TIM3->CCR1 = BRAKES_ACTUATOR_SPEED;
//	}

	// Arret de la PWM
//	TIM3->CCR1 = 0;
}
