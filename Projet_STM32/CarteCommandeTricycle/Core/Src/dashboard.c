#include "dashboard.h"
#include "main.h"
#include "brakes.h"
#include "motor.h"
#include "steering.h"

/// delay de 20 ms fixe la duree du pas (multiple de 20)
#define delay_volant 20

void stop_button_callback() {
	uint32_t delay_ShutDown = 0;
	while (!HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin)) { // touche STOP maintenue enfoncée pour Shut down alimentation
		delay_ShutDown++;
		if (delay_ShutDown > 5000000) { //
			// mise OFF de l'alimentation tricycle
			HAL_GPIO_WritePin(Out_MAINTIEN_GPIO_Port, Out_MAINTIEN_Pin, GPIO_PIN_RESET); //PC4
		}
	}
}

void brakes_button_callback() {
	brakes_set(!HAL_GPIO_ReadPin(FREIN_GPIO_Port, FREIN_Pin));
}

// TODO: change this to be the actual remote/local switch, to select between
//  controlling the tricycle with the dashboard or the CAN bus
void remote_button_callback() {
	const GPIO_PinState pin_state = HAL_GPIO_ReadPin(Remote_GPIO_Port, Remote_Pin);
	set_emergency_stop(pin_state);
}

void turn_button_callback(uint16_t btn_pin) {
	uint32_t delay_multiplier = 5; // accélération automatique appui touche, 5 vitesse lente et à 1 rapide
	while (!HAL_GPIO_ReadPin(LEFT_GPIO_Port, btn_pin)) { // touche left enfoncée
		if (btn_pin == LEFT_Pin) {
			steering_turn_left();
		} else {
			steering_turn_right();
		}
		HAL_Delay(delay_multiplier * delay_volant); //5 X 20 ms durée du pas
		delay_multiplier--;
		if (delay_multiplier < 1)
			delay_multiplier = 1;
	}
}


/**
 * interruptions externes des boutons du tableau de bord
 * PB2  bouton STOP falling edge
 * PB11 bouton FREIN ON falling edge FREIN OFF rising edge
 * PC7  bouton LOCAL ON falling edge REMOTE rising edge
 * PB0 LEFT  falling edge
 * PB1 RIGHT falling edge
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// TODO: deactivate in "remote" mode
	HAL_Delay(100); // temps 100 ms pour anti-rebond

	switch (GPIO_Pin) {
		case STOP_Pin:      // STOP traitement fait avant appel callback
			stop_button_callback();
			break;
		case FREIN_Pin:     // levier du frein
			brakes_button_callback();
			break;
		case Remote_Pin:    // levier local/remote
			remote_button_callback();
			break;
		case LEFT_Pin:      // left
		case RIGHT_Pin:     // right
			turn_button_callback(GPIO_Pin);
			break;
	}
}

