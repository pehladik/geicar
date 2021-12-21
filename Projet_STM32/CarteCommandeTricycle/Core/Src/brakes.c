#include "brakes.h"
#include "main.h"
#include "motor.h"

/// état 1 si arret urgence pour sortir du process frein on
uint8_t stat_arret_urg = 0;

/// Gestion verin de freinage
/// de 0 a 500 pour 0 à 100%
uint32_t PWM_verin_frein = 0x000;

/// Le frein pas active
uint32_t b_frein = 0;

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

void ARRET_URGENCE(void) {
	Moteur_off();
	Freinage_on();
}

void Freinage_on(void) {
	// Activation de la commande BREAK par '0'
	HAL_GPIO_WritePin(Out_BREAK_GPIO_Port, Out_BREAK_Pin, GPIO_PIN_RESET); //PA7
	// commande vitesse nulle DAC OUT
	// HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x000); // PA4 vitesse nulle 0x000000 à 0xFFFFFF / 0 à 3.3V
	while (HAL_GPIO_ReadPin(Fin_de_course_in_GPIO_Port, Fin_de_course_in_Pin)) { // PB13, prevoir time out
		// rentre le vérin pour tirer le cable et freiner
		TIM3->CCR1 = 250; // 0 à 500 pour 0 à 100 %
		HAL_GPIO_WritePin(Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, GPIO_PIN_SET); //PC9
	}

	TIM3->CCR1 = 0; //PWM arrete

}

void Freinage_off(void) {
	// Desactivation de la commande BREAK par '1'
	HAL_GPIO_WritePin(Out_BREAK_GPIO_Port, Out_BREAK_Pin, GPIO_PIN_SET); //PA7
	while ((HAL_GPIO_ReadPin(Fin_de_course_out_GPIO_Port, Fin_de_course_out_Pin)) && !(stat_arret_urg)) { //PB12, prevoir time out
		// sort le vérin pour relacher le cable et défreiner
		TIM3->CCR1 = 250; // 0 à 500 pour 0 à 100 %
		HAL_GPIO_WritePin(Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, GPIO_PIN_RESET);//PC9
	}
	// TODO: a retirer
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x0FFF);//3.3V
	TIM3->CCR1 = 0; // PWM arrete
	stat_arret_urg = 0; // à voir si utile ! evite un retrait accidentel du frein en cas de stop
}
