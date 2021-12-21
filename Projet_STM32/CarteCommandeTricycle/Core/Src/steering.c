#include <main.h>
#include "stm32l4xx_hal.h"
#include "steering.h"

/// angle volant centré 1500 :
/// [1000 1500 2000] pour [0° 60° 120°]
uint32_t angle_volant = 1500;

void steering_init(void) {
	// start TIMER 2 PWM servo du volant
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA15
	// centrage du volant PWM servo
	TIM2->CCR1 = angle_volant; // angle volant centré 1500 :  [1000 1500 2000] pour [0° 60° 120°]
}

void turn_left(void) {
	angle_volant += increment_volant; // 5 * 0.12 = 0.6 °
	if (angle_volant > 2000) {
		angle_volant = 2000;
	}
	// envoi
	TIM2->CCR1 = angle_volant;
}

void turn_right(void) {
	angle_volant -= increment_volant; // 5 * 0.12 = 0.6 °
	if (angle_volant < 1000) {
		angle_volant = 1000;
	}
	// envoi
	TIM2->CCR1 = angle_volant;
}