#include <main.h>
#include "stm32l4xx_hal.h"
#include "steering.h"

/// angle volant centré 60°:
/// [1000 1500 2000] pour [0° 60° 120°]
int angle_volant = 60;
#define MIN_CCR 1000
#define MAX_CCR 2000

/// Incrément bouton left et right du volant, en degres
#define increment_volant 5

void steering_init(void) {
	// start TIMER 2 PWM servo du volant
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA15
	// centrage du volant PWM servo
	steering_turn(60);
}

void steering_turn_left(void) {
	steering_turn(angle_volant + increment_volant);
}

void steering_turn_right(void) {
	steering_turn(angle_volant - increment_volant);
}

void steering_turn(int angle) {
	if (angle < 0) {
		angle = 0;
	}
	if (angle > 120) {
		angle = 120;
	}
	angle_volant = angle;
	uint32_t ccr = angle * (MAX_CCR - MIN_CCR) / 120 + MIN_CCR;
	TIM2->CCR1 = ccr;
}