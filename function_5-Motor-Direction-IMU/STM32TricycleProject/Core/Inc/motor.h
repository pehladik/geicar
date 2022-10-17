#ifndef CARTECOMMANDETRICYCLE_MOTOR_H
#define CARTECOMMANDETRICYCLE_MOTOR_H

#include "stm32l4xx_hal.h"

extern uint32_t DAC_vitesse;

/**
 * Initializes the motor's DAC
 */
void motor_init(void);

/**
 * Sets the power of the motor, in order to control its speed
 * @param power The desired power, between 0 and 1
 */
void motor_set_power(float power);

#endif //CARTECOMMANDETRICYCLE_MOTOR_H
