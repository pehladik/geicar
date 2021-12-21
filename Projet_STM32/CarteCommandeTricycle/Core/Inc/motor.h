#ifndef CARTECOMMANDETRICYCLE_MOTOR_H
#define CARTECOMMANDETRICYCLE_MOTOR_H

#include "stm32l4xx_hal.h"

extern uint32_t DAC_vitesse;

void motor_init(void);
void Moteur_off(void);
void Moteur_on(void);

#endif //CARTECOMMANDETRICYCLE_MOTOR_H
