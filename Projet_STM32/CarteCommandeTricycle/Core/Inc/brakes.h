#ifndef CARTECOMMANDETRICYCLE_BRAKES_H
#define CARTECOMMANDETRICYCLE_BRAKES_H

#include "stm32l4xx_hal.h"

void ARRET_URGENCE(void);
void Freinage_on(void);
void Freinage_off(void);
void brakes_init(void);

#endif //CARTECOMMANDETRICYCLE_BRAKES_H
