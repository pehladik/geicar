#ifndef CARTECOMMANDETRICYCLE_DASHBOARD_H
#define CARTECOMMANDETRICYCLE_DASHBOARD_H

#include "stm32l4xx_hal.h"

/**
 * interruptions externes des boutons du tableau de bord
 * PB2  bouton STOP falling edge
 * PB11 bouton FREIN ON falling edge FREIN OFF rising edge
 * PC7  bouton LOCAL ON falling edge REMOTE rising edge
 * PB0 LEFT  falling edge
 * PB1 RIGHT falling edge
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif //CARTECOMMANDETRICYCLE_DASHBOARD_H
