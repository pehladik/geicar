//
//  power.c
//  
//
//  Created by pehladik on 26/10/2019.
//

#include "power.h"

void power_boostrap(){
    /* auto-maintien alim */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}


void power_shutdown(){
    //coupure alim
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}
