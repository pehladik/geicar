//
//  power.c
//  
//
//  Created by pehladik on 26/10/2019.
//

#include "power.h"

void bootstarp_power(){
    /* auto-maintien alim */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}


void shutdown_power(){
    //coupure alim
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}
