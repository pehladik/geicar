//
//  power.h
//  
//
//  Created by pehladik on 26/10/2019.
//

#ifndef power_h
#define power_h

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

/**
* Set the power
**/
void power_boostrap();

/**
* shutdown the power
**/
void power_shutdown();


#endif /* power_h */
