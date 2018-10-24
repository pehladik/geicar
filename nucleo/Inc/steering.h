#ifndef _STERRING_H_
#define _STERRING_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"


/**
* Set the speed of the steering motor. Speed value has to be between 0 and 100
**/
void steering_set_speed(GPIO_PinState en_steering, int speed);

#endif
