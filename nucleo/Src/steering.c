#include "steering.h"

#define MAX_SPEED_STEERING 60
#define MIN_SPEED_STEERING 40 

extern uint32_t ADCBUF[5];

void steering_set_speed(GPIO_PinState en_steering, int speed){
	
		/* Threshold */
		/* The speed */
		if (speed < MIN_SPEED_STEERING){
			speed = MIN_SPEED_STEERING;
		} else if (speed > MAX_SPEED_STEERING){
			speed  = MAX_SPEED_STEERING;
		}
		
		speed = 3200 * ( speed/ 100.0 );
		
		TIM1->CCR3 = speed;
		
		/*        Enable moteurs        */
		/* GPIO_PIN_SET : activation    */
		/* GPIO_PIN_RESET : pont ouvert */
		HAL_GPIO_WritePin( GPIOC, GPIO_PIN_12, en_steering);  //PC12  AV
}

int get_steering_angle(void){
	return ADCBUF[1];
}

