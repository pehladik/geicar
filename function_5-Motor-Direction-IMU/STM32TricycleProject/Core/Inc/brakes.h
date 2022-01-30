#ifndef CARTECOMMANDETRICYCLE_BRAKES_H
#define CARTECOMMANDETRICYCLE_BRAKES_H

#include <stdbool.h>
#include "stm32l4xx_hal.h"

/**
 * Initializes the brakes' pins and PWM
 */
void brakes_init(void);

/**
 *	Stops the vehicle with the shortest delay possible,
 *	and keeps it stopped until calling set_emergency_stop(false)
 *	@param state Whether to enable or disable the emergency stop
 */
void set_emergency_stop(bool state);

/**
 * Activate or deactivate the vehicle's brakes
 * @param brake true to brake, false to stop braking
 */
void brakes_set(bool brake);

#endif //CARTECOMMANDETRICYCLE_BRAKES_H
