#include <main.h>
#include "debug.h"
#include "stm32l4xx_hal.h"

void set_debug_led(int on) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
