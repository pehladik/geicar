#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32l4xx_hal.h"
#include "main.h"

char uartBuf[100];

void uart_print(char *format, ...) {
	va_list argp;
	va_start(argp, format);
	sprintf(uartBuf, format, argp);
	HAL_UART_Transmit(&huart2, (uint8_t *) uartBuf, strlen(uartBuf), 100);
	va_end(argp);
}
