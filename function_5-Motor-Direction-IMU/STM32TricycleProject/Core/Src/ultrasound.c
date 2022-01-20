#include "ultrasound.h"
#include "main.h"

#define usTIM TIM4

uint8_t icFlag = 0;
uint8_t captureIdx = 0;
uint32_t edge1Time = 0;
uint32_t edge2Time = 0;

const double speedOfSound = 0.0343 / 2;

void us_delay(uint32_t uSec) {
	if (uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1;  // sets the value in the auto-reload register
	usTIM->EGR = 1;         // Re-initialises the timer
	usTIM->SR &= ~1;        // Resets the flag
	usTIM->CR1 |= 1;        // Enables the counter
	while ((usTIM->SR & 0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

double ultrasound_get_distance_trig_input() {
	double distance;
	uint32_t numTicks = 0;
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
	us_delay(3);

	//*** START Ultrasonic measure routine ***//
	//1. Output 10 usec TRIGGER
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
	us_delay(10);
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

	//2. Wait for ECHO pin rising edge
	while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET);

	//3. Start measuring ECHO pulse width in usec
	numTicks = 0;
	while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
		numTicks++;
		us_delay(2); //2.8usec
	};

	//4. Estimate distance in cm
	distance = (numTicks + 0.0) * 3.0 * speedOfSound;
	return distance;

}

double ultrasound_get_distance() {
	double distance;
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
	us_delay(3);

	//*** START Ultrasonic measure routine ***//
	//1. Output 10 usec TRIG
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
	us_delay(10);
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

	//2. ECHO signal pulse width
	//Start IC timer
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	//Wait for IC flag
	uint32_t startTick = HAL_GetTick();
	do {
		if (icFlag) break;
	} while ((HAL_GetTick() - startTick) < 500);  //500ms
	icFlag = 0;
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

	//Calculate distance in cm
	if (edge2Time > edge1Time) {
		distance = ((edge2Time - edge1Time) + 0.0) * speedOfSound;
	} else {
		distance = -1.0f;
	}
	return distance;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (captureIdx == 0) {
		//First edge
		edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); //__HAL_TIM_GetCounter(&htim3);//
		captureIdx = 1;
	} else if (captureIdx == 1) {
		//Second edge
		edge2Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		captureIdx = 0;
		icFlag = 1;
	}
}


