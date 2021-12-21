/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Mes_48V_Pin GPIO_PIN_0
#define Mes_48V_GPIO_Port GPIOC
#define Mes_5V_Pin GPIO_PIN_1
#define Mes_5V_GPIO_Port GPIOC
#define Mes_12V_Pin GPIO_PIN_2
#define Mes_12V_GPIO_Port GPIOC
#define Mes_14_8V_Pin GPIO_PIN_3
#define Mes_14_8V_GPIO_Port GPIOC
#define Mes_JAUGE_Pin GPIO_PIN_0
#define Mes_JAUGE_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Mes_OPT_Pin GPIO_PIN_6
#define Mes_OPT_GPIO_Port GPIOA
#define Out_BREAK_Pin GPIO_PIN_7
#define Out_BREAK_GPIO_Port GPIOA
#define Out_MAINTIEN_Pin GPIO_PIN_4
#define Out_MAINTIEN_GPIO_Port GPIOC
#define In_ETAT_POUSSOIR_Pin GPIO_PIN_5
#define In_ETAT_POUSSOIR_GPIO_Port GPIOC
#define LEFT_Pin GPIO_PIN_0
#define LEFT_GPIO_Port GPIOB
#define LEFT_EXTI_IRQn EXTI0_IRQn
#define RIGHT_Pin GPIO_PIN_1
#define RIGHT_GPIO_Port GPIOB
#define RIGHT_EXTI_IRQn EXTI1_IRQn
#define STOP_Pin GPIO_PIN_2
#define STOP_GPIO_Port GPIOB
#define STOP_EXTI_IRQn EXTI2_IRQn
#define FREIN_Pin GPIO_PIN_11
#define FREIN_GPIO_Port GPIOB
#define FREIN_EXTI_IRQn EXTI15_10_IRQn
#define Fin_de_course_out_Pin GPIO_PIN_12
#define Fin_de_course_out_GPIO_Port GPIOB
#define Fin_de_course_in_Pin GPIO_PIN_13
#define Fin_de_course_in_GPIO_Port GPIOB
#define PWM_FREIN_Pin GPIO_PIN_6
#define PWM_FREIN_GPIO_Port GPIOC
#define Remote_Pin GPIO_PIN_7
#define Remote_GPIO_Port GPIOC
#define Remote_EXTI_IRQn EXTI9_5_IRQn
#define In_opt_Pin GPIO_PIN_8
#define In_opt_GPIO_Port GPIOC
#define Direct_FREIN_Pin GPIO_PIN_9
#define Direct_FREIN_GPIO_Port GPIOC
#define Incapt_T1_Hall_Pin GPIO_PIN_8
#define Incapt_T1_Hall_GPIO_Port GPIOA
#define Incapt_T1_OPT_Pin GPIO_PIN_9
#define Incapt_T1_OPT_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PWM_SERVO_Pin GPIO_PIN_15
#define PWM_SERVO_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern DAC_HandleTypeDef hdac1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
