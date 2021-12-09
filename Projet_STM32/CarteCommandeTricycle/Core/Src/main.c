/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "I2C.h"
#include "UART.h"
#include "timer.h"
#include "DAC_ADC.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/*ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/*static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);*/
/* USER CODE BEGIN PFP */
void ARRET_URGENCE(void);
void Demarrage(void);
void Freinage_on(void);
void Freinage_off(void);
void Moteur_on(void);
void Moteur_stop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Arret d'urgence */
uint8_t stat_arret_urg = 0; // état 1 si arret urgence pour sortir du process frein on

/* Gestion volant */
/* Incrément bouton left et right du volant                  */
/* delay de 20 ms fixe la duréé du pas (multiple de 20)      */
/* PAS = 0.12° X increment_volant                            */
uint32_t angle_volant = 1500;// angle volant centré 1500 :  [1000 1500 2000] pour[0° 60° 120°]

#define increment_volant 5 //5 X 0.12°
#define delay_volant 20 // en ms

/* Gestion vitesse */
/* 0 à 0xFFFFFF sur 12 bits, DAC OUT PA4 : 0 à 3.3V */
uint32_t DAC_vitesse = 0X000;

/* Gestion verin de freinage */
/* 0 à 500 pour 0 à 100 % */
uint32_t PWM_verin_frein = 0X000;

uint32_t b_frein = 0;		//Le frein pas activé
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /*MX_CAN1_Init();
  CAN_config();*/
  /* USER CODE BEGIN 2 */

  /* Premier démarrage de la carte */
  Demarrage();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_WritePin(Out_MAINTIEN_GPIO_Port, Out_MAINTIEN_Pin, GPIO_PIN_SET);//PC4
	  	/* tests de sécurité à faire pour conserver le maintien*/

	/*éteidre LED verte carte nucleo*/
	/*HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);//PA5 LED verte debug
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);//PA5 LED verte debug
	HAL_Delay(1000);*/
	/*if (b_frein==0){
		Moteur_on();
	}else
	{
		Moteur_stop();
	}*/

	//Serre le frein
	/*while (HAL_GPIO_ReadPin(Fin_de_course_in_GPIO_Port, Fin_de_course_in_Pin)) { // PB13, prevoir time out
			// rentre le vérin pour tirer le cable et freiner
			TIM3->CCR1 = 250; // 0 à 500 pour 0 à 100 %
			HAL_GPIO_WritePin( Direct_FREIN_GPIO_Port, Direct_FREIN_Pin,GPIO_PIN_SET); //PC9
			HAL_Delay(1000);
		}

	HAL_Delay(1000);*/

	//Relache le frein
	/*while ((HAL_GPIO_ReadPin(Fin_de_course_out_GPIO_Port, Fin_de_course_out_Pin)) && !(stat_arret_urg)) {//PB12, prevoir time out
		//sort le vérin pour relacher le cable et défreiner
		TIM3->CCR1 = 250; // 0 à 500 pour 0 à 100 %
		HAL_GPIO_WritePin( Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, GPIO_PIN_RESET);//PC9
		HAL_Delay(1000);
	}
	HAL_Delay(1000);*/

	//TIM3->CCR1 = 0; //PWM arrete

	//Start CAN Bus
	//HAL_CAN_Start();

	//Stop CAN Bus
	//HAL_CAN_Stop();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}


/* interruptions externes des boutons du tableau de bord*/
/*
 * PB2  bouton STOP falling edge
 * PB11 bouton FREIN ON falling edge FREIN OFF rising edge
 * PC7  bouton LOCAL ON falling edge REMOTE rising edge
 * PB0 LEFT  falling edge
 * PB1 RIGHT falling edge
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	uint32_t acctouche = 5, delay_ShutDown = 0; // accélération automatique appui touche, 5 vitesse lente et à 1 rapide
	HAL_Delay(100); // temps 100 ms pour anti-rebond
	if (GPIO_Pin == GPIO_PIN_2) { // STOP traitement fait avant appel callback
		while (!HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin)) { // touche STOP maintenue enfoncée pour Shut down alimentation
			delay_ShutDown++;
			if (delay_ShutDown > 5000000) { //
				/* mise OFF de l'alimentation tricycle*/
				HAL_GPIO_WritePin(Out_MAINTIEN_GPIO_Port, Out_MAINTIEN_Pin,GPIO_PIN_RESET); //PC4
				//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); //test
				// arret alim
			}
		}
	} else if (GPIO_Pin == GPIO_PIN_11) {    			//levier du frein
		if (HAL_GPIO_ReadPin(FREIN_GPIO_Port, FREIN_Pin)) {
			// levier frein off
			//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); //LED verte debug
			Freinage_off();
		} else {
			// levier frein on
			//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // LED verte debug
			Freinage_on();
		}
	} else if (GPIO_Pin == GPIO_PIN_7) {    			//levier local/remote
		if (HAL_GPIO_ReadPin(REMOTE_GPIO_Port, Remote_Pin)) {
			ARRET_URGENCE();
		} else {
			Moteur_on();
		}
		//
	} else if (GPIO_Pin == GPIO_PIN_0) {    		//left -->  1000 1500 2000
		while (!HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin)) { // touche left enfoncée
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			angle_volant += increment_volant; //5 X 0.12 = 0.6 °
			if (angle_volant > 2000) {
				angle_volant = 2000;
			}
			TIM2->CCR1 = angle_volant; // envoi
			HAL_Delay(acctouche * delay_volant); //5 X 20 ms durée du pas
			acctouche--;
			if (acctouche < 1)
				acctouche = 1;
		}

	} else if (GPIO_Pin == GPIO_PIN_1) { //right -->  2000 1500 1000
		while (!HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin)) { // touche right enfoncée
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			angle_volant -= increment_volant; ////5 X 0.12 = 0.6 °
			if (angle_volant < 1000) {
				angle_volant = 1000;
			}
			TIM2->CCR1 = angle_volant; // envoi
			HAL_Delay(acctouche * delay_volant); //5 X 20 ms durée du pas
			acctouche--;
			if (acctouche < 1)
				acctouche = 1;
		}

	}
}



void Demarrage(void)
{

	/* AUTOMAINTIEN */
	/* mise en maintien  ON de l'alimentation tricycle*/
	HAL_GPIO_WritePin(Out_MAINTIEN_GPIO_Port, Out_MAINTIEN_Pin, GPIO_PIN_SET);//PC4
	/* tests de sécurité à faire pour conserver le maintien*/
	/*.............................................................*/

	/* VOLANT */
	/* start TIMER 2 PWM servo du volant */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PA15
	/* centrage du volant PWM servo*/
	TIM2->CCR1 = angle_volant;// angle volant centré 1500 :  [1000 1500 2000] pour[0° 60° 120°]

	/*FREIN */
	/* Activation de la commande BREAK par '0' */
	HAL_GPIO_WritePin( Out_BREAK_GPIO_Port,  Out_BREAK_Pin, GPIO_PIN_RESET);//PA7
	/* start TIMER 3 PWM vérin du frein */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//PC6
	/* PWM du vérin de freinage off*/
	TIM3->CCR1 = 0; // 0 à 500 pour 0 à 100 %
	/* Direction vérin rentré ou sorti */
	HAL_GPIO_WritePin( Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, GPIO_PIN_SET);//PC9

	/* VITESSE */
	/* start  DAC vitesse */
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R, DAC_vitesse);//PA4 écriture 12 bits sur sortie
	HAL_DAC_Start(&hdac1,  DAC_CHANNEL_1);// démarrage une fois
	/*commande vitesse nulle DAC OUT*/
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0x000);// PA4 vitesse nulle

	/* DIVERS */
	/*allumer LED verte carte nucleo*/
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);//PA5 LED verte debug	//GPIO_PIN_RESET pour éteindre

}

void ARRET_URGENCE(void)
{
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);//PA5 LED verte on pour debug
	/*FREIN */
	/* Activation de la commande BREAK par '0' */
	HAL_GPIO_WritePin( Out_BREAK_GPIO_Port,  Out_BREAK_Pin, GPIO_PIN_RESET);//PA7
	/*commande vitesse nulle DAC OUT*/
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0x000);// PA4 vitesse nulle 0x000000 à 0xFFFFFF / 0 à 3.3V
	Freinage_on();

}

void Freinage_on(void) {
	/* Activation de la commande BREAK par '0' */
	HAL_GPIO_WritePin( Out_BREAK_GPIO_Port, Out_BREAK_Pin, GPIO_PIN_RESET); //PA7
	/*commande vitesse nulle DAC OUT*/
	//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x000); // PA4 vitesse nulle 0x000000 à 0xFFFFFF / 0 à 3.3V
	while (HAL_GPIO_ReadPin(Fin_de_course_in_GPIO_Port, Fin_de_course_in_Pin)) { // PB13, prevoir time out
		/* rentre le vérin pour tirer le cable et freiner */
		TIM3->CCR1 = 250; // 0 à 500 pour 0 à 100 %
		HAL_GPIO_WritePin( Direct_FREIN_GPIO_Port, Direct_FREIN_Pin,GPIO_PIN_SET); //PC9
	}

	TIM3->CCR1 = 0; //PWM arrete

}
void Freinage_off(void)
{
	/* Desactivation de la commande BREAK par '1' */
	HAL_GPIO_WritePin( Out_BREAK_GPIO_Port,  Out_BREAK_Pin, GPIO_PIN_SET);//PA7
	while ((HAL_GPIO_ReadPin(Fin_de_course_out_GPIO_Port, Fin_de_course_out_Pin)) && !(stat_arret_urg)) {//PB12, prevoir time out
		/* sort le vérin pour relacher le cable et défreiner */
		TIM3->CCR1 = 250; // 0 à 500 pour 0 à 100 %
		HAL_GPIO_WritePin( Direct_FREIN_GPIO_Port, Direct_FREIN_Pin, GPIO_PIN_RESET);//PC9
	}
	/* a retirer : */
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0x0FFF);//3.3V
	TIM3->CCR1 = 0; //PWM arrete
	stat_arret_urg = 0; // à voir si utile ! evite un retrait accidentel du frein en cas de stop
}

void Moteur_on(void){
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0xFFF);// PA4 vitesse nulle 0x000000 à 0xFFFFFF / 0 à 3.3V
}

void Moteur_off(void){
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1,DAC_ALIGN_12B_R, 0x000);// PA4 vitesse nulle 0x000000 à 0xFFFFFF / 0 à 3.3V
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
