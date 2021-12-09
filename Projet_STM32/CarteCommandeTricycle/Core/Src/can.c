#include "can.h"
#include "main.h"
#include "stm32l4xx_hal_can.h"

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	/*HAL_CAN_MspInit();
	__HAL_RCC_CANx_CLK_ENABLE();*/

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);//PA5 LED verte on pour debug
		Error_Handler();
	}else{

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);//PA5 LED verte on pour debug
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);//PA5 LED verte on pour debug
		HAL_Delay(1000);
	}

  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void CAN_config(void){

	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x320 << 5;           // Ici, 320 est l'adresse de la carte. Il peux être différent pour chaque carte.
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0xFFF << 5;       // Le masque peux servir à accepter une plage d'adresse au lieu d'une adresse unique.
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1);                                             // Démarre le périphérique CAN
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // Active le mode interruption

	TxHeader.StdId = 0x321;      // Détermine l'adresse du périphérique au quel la trame est destiné.
	                                // Si plusieurs périphériques sur le bus comprennent cette adresse dans leur filtre, ils recevront tous la trame.
	TxHeader.ExtId = 0x01;       // Adresse étendue, non utilisée dans note cas
	TxHeader.RTR = CAN_RTR_DATA; // Précise que la trame contient des données
	TxHeader.IDE = CAN_ID_STD;   // Précise que la trame est de type Standard
	TxHeader.DLC = 2;            // Précise le nombre d'octets de données que la trame transporte ( De 0 à 8 )
	TxHeader.TransmitGlobalTime = DISABLE;

}


