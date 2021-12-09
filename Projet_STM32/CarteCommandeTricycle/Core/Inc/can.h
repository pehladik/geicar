#include "stm32l4xx_hal.h"

void MX_CAN1_Init(void);

void CAN_config(void);

CAN_HandleTypeDef 		hcan1;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
