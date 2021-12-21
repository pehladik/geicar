#include "main.h"
#include "debug.h"

void CAN_start(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_Start(hcan) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
	canfilterconfig.FilterIdHigh = 0;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0;
	canfilterconfig.FilterMaskIdLow = 0;
	canfilterconfig.SlaveStartFilterBank = 20;

	if (HAL_CAN_ConfigFilter(hcan, &canfilterconfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}
}

void CAN_send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length) {
	CAN_TxHeaderTypeDef header;
	uint32_t mailbox;
	header.IDE = CAN_ID_STD;
	header.StdId = id;
	header.RTR = CAN_RTR_DATA;
	header.DLC = length;

	if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK) {
		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
		Error_Handler();
	}

	// test: enable or disable LED 2 when receiving msg with id 1
	if (header.StdId == 1) {
		set_debug_led(data[0]);
		// echo the msg
		uint8_t txData[] = {data[0]};
		CAN_send(hcan, 2, txData, 1);
	}
}
