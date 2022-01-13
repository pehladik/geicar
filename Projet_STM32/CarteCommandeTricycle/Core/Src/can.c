#include "main.h"
#include "brakes.h"
#include "steering.h"
#include "motor.h"
#include "uart.h"

void CAN_start(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_Start(hcan) != HAL_OK) {
		uart_print("Failed to start CAN bus\r\n");
		return;
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
		uart_print("Failed to configure CAN filter\r\n");
		return;
	}

	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		uart_print("Failed to configure CAN callback\r\n");
		return;
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
		uart_print("Failed to send CAN message\r\n");
		return;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
		uart_print("Failed to receive CAN message\r\n");
		return;
	}

	switch (header.StdId) {
		case 0x01:
			set_emergency_stop(data[0]);
			break;
		case 0x02:
			brakes_set(data[0]);
			break;
		case 0x03:
			motor_set_power(((float) data[0]) / 100);
			break;
		case 0x04:
			steering_turn(data[0]);
			break;
	}
}
