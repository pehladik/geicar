#ifndef CARTECOMMANDETRICYCLE_CAN_H
#define CARTECOMMANDETRICYCLE_CAN_H

#include "stm32l4xx_hal.h"

void CAN_start(CAN_HandleTypeDef *hcan);
void CAN_send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t length);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif //CARTECOMMANDETRICYCLE_CAN_H
