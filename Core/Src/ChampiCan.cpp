/*
 * ChampiCan.cpp
 *
 *  Created on: Feb 19, 2024
 *      Author: arusso
 */

#include <ChampiCan.h>

#include "stdio.h"
#include <string.h>

ChampiCan::ChampiCan(FDCAN_HandleTypeDef *handle_fdcan) {

	handle_fdcan_ = handle_fdcan;

	tx_header_.IdType = FDCAN_STANDARD_ID;
	tx_header_.TxFrameType = FDCAN_DATA_FRAME;
	tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header_.MessageMarker = 0;

}

ChampiCan::ChampiCan() = default;

int ChampiCan::start() {
	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(handle_fdcan_) != HAL_OK)
	{
		return 1;
	}
	if (HAL_FDCAN_ActivateNotification(handle_fdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		return 1;
	}
	return 0;
}

int ChampiCan::stop() {
    /* Stop the FDCAN module */
    int ret = 0;
    if (HAL_FDCAN_DeactivateNotification(handle_fdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != HAL_OK)
    {
        ret = 1;
    }
    if (HAL_FDCAN_Stop(handle_fdcan_) != HAL_OK)
    {
        ret = 1;
    }
    return ret;
}
int ChampiCan::send_frame(uint32_t id, uint8_t *frame_data, uint32_t size) {
	tx_header_.Identifier = id;
	tx_header_.DataLength = size;

    int ret = HAL_FDCAN_AddMessageToTxFifoQ(handle_fdcan_, &tx_header_, frame_data);

    if (ret == HAL_OK) {
        return 0;
    }

    /* We got an error, try again until it works. Also blink the LED at 2Hz */
    // Get led value to restore it after the loop
    GPIO_PinState led_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    unsigned long last_time = HAL_GetTick();
    while(ret != HAL_OK) {
        ret = HAL_FDCAN_AddMessageToTxFifoQ(handle_fdcan_, &tx_header_, frame_data);
        HAL_Delay(5);
        unsigned long now = HAL_GetTick();
        if(now - last_time > 500) {
              last_time = now;
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8); // The built-in LED
        }
    }
    // Restore the LED state
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, led_state);
    return 0;
}

int ChampiCan::send_msg(uint32_t id, uint8_t *msg, uint32_t msg_size) {

	static uint16_t msg_number = 0;

	if (msg_size > 512) {
		return 1;
	}

	uint8_t frame_data[8] = {0};
	uint16_t nb_frames = (uint16_t) msg_size / 6 + (msg_size % 6 > 0 ? 1 : 0);

	for(uint16_t i=0; i<nb_frames; i++) {

		// Frame descriptor
		uint16_t frame_descriptor = msg_number << 12 | (nb_frames << 6) | i;
		memcpy(frame_data, (char*)&frame_descriptor, 2);

		// Data size
		int num_bytes_frame = 6;
		if(i==nb_frames-1) {
			num_bytes_frame = msg_size % 6;
		}

		// Data
		memcpy(frame_data+2, msg + i*6, num_bytes_frame);

		// Send
		if(send_frame(id, frame_data, num_bytes_frame+2) != 0) {
			msg_number = (msg_number + 1) % 4;
			return 1;
		}
	}

    msg_number = (msg_number + 1) % 4;

    return 0;
}


ChampiCan::~ChampiCan() = default;

