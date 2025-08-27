/*
 * dmx512.h
 *
 *  Created on: Jun 1, 2025
 *      Author: cyfin
 */

#ifndef INC_DMX512_H_
#define INC_DMX512_H_

#include "stm32c0xx_hal.h"

#define DMX512_DATA_SIZE 513
#define DMX512_FRAME_QUEUE_SIZE 3

typedef struct {
	uint8_t dmx_data[DMX512_DATA_SIZE];
	uint16_t dmx_data_length;
} DMX512_Frame;

typedef struct {
	DMX512_Frame queue[DMX512_FRAME_QUEUE_SIZE];
	uint8_t r; // read index
	uint8_t w; // write index
} DMX512_FrameQueue;

void DMX512_Init(UART_HandleTypeDef *which_huart);
int DMX512_GetFrame(DMX512_Frame **frame);
void DMX512_Loop(void);

#endif /* INC_DMX512_H_ */
