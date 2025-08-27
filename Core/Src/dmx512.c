/*
 * dmx512.c
 *
 *  Created on: Jun 14, 2025
 *      Author: cyfin
 */

#include "dmx512.h"
#include "stm32c0xx_hal.h"

// No intermediate buffer; write directly to queue

UART_HandleTypeDef *huart_dmx512;

DMX512_FrameQueue rx_queue = { .r = 0, .w = 0 };

// this file will control USART1 directly with registers

void DMX512_Init(UART_HandleTypeDef *which_huart) {
	// enable all interrupts we care about
	USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE; // RX not empty interrupt
	USART1->CR1 |= USART_CR1_IDLEIE;		 // IDLE line detected interrupt
	USART1->CR2 |= USART_CR2_LBDIE;			 // LIN break detection interrupt

	USART1->CR1 |= USART_CR1_RE; // Enable receiver
}

static void DMX512_RxFlush() {
	DMX512_Frame *frame = &rx_queue.queue[rx_queue.w];
	if (frame->dmx_data_length > 0) {
		// Advance write index, handle overflow (overwrite oldest)
		rx_queue.w = (rx_queue.w + 1) % DMX512_FRAME_QUEUE_SIZE;
		// Reset for next frame
		frame = &rx_queue.queue[rx_queue.w];
		frame->dmx_data_length = 0;
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	}
}

int DMX512_GetFrame(DMX512_Frame **frame) {
	if (rx_queue.r == rx_queue.w) {
		return -1; // No new frame
	}
	*frame = &rx_queue.queue[rx_queue.r];
	rx_queue.r = (rx_queue.r + 1) % DMX512_FRAME_QUEUE_SIZE;
	return 0; // Success
}

void DMX512_Loop() {
	DMX512_Frame *frame = &rx_queue.queue[rx_queue.w];
	// wait for data to be received, detect idle line to reset counter
	if (USART1->ISR & USART_ISR_RXNE_RXFNE) {
		if (frame->dmx_data_length < DMX512_DATA_SIZE) {
			frame->dmx_data[frame->dmx_data_length++] = (uint8_t) (USART1->RDR & 0xFF); // Write directly to frame
		} else {
			DMX512_RxFlush(); // Queue frame if buffer full
		}
	}
	if (USART1->ISR & USART_ISR_LBDF) {
		USART1->ICR |= USART_ICR_LBDCF; // Clear the break detection flag
		DMX512_RxFlush();				// Queue frame on break detection
	} else if (USART1->ISR & USART_ISR_IDLE && frame->dmx_data_length > 0) {
		USART1->ICR |= USART_ICR_IDLECF; // Clear the idle line flag
		DMX512_RxFlush(); // Queue frame on idle line detection
	}
	// write pin A8 to the same value as SR NE
	if (USART1->ISR & USART_ISR_NE) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // Set pin A8 if NE flag is set
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Reset pin A8 if NE flag is not set
	}
}
