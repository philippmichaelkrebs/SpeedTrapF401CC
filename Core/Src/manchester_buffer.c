/*
 * manchester_bufferical_layer.c
 *
 *  Created on: Oct 28, 2025
 *      Author: philipp
 */

#include "manchester_buffer.h"

		volatile uint16_t 	buffer_circular [40] = {0};
static 	volatile uint16_t 	buffer_messages [MANCHESTER_BUFFER_CIRC_BUF_LEN] = {0};
static 	volatile uint8_t 	buffer_messages_sector = 0;
static 	volatile uint32_t 	buffer_sector_complete_trigger = 0;

uint16_t manchester_buffer_circ_next(uint16_t i) {
	uint16_t ret = i+1;
	return ret >= MANCHESTER_BUFFER_CIRC_BUF_LEN ? 0 : ret;
}

uint8_t manchester_buffer_sector_free(uint8_t sector){
	return buffer_sector_complete_trigger & (0x0001 << sector);
}

uint8_t manchester_buffer_sector_of(uint8_t idx){
	if (idx < 20)
		return 0;
	else if (idx < 40)
		return 1;
	else if (idx < 60)
		return 2;
	else
		return 3;
}

uint8_t manchester_buffer_next_sector(uint8_t sector){
	uint8_t sec = sector+1;
	return sec < 4 ? sec : 0;
}

void manchester_buffer_clear_sector(uint8_t sector){
	buffer_sector_complete_trigger &= ~(0x0001 << sector);
}

uint16_t manchester_buffer_get_capture(uint8_t index){
	return buffer_messages[index];
}

void manchester_buffer_dma_tc(void){
	buffer_messages_sector++;
	if (buffer_messages_sector > 3)
		buffer_messages_sector = 0;

	for (uint8_t buf_idx = 0; buf_idx < 20; buf_idx++)
		buffer_messages[buffer_messages_sector*20+buf_idx] = buffer_circular[buf_idx+20];

	buffer_sector_complete_trigger |= (0x0001 << buffer_messages_sector);

}

void manchester_buffer_dma_ht(void){
	buffer_messages_sector++;
	if (buffer_messages_sector > 3)
		buffer_messages_sector = 0;

	for (uint8_t buf_idx = 0; buf_idx < 20; buf_idx++)
		buffer_messages[buffer_messages_sector*20+buf_idx] = buffer_circular[buf_idx];

	buffer_sector_complete_trigger |= (0x0001 << buffer_messages_sector);

}

void manchester_buffer_init(void){
	/*
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)&TIM4->CCR4);
	  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)buffer_circular);
	  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, 40);
	  // --- 4. Enable interrupts for half/full transfer ---
	  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_6);
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
	  // --- 5. Enable DMA channel ---
	  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
	  // --- 6. Enable TIM3 DMA request on CC1 ---
	  LL_TIM_EnableDMAReq_CC4(TIM4);
	  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
	  // --- 7. Start timer ---
	  LL_TIM_EnableCounter(TIM4);
	  */
}
