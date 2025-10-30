/*
 * manchester_bufferical_layer.h
 *
 *  Created on: Oct 28, 2025
 *      Author: philipp
 */

#ifndef INC_MANCHESTER_BUFFER_H_
#define INC_MANCHESTER_BUFFER_H_

#include "stm32f4xx.h"

#define MANCHESTER_GAP_TICKS       		150     // minimum ticks between messages
#define MANCHESTER_BUFFER_CIRC_BUF_LEN 	80U

extern volatile uint16_t 	buffer_circular [40];

void manchester_buffer_dma_tc(void);
void manchester_buffer_dma_ht(void);

uint16_t manchester_buffer_circ_next(uint16_t i);
uint8_t manchester_buffer_sector_free(uint8_t sector);
uint8_t manchester_buffer_sector_of(uint8_t idx);
uint8_t manchester_buffer_next_sector(uint8_t sector);
void manchester_buffer_clear_sector(uint8_t sector);
uint16_t manchester_buffer_get_capture(uint8_t index);

#endif /* INC_MANCHESTER_BUFFER_H_ */
