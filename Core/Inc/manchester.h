/*
 * manchester.h
 *
 *  Created on: Mar 16, 2025
 *      Author: philipp
 */

#ifndef INC_MANCHESTER_H_
#define INC_MANCHESTER_H_

#include "stm32f4xx.h"

#define HALF_BIT_TICKS   32
#define FULL_BIT_TICKS   64
#define GAP_THRESHOLD    132   // consider 160 as safer midpoint
#define GLITCH_FILTER    10    // ignore pulses shorter than ~16 µs

typedef enum{
	EDGE_RISING,
	EDGE_FALLING,
	EDGE_UNKNOWN
} EdgeDirection;

typedef struct {
	uint16_t data;
	uint16_t last_edge;
} ManchesterDecoder;

extern 	ManchesterDecoder 	data_manchester [2];
extern	uint16_t 			data_manchester_circular_read_index;
extern 	uint8_t				data_manchester_circular_index;
extern 	uint8_t				message_delay;
extern 	uint8_t				first_gap;
extern 	EdgeDirection		edge_direction;

void 		track_data_init(void);
void		track_data_circ_next();
uint8_t		track_data_decode(uint16_t capture);
uint16_t 	track_data_message(void);

#endif /* INC_MANCHESTER_H_ */
