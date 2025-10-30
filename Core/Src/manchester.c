/*
 * manchester.c
 *
 *  Created on: Oct 28, 2025
 *      Author: philipp
 */


#include "manchester.h"


ManchesterDecoder 	data_manchester[2];
uint8_t				data_manchester_circular_index = 0;
uint8_t				message_delay;
uint8_t				first_gap;
EdgeDirection		edge_direction;
uint16_t			last_edge;

void track_data_init(void){
	for (uint8_t idx = 0; idx < 2; idx++){
		data_manchester[idx].data 			= 0x00;
	}
	last_edge = 0;
	message_delay = 0;
	first_gap = 1;
	edge_direction = EDGE_FALLING;
}

uint8_t track_data_decode(uint16_t capture){
	/*
	 * 32 ticks are 50 us. This shortens the isr execution time by day and night.
	 * This is because we can replace the division with a bit shift.
	 */
    uint16_t diff = capture - last_edge;
    if (diff <= GLITCH_FILTER) {
        return 0; // glitch: nothing to do
    }

    if (diff > 6000)
    	first_gap++;

    if (132 < diff){
    	// if last edge occurs more then 400us ago
    	edge_direction = EDGE_FALLING;
    	data_manchester_circular_index ^= 1; // switch buffer
    	last_edge = capture;
    	data_manchester[data_manchester_circular_index].data = 0x0001;
    	if (first_gap){
    		first_gap = 0;
    		return 0;
    	}
    	return 1;
    }
    else{
    	edge_direction = (EDGE_FALLING == edge_direction) ? EDGE_RISING : EDGE_FALLING;
    	if ((48 < diff) && (diff < 80)){
    		last_edge = capture;
    		data_manchester[data_manchester_circular_index].data <<= 1;
    		if (EDGE_FALLING == edge_direction)
    			data_manchester[data_manchester_circular_index].data |= 0x0001;
    	}
    }

	return 0;

}

uint16_t track_data_message(void){
	// return prev index
	uint8_t idx = data_manchester_circular_index == 0 ? 1 : 0;
	return data_manchester[idx].data;
}

