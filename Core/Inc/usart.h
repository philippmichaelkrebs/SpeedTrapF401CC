
/*
 * manchester.h
 *
 *  Created on: Mar 16, 2025
 *      Author: philipp
 *
 *  Interface to the base controller and the screen controller
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f4xx.h"
#include "race_state.h"


#define UART_TX_BUF_LEN		(20U) //32U
#define UART_RX_BUF_LEN		(9U) //32U


typedef struct {
	uint8_t 	race_state; // ; 8: pause
	uint8_t 	start_lights;
	uint8_t 	jump_start_id;
	uint8_t 	speed_trap_settings; // 1:on/off ; 2:faster then / slower then
	uint8_t 	speed_trap_sensitivity;
	uint32_t 	track_ticks;

} USART_BASE_DATA;

typedef struct {
	uint8_t 	race_state;
	uint8_t 	start_lights;
	uint8_t 	jump_start_id;
	uint8_t 	pause;
	uint32_t 	track_ticks;
	uint16_t	speed_id0_ticks;
	uint16_t	speed_id1_ticks;
	uint16_t	speed_id2_ticks;
	uint16_t	speed_id3_ticks;
	uint16_t	speed_id4_ticks;
	uint16_t	speed_id5_ticks;
} USART_DISPLAY_DATA;

#endif /* INC_USART_H_ */
