/*
 * speedtrap.h
 *
 *  Created on: Oct 8, 2025
 *      Author: pkr
 */

/*
 * A segment is 30cm long. We need some space for the flash. So lets say we have
 * 20cm distance for the leds. The cars reaches a maximum speed of round about
 * 4 m/s. That means it passes the segment at fullspeed in 50ms.
 *
 * The timer does 1 tick each 8us. So we have 125 ticks each ms.
 * We got at fullspeed 6250 tickss
 * Slowest lets say 1 m/s is 25000 ticks.
 *
 * The flash threshold should be adjustable between fullspeed and the half of it.
 * This leads to a difference of 6250 ticks. This is adjustable via analog in.
 */

/*
 * ISR & TIMER SETTINGS
 *
 * Max isr freq is 15.625kHz
 * Divided in 8 segments our min frame is 8us long.
 * This leads to 524288us until the timer overflows (16 bit)
 *
 * Timer for cc prescaler set to 383
 * Timer for low_res overflow protection and speed calc to 100Hz
 *
 */

#ifndef INC_SPEEDTRAP_H_
#define INC_SPEEDTRAP_H_

#include "stm32f4xx.h"

#define SPTR_DRIVER		6U
#define SPTR_GATES		4U
#define SPTR_BASES		3U
#define SPTR_DISTANCE	200U // in mm

typedef struct {
	uint8_t 	id;
	uint8_t 	in_section;
	uint16_t	time_entry_hires;
	uint16_t	time_entry_lores;
} vehicle;

typedef struct {
	uint8_t		pointer;
	uint16_t 	captures [SPTR_BASES];
	uint16_t	reset_timer;
	uint16_t	last_capture;
} segment_gate;

void 	sptr_init(void);
void 	sptr_set_trigger_sensitivity(uint16_t value);
uint8_t sptr_triggered(void);
uint8_t sptr_flash_active(uint16_t hundredth);
void 	sptr_isr_entry_identification_lane_1(uint16_t capture);
void	sptr_isr_exit_identification_lane_1(uint16_t capture);
void 	sptr_isr_entry_identification_lane_2(uint16_t capture);
void 	sptr_isr_exit_identification_lane_2(uint16_t capture);
void 	sptr_isr_overflow_timer_low_res(void);


#endif /* INC_SPEEDTRAP_H_ */
