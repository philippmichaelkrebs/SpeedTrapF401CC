/*
 * race_state.h
 *
 *  Created on: Aug 25, 2025
 *      Author: philipp
 */

#ifndef INC_RACE_STATE_H_
#define INC_RACE_STATE_H_

#include "stm32f4xx.h"
#include "manchester.h"
#include "manchester_buffer.h"

typedef enum {
	CU_UNKNOWN,
	CU_SYSTEM_CALL,
	CU_SYSTEM_UNKNOWN,
	CU_SYSTEM_RESET,
	CU_START_LIGHT,
	CU_CONTROLLER_SAFETY_CAR,
	CU_CONTROLLER_DRIVER,
	CU_CONTROLLER_UNKNOWN,
	CU_CONTROLLER_SETTINGS,
	CU_CONTROLLER_JUMPSTART
} CuDataType;

typedef enum {
	RACE,
	QUALIFYING_TRAINING
}SessionType;

typedef enum {
	RACE_STATE_STARTUP,
	RACE_STATE_RED_FLAG,
	RACE_STATE_YELLOW_FLAG,
	RACE_STATE_GREEN_FLAG,
	RACE_STATE_START_PROC,
	RACE_STATE_JUMP_START,
	RACE_STATE_OPEN
} RaceState;

typedef struct {
	RaceState curr;
	RaceState prev;
} RaceStateHistory;

typedef struct {
	uint8_t red_count; 			// 0 - 5
	uint8_t red_count_prev;
	uint8_t yellow_flag; 		// 0 - 1
	uint8_t yellow_flag_prev;
	uint8_t green_flag; 		// 0 - 1
	uint8_t green_flag_prev;
	uint8_t update_flag;
} StartLightValue;

typedef struct {
	uint8_t 	fuel;
	uint8_t 	position;
	uint8_t 	finished;
	uint16_t 	laps;
	uint16_t 	laps_behind;
	uint8_t		red_flag_flag;
	uint8_t		joined_race;
} DriverData;

typedef struct {
	uint8_t tenths;
	uint8_t seconds_first;
	uint8_t seconds_second;
	uint8_t	minutes_first;
	uint8_t	minutes_second;
} TrackClock;

typedef struct {
	DriverData 	driver_data[6];
	uint8_t		wildcard;
	uint8_t		start_lights;
	uint16_t	lap_count;
	uint8_t		chaos;
	RaceState	race_state;
	SessionType	session_type;
	uint32_t	tenths;
	TrackClock	clock;
	uint8_t 	in_race_counter;
	//uint8_t	lap_count_nibble_completed;
} RaceInfo;

extern uint32_t				track_ticks;
extern uint32_t				tenths_calculator;
extern RaceInfo				race_info;
// race state
extern RaceStateHistory 	race_state;
extern StartLightValue 		start_light_value;
extern uint8_t 				safetycar_debounce; // neccessary because of ghost car messages
extern uint32_t 			green_flag_triggered_time;
extern uint32_t 			safetycar_flash_interval;
extern uint32_t 			green_flag_millis_flash;
extern uint32_t 			jump_start_millis_flash;

extern SessionType 			session_type;
extern uint32_t 			start_light_change_time;



void update_race_state(RaceStateHistory *stateHistory, RaceState newState);
void update_start_light_value_red(StartLightValue *startLightValue, uint8_t newValue);
void update_start_light_value_green(StartLightValue *startLightValue, uint8_t newValue);
void update_start_light_value_yellow(StartLightValue *startLightValue, uint8_t newValue);
void update_race_state_machine(CuDataType message_type, uint16_t value);
void update_race_state_machine_regular(void);
uint8_t update_race_info(uint16_t message); // returns 1 if status message received 0 otherwise
uint8_t update_race_info_status(void);

#endif /* INC_RACE_STATE_H_ */
