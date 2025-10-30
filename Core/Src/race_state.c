/*
 * race_state.c
 *
 *  Created on: Sep 4, 2025
 *      Author: philipp
 */


#include "stm32f4xx.h"
#include "stm32f4xx_ll_dma.h"
#include "race_state.h"

// race state
uint32_t			track_ticks = 0;
uint32_t			tenths_calculator = 0;
RaceStateHistory 	race_state = {RACE_STATE_STARTUP, RACE_STATE_STARTUP};
StartLightValue 	start_light_value = {0};
uint8_t 			safetycar_debounce = 0; // necessary because of ghost car messages
uint32_t 			green_flag_triggered_time = 0;
uint32_t 			safetycar_flash_interval = 0;
uint32_t 			green_flag_millis_flash = 0;
uint32_t 			jump_start_millis_flash = 0;
SessionType 		session_type = RACE;
uint32_t 			start_light_change_time = 0;
RaceInfo	race_info = {
		.driver_data = {
				{ .fuel = 0, .position = 255, .finished = 0, .laps = 0, .laps_behind = 0, .red_flag_flag = 0, .joined_race = 0  },
				{ .fuel = 0, .position = 255, .finished = 0, .laps = 0, .laps_behind = 0, .red_flag_flag = 0, .joined_race = 0  },
				{ .fuel = 0, .position = 255, .finished = 0, .laps = 0, .laps_behind = 0, .red_flag_flag = 0, .joined_race = 0  },
				{ .fuel = 0, .position = 255, .finished = 0, .laps = 0, .laps_behind = 0, .red_flag_flag = 0, .joined_race = 0  },
				{ .fuel = 0, .position = 255, .finished = 0, .laps = 0, .laps_behind = 0, .red_flag_flag = 0, .joined_race = 0  },
				{ .fuel = 0, .position = 255, .finished = 0, .laps = 0, .laps_behind = 0, .red_flag_flag = 0, .joined_race = 0  }
		},
		.wildcard 			= 0xFF,       //
		.start_lights 		= 0,
		.lap_count 			= 0,
		.chaos 				= 0,
		.race_state 		= RACE_STATE_STARTUP,
		.session_type 		= RACE,
		.tenths				= 0,
		.clock				= {.tenths = 0, .seconds_first = 0, .seconds_second = 0, .minutes_first = 0, .minutes_second = 0},
		.in_race_counter	= 0
		//.lap_count_nibble_completed = 1,
};

static 	uint8_t		manchester_buf_cursor	= 255;


void update_race_state(RaceStateHistory *stateHistory, RaceState newState) {
	stateHistory->prev = stateHistory->curr;
	stateHistory->curr = newState;
	race_info.race_state = newState;
}

void update_start_light_value_red(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->red_count_prev = startLightValue->red_count;
	startLightValue->red_count = newValue;
	startLightValue->update_flag = 1;
	race_info.start_lights = newValue;
}

void update_start_light_value_green(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->green_flag_prev = startLightValue->green_flag;
	startLightValue->green_flag = newValue;
	startLightValue->update_flag = 1;
}

void update_start_light_value_yellow(StartLightValue *startLightValue, uint8_t newValue) {
	startLightValue->yellow_flag_prev = startLightValue->yellow_flag;
	startLightValue->yellow_flag = newValue;
	startLightValue->update_flag = 1;
}

void update_race_state_machine(CuDataType message_type, uint16_t value){
	// set start light

	if (CU_START_LIGHT == message_type){
		update_start_light_value_red(&start_light_value, value);

		if (value == 5 ){
			if (race_state.curr != RACE_STATE_START_PROC) // if start proc then it relates to it
				update_race_state(&race_state, RACE_STATE_RED_FLAG);
		} else if (value == 1) //
			update_race_state(&race_state, RACE_STATE_START_PROC);
		else if (value == 0) {
			if  ((race_state.curr == RACE_STATE_START_PROC) && race_state.prev == RACE_STATE_RED_FLAG){
				start_light_change_time = track_ticks;
			}
			else if (race_state.curr == RACE_STATE_RED_FLAG){
				update_race_state(&race_state, RACE_STATE_GREEN_FLAG);
				green_flag_triggered_time = track_ticks;
			}
		}
	}

	if (CU_CONTROLLER_JUMPSTART == message_type){
		update_race_state(&race_state, RACE_STATE_JUMP_START);
	}

	// check for yellow flag
	if (CU_CONTROLLER_SAFETY_CAR == message_type){
		if (value && (race_state.curr != RACE_STATE_YELLOW_FLAG))
			safetycar_debounce++;

		if (!value && (race_state.curr == RACE_STATE_YELLOW_FLAG))
			safetycar_debounce--;

		if ((1 < safetycar_debounce) && (race_state.curr != RACE_STATE_YELLOW_FLAG))
			update_race_state(&race_state, RACE_STATE_YELLOW_FLAG);

		if ((safetycar_debounce == 0) && (race_state.curr == RACE_STATE_YELLOW_FLAG)){
			update_race_state(&race_state, RACE_STATE_GREEN_FLAG);
			green_flag_triggered_time = track_ticks;
		}
	}
}

void update_race_state_machine_regular(void){

	/*
	 * Update the lights and update race state if required (green flag)
	 */


	// release track
	if (RACE_STATE_START_PROC == race_state.curr){
		if (!start_light_value.red_count && (track_ticks - start_light_change_time > 6)){
			update_race_state(&race_state, RACE_STATE_OPEN);
		}
	}

	// start light value changed
	if (start_light_value.red_count != start_light_value.red_count_prev)
		update_start_light_value_red(&start_light_value, start_light_value.red_count);

	// SAFETY CAR
	if (RACE_STATE_YELLOW_FLAG == race_state.curr){
		if (track_ticks - safetycar_flash_interval > 3){ // (millis - safetycar_flash_interval > 200){
			safetycar_flash_interval = track_ticks;
			if (0 == start_light_value.yellow_flag)
				update_start_light_value_yellow(&start_light_value, 1);
			else
				update_start_light_value_yellow(&start_light_value, 0);
		}
	} else {
		if (start_light_value.yellow_flag && (RACE_STATE_JUMP_START != race_state.curr)){
			update_start_light_value_yellow(&start_light_value, 0);
			update_start_light_value_yellow(&start_light_value, 0);
		}
	}

	// green flag
	if (RACE_STATE_GREEN_FLAG == race_state.curr){
		if (track_ticks - green_flag_millis_flash > 7){ // (millis - green_flag_millis_flash > 500)
			if (!start_light_value.green_flag){
				green_flag_millis_flash = track_ticks;
				update_start_light_value_green(&start_light_value, 1);
			}else {
				green_flag_millis_flash = track_ticks;
				update_start_light_value_green(&start_light_value, 0);
			}
		}

		if (track_ticks - green_flag_triggered_time > 142) //millis - green_flag_triggered_time > 10000
			update_race_state(&race_state, RACE_STATE_OPEN);
	}else{
		if ((1 == start_light_value.green_flag) && (RACE == session_type))
			update_start_light_value_green(&start_light_value, 0);
		else if ((0 == start_light_value.green_flag) && (QUALIFYING_TRAINING == session_type) && (RACE_STATE_OPEN == race_state.curr))
			update_start_light_value_green(&start_light_value, 1);
		else if ((1 == start_light_value.green_flag) && (QUALIFYING_TRAINING == session_type) && (RACE_STATE_OPEN != race_state.curr))
			update_start_light_value_green(&start_light_value, 0);
	}


	// JUMP START
	if (RACE_STATE_JUMP_START == race_state.curr){
		if (track_ticks - jump_start_millis_flash > 7){ //(millis - jump_start_millis_flash > 500)
			jump_start_millis_flash = track_ticks;
			update_start_light_value_red(&start_light_value, 5);
			if (!start_light_value.yellow_flag)
				update_start_light_value_yellow(&start_light_value, 1);
			else
				update_start_light_value_yellow(&start_light_value, 0);
		}
	}
}

static void track_timer(){
	if ( 	(RACE_STATE_OPEN == race_state.curr) ||
			(RACE_STATE_GREEN_FLAG == race_state.curr) ||
			(RACE_STATE_YELLOW_FLAG == race_state.curr) ){
		tenths_calculator++;
	} else if (	(RACE_STATE_START_PROC == race_state.curr) &&
				(0 == start_light_value.red_count) ){
		// don't miss the first tenths after start!
		// downside is the calculation during jump start
		tenths_calculator++;
	}
}

uint8_t update_race_info(uint16_t message){


	/*
	 * Process
	 *
	 * Filter the message of interest here
	 *
	 * For the lights we need safety car and status messages.
	 *
	 * If the safety car is released, choose this one. Otherwise we only
	 * need the status messages.
	 *
	 * Status messages
	 * 	lights should be indicated with 0x17 (and)
	 * 	jump start indicated with (((0x000F << 8) & message) == (0x001 << 11)) && ((0x000D << 4) & message)
	 *
	 * Safety car message
	 * 	indicated with 0b0000001111000000;
	 */

	uint8_t status_message_identifier 	= 0;
	uint8_t lap_finished_identifier 	= 255;
	if (( message > 512-1 ) && (message < 1024)){
		// car message
		uint16_t car_id = message & (0x0007 << 6);
		//uint8_t speed = message & (0x000F << 1);

		if ( car_id == (0x0007 << 6)){
			// safety car message (id 7)
			update_race_state_machine(CU_CONTROLLER_SAFETY_CAR, (message & (0x01 << 1)));
		} else if ( !(message & (0x0007 << 6)) ){
			// car with id 0
			//tenths_calculator++;
			track_timer();
		} else if ( car_id == (0x0005 << 6) ){
			// car with id 5
			//tenths_calculator++;
			track_timer();
		}
	} else if ((message > 4096-1) && (message < 8192)){
		// control unit message
		track_ticks++; // message block repeats every 75ms
		//tenths_calculator++;
		track_timer();
		status_message_identifier = 1;

		uint16_t encode_signal = message;
		uint8_t controller =
				((encode_signal >> 0) & 0x01) << 2 |
				((encode_signal >> 1) & 0x01) << 1 |
				((encode_signal >> 2) & 0x01) << 0;

		uint8_t command =
				((encode_signal >> 3) & 0x01) << 4 |
				((encode_signal >> 4) & 0x01) << 3 |
				((encode_signal >> 5) & 0x01) << 2 |
				((encode_signal >> 6) & 0x01) << 1 |
				((encode_signal >> 7) & 0x01) << 0;

		uint8_t value =
				((encode_signal >> 8) & 0x01) << 3 |
				((encode_signal >> 9) & 0x01) << 2 |
				((encode_signal >> 10) & 0x01) << 1 |
				((encode_signal >> 11) & 0x01) << 0;

		if ((11 == command) && (1 == value)){
			// jump start
			update_race_state_machine(CU_CONTROLLER_JUMPSTART, 1);
			race_info.wildcard = controller;
		} else if ((16 == command) && (7 == controller)){
			// start light
			if ((RACE_STATE_START_PROC == race_state.curr) && (!value)){
				race_info.tenths  	= 0;
				tenths_calculator = 0;
				race_info.clock.tenths = 0;
				race_info.clock.seconds_first = 0;
				race_info.clock.seconds_second = 0;
				race_info.clock.minutes_first = 0;
				race_info.clock.minutes_second = 0;
			}
			update_race_state_machine(CU_START_LIGHT, value);

			// handle reset
			if ((1==value) && (1 == race_info.chaos)){
				// race start after chaos needs a reset
				tenths_calculator 				= 0;
				race_info.chaos					= 0;
				race_info.lap_count       		= 0;
				race_info.wildcard  			= 0xFF;
				race_info.in_race_counter		= 0;
				race_info.tenths  				= 0;
				race_info.clock.tenths = 0;
				race_info.clock.seconds_first = 0;
				race_info.clock.seconds_second = 0;
				race_info.clock.minutes_first = 0;
				race_info.clock.minutes_second = 0;
				for (uint8_t _cntrl = 0; _cntrl < 6; _cntrl++){
					race_info.driver_data[_cntrl].finished = 0;
					race_info.driver_data[_cntrl].position = 255;
					race_info.driver_data[_cntrl].fuel = 0;
					race_info.driver_data[_cntrl].laps = 0;
					race_info.driver_data[_cntrl].laps_behind = 0;
					race_info.driver_data[_cntrl].red_flag_flag = 0;
					race_info.driver_data[_cntrl].joined_race = 0;
				}
			}
		} else {
			// catch unprocessed cu data for uart

			// unfortunately we have to go through various if statements.
			if ((6 > controller) && !(0 == command)){
				// things we never ever want to proceed
				if (4 == command){
					if (8 > value){
						// fuel level of car - 7 is full
						race_info.driver_data[controller].fuel = value;
					} else if (16 > value){
						// not really neccessary. Called at jump start
						race_info.driver_data[controller].fuel = value - 8;
					}
				}else if (6 == command){
					if (value < 9){
						// postition
						//value--; // position -1 to fit in 4 bits
						if (255 == race_info.driver_data[controller].position){
							lap_finished_identifier = controller;
							race_info.driver_data[controller].laps = 0;
						}

						if ( race_info.driver_data[controller].red_flag_flag ){
							// red flag resets positions. First
							race_info.driver_data[controller].red_flag_flag = 0;
							race_info.driver_data[controller].laps++;
						}
					} else if (9 == value) {
						// chaos / reset
						race_info.chaos = 1;
						for (uint8_t _cntrl = 0; _cntrl < 6; _cntrl++){
							race_info.driver_data[_cntrl].red_flag_flag = 1;
						}
					}
				} else if (7 == command){
					// driver finishes race
					race_info.driver_data[controller].finished = 1;
				} else if (8 == command){
					// driver finishes lap / race with new fastest lap
					lap_finished_identifier = controller;
					//race_info.wildcard = controller;
					race_info.driver_data[controller].laps++;
				} else if (9 == command){
					lap_finished_identifier = controller;
					// driver finishes lap / race without new fastest lap
					race_info.driver_data[controller].laps++;
				} else if ((10 == command) && (8 > value)){
					// fuel level of car - 7 is full
					race_info.driver_data[controller].fuel = value;
				}
			}/* else if ((17 == command) && (7 == controller)){
				// first nibble lap count leader
				race_info.lap_count = value << 4;
				race_info.lap_count_nibble_completed = 0;
			} else if ((18 == command) && (7 == controller)){
				// first nibble lap count leader
				race_info.lap_count |= value;
				race_info.lap_count_nibble_completed = 1;
			}*/
		} // catch unprocessed cu data for uart end
	} // control unit message end#

	if (4 == tenths_calculator){
		tenths_calculator = 0;
		race_info.tenths++;
		race_info.clock.tenths++;
	}

	if (9 < race_info.clock.tenths){
		race_info.clock.tenths = 0;
		race_info.clock.seconds_first++;
		if (9 < race_info.clock.seconds_first){
			race_info.clock.seconds_first = 0;
			race_info.clock.seconds_second++;
			if (5 < race_info.clock.seconds_second){
				race_info.clock.seconds_second = 0;
				race_info.clock.minutes_first++;
				if (9 < race_info.clock.minutes_first){
					race_info.clock.minutes_first = 0;
					race_info.clock.minutes_second++;
				}
			}
		}
	}

	if (status_message_identifier){
		/*
		 * Chaos resets lap counts and other track information.
		 * That makes some hacks necessary.
		 */
		for (uint8_t driver_index = 0; driver_index < 6; driver_index++){
			if (race_info.driver_data[driver_index].laps > race_info.lap_count){
				race_info.lap_count = race_info.driver_data[driver_index].laps;
			}
		}
	}

	if (6 > lap_finished_identifier){
		// position must be calculated out of lap increment because of chaos handling
		if (!race_info.driver_data[lap_finished_identifier].joined_race){
			race_info.driver_data[lap_finished_identifier].joined_race = 1;
			race_info.driver_data[lap_finished_identifier].position = race_info.in_race_counter++;
		} else {
			uint8_t position = 0;
			for (uint8_t driver_index = 0; driver_index < 6; driver_index++){
				if ((driver_index != lap_finished_identifier) &&
						(race_info.driver_data[driver_index].laps >= race_info.driver_data[lap_finished_identifier].laps)){
					position++;
				}
			}

			if (position < race_info.driver_data[lap_finished_identifier].position){
				// position change
				for (uint8_t driver_index = 0; driver_index < 6; driver_index++){
					if ((race_info.driver_data[driver_index].position >= position) &&
							(race_info.driver_data[driver_index].position < race_info.driver_data[lap_finished_identifier].position)){
						// between old and new position
						race_info.driver_data[driver_index].position++;
					}
				}
			}
			race_info.driver_data[lap_finished_identifier].position = position;
		}


	}


	/*
	 * OVERLAP
	 *
	 * New algorithm overlap count because the old one lecks at pit stop of race leader.
	 * In this case no calculation happen.
	 */
	if (lap_finished_identifier < 6){
		uint8_t total_laps_temp = 0;
		uint8_t first = 255;
		for (uint8_t driver_first_idx = 0; driver_first_idx < 6; driver_first_idx++){
			if (race_info.driver_data[driver_first_idx].laps > total_laps_temp){
				total_laps_temp = race_info.driver_data[driver_first_idx].laps;
				first = driver_first_idx;
			}
		}

		uint8_t lap_shift = 0;
		if (lap_finished_identifier == first)
			lap_shift = 1;

		for (uint8_t driver_lap_behind_idx = 0; driver_lap_behind_idx < 6;driver_lap_behind_idx++){
			uint16_t lap_behind_count = race_info.driver_data[first].laps - race_info.driver_data[driver_lap_behind_idx].laps;
			if (lap_behind_count > lap_shift)
				race_info.driver_data[driver_lap_behind_idx].laps_behind = lap_behind_count-lap_shift;
			else
				race_info.driver_data[driver_lap_behind_idx].laps_behind = 0;
		}

	}

	return status_message_identifier;
}


uint8_t update_race_info_status(void){
	uint8_t carrera_system_status_flag = 0;

	uint8_t sector_actu = manchester_buffer_sector_of(manchester_buf_cursor);
	uint8_t sector_prev = manchester_buffer_sector_of(manchester_buf_cursor);
	// if next sector free then proceed
	while ((manchester_buffer_sector_free(manchester_buffer_sector_of(manchester_buffer_circ_next(manchester_buf_cursor))))){
		manchester_buf_cursor = manchester_buffer_circ_next(manchester_buf_cursor);
		sector_actu = manchester_buffer_sector_of(manchester_buf_cursor);
		if (sector_actu != sector_prev)
			manchester_buffer_clear_sector(sector_prev);
		sector_prev = sector_actu;

		if (track_data_decode(manchester_buffer_get_capture(manchester_buf_cursor))){
			if (update_race_info(track_data_message())){
				update_race_state_machine_regular();
				carrera_system_status_flag = 1;
			}
		}
	}

	return carrera_system_status_flag;
}
