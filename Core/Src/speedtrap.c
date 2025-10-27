/*
 * speedtrap.c
 *
 *  Created on: Oct 8, 2025
 *      Author: pkr
 */


# include "speedtrap.h"

static vehicle 		sptr_vehicles [SPTR_DRIVER];
static segment_gate sptr_gates [SPTR_GATES];		// 0 and 1 for left lane, 2 and 3 for right lane
static uint16_t 	sptr_timer_low_res;
static uint16_t		sptr_trigger_sensitivity;
static	uint8_t		sptr_trigger_local = 0;
static uint16_t		sptr_flash_active_flag = 0;
static uint16_t		sptr_flash_active_time = 0;

void sptr_init(void){
	for (uint8_t driver = 0; driver < SPTR_DRIVER; driver++){
		sptr_vehicles[driver].id = driver;
		sptr_vehicles[driver].in_section = 0;
		sptr_vehicles[driver].time_entry_hires = 0;
		sptr_vehicles[driver].time_entry_lores = 0;
	}

	for (uint8_t gate = 0; gate < SPTR_GATES; gate++){
		for (uint8_t bases = 0; bases < SPTR_BASES; bases++)
			sptr_gates[gate].captures[bases] = 0;
		sptr_gates[gate].pointer = 0;
		sptr_gates[gate].reset_timer = 0;
		sptr_gates[gate].last_capture = 0;
	}

	sptr_timer_low_res = 0;
	sptr_trigger_sensitivity = 0;
}

void sptr_set_trigger_sensitivity(uint16_t value){
	sptr_trigger_sensitivity = value;
}


uint8_t sptr_triggered(void){
	if (sptr_trigger_local){
		sptr_trigger_local = 0;
		return 1;
	}
	return 0;
}


uint8_t sptr_flash_active(uint16_t hundredth){
	if (sptr_triggered() && !sptr_flash_active_flag){
		sptr_flash_active_time = hundredth;
		sptr_flash_active_flag = 1;
	} else{
		if (sptr_flash_active_flag){
			if ((hundredth - sptr_flash_active_time) > 9){
				sptr_flash_active_flag = 0;
				return 0;
			}
		}
	}
	return 1;

}

void sptr_isr_entry_identification_lane_1(uint16_t capture){
	segment_gate *gate = &sptr_gates[1];

	if (gate->pointer < SPTR_BASES){
		/*
		 * if in actual interval and
		 *
		 * overflow happens two times or more
		 * or
		 * overflow is 1 and cc is gt 1250 (10ms)
		 */
		if ((sptr_timer_low_res > (gate->reset_timer + 1)) ||
				((sptr_timer_low_res == (gate->reset_timer + 1)) &&
						capture >= gate->last_capture) ||
						((capture- gate->last_capture)>1250)  ){
			gate->pointer = SPTR_BASES + 1; // redirect to reset because of mismeasurement
		} else {
			gate->captures[gate->pointer] = capture;
			gate->last_capture = capture;
			gate->pointer++;
		}
	}

	if (gate->pointer > SPTR_BASES){
		// check if first edge is at least 10ms gone
		// if first edge more the 10ms gone, reset gate


		if ((sptr_timer_low_res > (gate->reset_timer + 1)) ||
				((sptr_timer_low_res == (gate->reset_timer + 1)) &&
						capture >= gate->last_capture) ||
						((capture- gate->last_capture)>1250)  ){

			gate->captures[0] = capture;
			gate->last_capture = capture;
			gate->pointer = 1;
			gate->reset_timer = sptr_timer_low_res;

		}
	}



	if (gate->pointer == SPTR_BASES){
		// increment pointer and go in on hold state
		gate->pointer++;

		// calculate id
		uint16_t diff1 = gate->captures[1] - gate->captures[0];
		uint16_t diff2 = gate->captures[2] - gate->captures[1];

		if ((diff1 - diff2) > 3){ // if discrepancy between diff1 and diff2 greater then 3*8=24 microseconds -> mismeasurement
			return;
		} else if(diff1 < 5){ // period smaller then 64us - 32us -> mismeasurement
			return;
		} else if (diff1 > 52){ // period greather then 384us + 32us-> mismeasurement
			return;
		}
		// set incoming car

		uint16_t id = ((diff1 + 4) >> 3) - 1;
		if (id < 6){
			sptr_vehicles[id].in_section = 1;
			sptr_vehicles[id].time_entry_hires = capture; // todo: time in ms
			sptr_vehicles[id].time_entry_lores = sptr_timer_low_res;
		}
	}
}

void sptr_isr_exit_identification_lane_1(uint16_t capture){
	segment_gate *gate = &sptr_gates[0];

	if (gate->pointer < SPTR_BASES){
		if ((sptr_timer_low_res > (gate->reset_timer + 1)) ||
				((sptr_timer_low_res == (gate->reset_timer + 1)) &&
						capture >= gate->last_capture) ||
						((capture- gate->last_capture)>1250)  ){
			gate->pointer = SPTR_BASES + 1; // redirect to reset because of mismeasurement
		} else {
			// regular measurement
			gate->captures[gate->pointer] = capture; // todo: vulnerable use of pointer - potential failure
			gate->last_capture = capture;
			gate->pointer++;
		}
	}

	if (gate->pointer > SPTR_BASES){
		// check if first edge is at least 10ms gone
		// if first edge more the 10ms gone, reset gate


		/*
		 * 10ms gone
		 * if sptr_timer_low_res > first_edge + 1 or
		 * sptr_timer_low_res == first_edge + 1 and capture-first_edge > 1250
		 */

		if ((sptr_timer_low_res > (gate->reset_timer + 1)) ||
				((sptr_timer_low_res == (gate->reset_timer + 1)) &&
						capture >= gate->last_capture) ||
						((capture- gate->last_capture)>1250)  ){
			// first edge
			gate->captures[0] = capture;
			gate->last_capture = capture;
			gate->pointer = 1;
			gate->reset_timer = sptr_timer_low_res;

		}
	}



	if (gate->pointer == SPTR_BASES){
		// increment pointer and go in on hold state
		gate->pointer++;

		// calculate id
		uint16_t diff1 = gate->captures[1] - gate->captures[0];
		uint16_t diff2 = gate->captures[2] - gate->captures[1];

		if ((diff1 - diff2) > 3){ // if discrepancy between diff1 and diff2 greater then 3*8=24 microseconds -> mismeasurement
			return;
		} else if(diff1 < 5){ // period smaller then 64us - 32us -> mismeasurement
			return;
		} else if (diff1 > 52){ // period greater then 384us + 32us-> mismeasurement
			return;
		}

		// calculate outgoing car

		uint16_t id = ((diff1 + 4) >> 3) - 1;
		if (id < 6){
			if ((sptr_timer_low_res - sptr_vehicles[id].time_entry_lores) < 2){
				uint16_t hires_diff = capture - sptr_vehicles[id].time_entry_hires;
				if (hires_diff < (6250 + sptr_trigger_sensitivity)){
					sptr_trigger_local = 1;
				}
			}
		}
	}
}

void sptr_isr_entry_identification_lane_2(uint16_t capture){
	return;
}
void sptr_isr_exit_identification_lane_2(uint16_t capture){
	return;
}

void sptr_isr_overflow_timer_low_res(void){
	// overflow of the cc timers to get the right speed
	// overflow occurs every 524.280ms
	sptr_timer_low_res++;
}
