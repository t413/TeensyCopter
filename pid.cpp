/*
 * pid.c
 *
 *  Created on: Nov 18, 2010
 *      Author: timo
 */


#include "pid.h"

#define TOTAL_SCALING_FACTOR 1500.00 //(1<<10) = 1,024.
#define INTEGRAL_SCALING_FACTOR 8



PID::PID(int16_t new_p, int16_t new_i, int16_t new_d) {
    p = new_p;
    i = new_i;
    d = new_d;
}

PID::PID(void) {
    p = 0;
    i = 0;
    d = 0;
}

int16_t PID::update(int16_t incoming_val, int16_t goal_val){
    
	int16_t delta = goal_val - incoming_val;
    
	//integral calculation.
	error += delta;
	dual_clip( &error, i_limit * INTEGRAL_SCALING_FACTOR * TOTAL_SCALING_FACTOR/i );  //limits how high or low the error can get
    
	//derivative calculation.
	int16_t this_d = (incoming_val - prev_val);
	prev_val = incoming_val;
    
	return (float)( (delta * (float)p) + (error * i)/INTEGRAL_SCALING_FACTOR + (this_d * d)) / (float)TOTAL_SCALING_FACTOR;
}

void PID::zero(void){
    error = 0;
}


void PID::dual_clip(int64_t * val, uint16_t cap){
	if (*val > cap) *val = cap;
	else if (*val < -cap) *val = -cap;
}

