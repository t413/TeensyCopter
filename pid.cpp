/*
 * pid.c
 *
 *  Created on: Nov 18, 2010
 *      Author: timo
 */


#include "pid.h"

#define TOTAL_SCALING_FACTOR (1<<10) //(1<<10) = 1,024.
#define INTEGRAL_SCALING_FACTOR 8



PID::PID(short new_p, short new_i, short new_d) {
    p = new_p;
    i = new_i;
    d = new_d;
}

PID::PID(void) {
    p = 0;
    i = 0;
    d = 0;
}

short PID::update(short incoming_val, short goal_val){
    
	short delta = goal_val - incoming_val;
    
	//integral calculation.
	error += delta;
	//dual_clip( &error, i_limit * INTEGRAL_SCALING_FACTOR * TOTAL_SCALING_FACTOR/i );  //limits how high or low the error can get
    
	//derivative calculation.
	short this_d = (incoming_val - prev_val);
	prev_val = incoming_val;
    
	return ( (delta * p) + (error * i)/INTEGRAL_SCALING_FACTOR + (this_d * d)) / TOTAL_SCALING_FACTOR;
}

void PID::zero(void){
    error = 0;
}


/*
void PID:dual_clip(long * val, unsigned short cap){
	if (*val > cap) *val = cap;
	else if (*val < -cap) *val = -cap;
}
*/