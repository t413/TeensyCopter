/*
 * pid.h
 *
 *  Created on: Nov 18, 2010
 *      Author: timo
 */

#ifndef PID_H_
#define PID_H_

#include <inttypes.h>


class PID{
    public:
    PID(void); //default constructor
    PID(int16_t p, int16_t i, int16_t d);
    int16_t update(int16_t incoming_val, int16_t goal_val);
    void zero(); //zeroes integral error.
    
    int16_t p, i, d;
    
    private:
        void dual_clip(int64_t * val, uint16_t cap);
        uint16_t i_limit;
        int64_t error;
        int16_t prev_val;
};



#endif /* PID_H_ */
