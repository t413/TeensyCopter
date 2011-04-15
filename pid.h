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
    PID(short p, short i, short d);
    short update(short incoming_val, short goal_val);
    void zero(); //zeroes integral error.
    
    short p, i, d;
    
    private:
        //void dual_clip(long * val, unsigned short cap);
        unsigned short i_limit;
        signed long error;
        signed short prev_val;
};



#endif /* PID_H_ */
