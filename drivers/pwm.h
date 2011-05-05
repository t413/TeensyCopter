//
//  pwm.h
//  teensy_copter
//
//  Created by Tim O'Brien on 4/13/11.
//  Copyright 2011 t413.com. All rights reserved.
//

#ifndef PWM_H_
#define PWM_H_
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif

//used to write to the output
#define SERVO_MIN 2000
#define SERVO_MAX 4000
#define SERVO_MID 3000

    
void pwm_init(void);
void write_servo(uint8_t which, uint16_t in_val);
void write_motors(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3);
    
#define write_motors_zero() write_motors(SERVO_MIN,SERVO_MIN,SERVO_MIN,SERVO_MIN)


    
    
#ifdef __cplusplus
}
#endif
#endif