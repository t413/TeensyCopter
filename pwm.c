//
//  pwm.c
//  teensy_copter
//
//  Created by Tim O'Brien on 4/13/11.
//  Copyright 2011 t413.com. All rights reserved.
//

#include <avr/io.h>
#include <inttypes.h>
#include "pwm.h"


//TODO: make variable argument init that inits spesific channels.

void pwm_init(void) { 
    
    /* ---- set up timers 1 and 3 to enable channels A and B ---- */
    //sets up OC1A then OC1B, then enables them.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); 
    // WGM11, WGM12 and WGM13 set PWM output mode 14 so ICR1 is top 
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11); // <--- Prescaling by 8

    //sets up OC3A then OC3B, then enables them.
    TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31); 
    TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31);
    
    // Using ICR1 as TOP For 50Hz Output at max resolution set at 40,000 ... 16,000,000 / 8 * 40001 = ~50hz 

    ICR1 = 40000; //counter counts to this value before restarting. Called TOP.
    ICR3 = 40000; //counter counts to this value before restarting. Called TOP.
    //(F_CPU / (CLKPR+1))/(50*8);  // should equal 40000 when at 16Mhz.
    
    // Setup PWM prior to setting the data direction 
    // Setup the pwm output pin an output - because only doing one just set the bit value 
    // Use OR in case other bits have been set in DDRB 


    /*-- set pins to outputs --*/
    DDRB |= (1 << 5);
    DDRB |= (1 << 6);
    DDRC |= (1 << 5);
    DDRC |= (1 << 6);
    // Set OCRA1 to something - Servo is based on 1 - 2ms pulse if 40000 = 20ms pulse whats between 1 and 2 (2200 TO 3800) 

    OCR1A = 2200; 
    OCR1B = 2200; 
    OCR3A = 2200; 
    OCR3B = 2200; 
}


//input between 1000 and 2000
void write_servo(unsigned char which, short in_val){
    //(2200 TO 3800) 
    in_val = in_val + 1200;
    if (in_val > 3800) in_val = 3800;
    if (in_val < 2200) in_val = 2200;
    
    switch (which){
        case 0 : OCR1A = in_val; break;
        case 1 : OCR1B = in_val; break; 
        case 2 : OCR3A = in_val; break; 
        case 3 : OCR3B = in_val; break; 
    }
}

void write_motors_zero(void){
    OCR1A = 2200;
    OCR1B = 2200;
    OCR3A = 2200;
    OCR3B = 2200;
}

