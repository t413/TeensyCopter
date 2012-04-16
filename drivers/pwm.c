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

static uint8_t fastmode;
uint16_t limit (uint16_t in_val, uint16_t min, uint16_t max);

//TODO: make variable argument init that inits spesific channels.

void pwm_init(void) { 
    
    /* ------------------------------------------------------- */
    /* ---- set up timer 1 to enable channels A, B, and C ---- */
    
    //Setup OC1A to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR1A |= (1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0);
    
    //Setup OC1B to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR1A |= (1 << COM1B1);
    TCCR1A &= ~(1 << COM1B0);
    
    //Setup OC1C to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR1A |= (1 << COM1C1);
    TCCR1A &= ~(1 << COM1C0);
    
    // !(WGM10) WGM11, WGM12 and WGM13 set for FAST PWM output (mode 14 in datasheet) so ICR1 is top 
    TCCR1A |= (1 << WGM11); 
    TCCR1A &= ~(1 << WGM10); 
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    
    //Timing:
    // Using ICR1 as TOP For 50Hz Output at max resolution set at 40,000 ... 16,000,000 / 8 * 40001 = ~50hz 
    //(F_CPU / (CLKPR+1))/(50*8);  // should equal 40000 when at 16Mhz.
    TCCR1B |= (1 << CS11); // <--- Prescaling by 8 (page 138)
    ICR1 = 40000; //counter counts to this value before restarting. Called TOP.
    fastmode = 0;
    
    
    /* -------------------------------------------- */
    /* ---- set up timer 3 to enable channel A ---- */
    
    //Setup OC3A to Clear on compare match, set OCnA/OCnB/OCnC at TOP
    TCCR3A |= (1 << COM3A1); 
    TCCR3A &= ~(1 << COM3A0); 
    
    TCCR3A |= (1 << WGM31); 
    TCCR3A &= ~(1 << WGM30); 
    TCCR3B |= (1 << WGM32) | (1 << WGM33);
    
    //Timing:
    TCCR3B |= (1 << CS11); // <--- Prescaling by 8 (page 138)
    ICR3 = 40000; //counter counts to this value before restarting. Called TOP.
    
    
    
    /* ----------------------- */
    /*-- set pins to outputs --*/
    DDRB |= (1 << 5); //0  ->  right bottom port   OC1A
    DDRB |= (1 << 6); //1  ->  right top port      OC1B
    DDRB |= (1 << 7); //2  ->  left bottom port    OC1C
    DDRC |= (1 << 6); //3  ->  left top port       OC3A
    // Set OCRA1 to something - Servo is based on 1 - 2ms pulse if 40000 = 20ms pulse whats between 1 and 2 (2000 TO 4000) 
    
    
    /* -------------------------------- */
    /* ---- set up timer 4 for PWM ---- */
    TCCR4B = (1<<CS42); //prescaler for for OC4A and OC4D
    OCR4C = 0xFF; //OCR4C acts as TOP value for OC4A and OC4D
    
    //set up OCRA
    DDRC |= (1<<7); //port C7 as output for OC4A
    TCCR4A = (1<<COM4A1) | (1<<PWM4A);
    
    //set up OCRD
    DDRD |= (1<<7); //port D7 as output for OCRD
    TCCR4C = (1<<COM4D1) | (1<<PWM4D);
    
    // now set OCR4D and OCR4A
    OCR4D = OCR4A = 0;
}


/**
 * Enable fast PWM ouput for Timer 1 (servos 1, 2, and 3)
 * Great for re-flashed high speed ESCs
 */
void enableFastPWMforServos0_1_2(void) {
    TCCR1B &= ~((1 << CS11) | (1 << CS12));
    TCCR1B |= (1 << CS10);
    fastmode |= 1;
}
void enableFastPWMforServos3(void) {
    TCCR3B &= ~((1 << CS11) | (1 << CS12));
    TCCR3B |= (1 << CS10);
    fastmode |= 2;
}

#define map(x,in_min,in_max,out_min,out_max) (((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))


//input between 1000 and 2000
void write_servo(uint8_t which, uint16_t in_val){
    //(2000 TO 4000)
    in_val = ((in_val-1000)*2) + SERVO_MIN; //should be scaled between 2000 and 4000 now.
    in_val = limit(in_val, SERVO_MIN, SERVO_MAX);
    
    if ((fastmode & 1) && (which != 3)){
        in_val = (in_val-SERVO_MIN)*8 + 16000;
    } else if ((fastmode & 2) && (which == 3)){
        in_val = (in_val-SERVO_MIN)*8 + 16000;
    }
    
    switch (which){
        case 0 : OCR1A = in_val; break; 
        case 1 : OCR1B = in_val; break; 
        case 2 : OCR1C = in_val; break; 
        case 3 : OCR3A = in_val; break; 
    }
}

void write_motors(uint16_t m0, uint16_t m1, uint16_t m2, uint16_t m3) {
    if (fastmode & 1) {
        OCR1A = (limit(m0, SERVO_MIN, SERVO_MAX)-SERVO_MIN)*8 + 16000;;
        OCR1B = (limit(m1, SERVO_MIN, SERVO_MAX)-SERVO_MIN)*8 + 16000;;
        OCR1C = (limit(m2, SERVO_MIN, SERVO_MAX)-SERVO_MIN)*8 + 16000;;
    } else {
        OCR1A = limit(m0, SERVO_MIN, SERVO_MAX);
        OCR1B = limit(m1, SERVO_MIN, SERVO_MAX);
        OCR1C = limit(m2, SERVO_MIN, SERVO_MAX);
    }
    
    if (fastmode & 2) {
        OCR3A = (limit(m3, SERVO_MIN, SERVO_MAX)-SERVO_MIN)*8 + 16000;;
    } else {
        OCR3A = limit(m3, SERVO_MIN, SERVO_MAX);
    }
}

uint16_t limit (uint16_t in_val, uint16_t min, uint16_t max){
    if (in_val > max) return max;
    else if (in_val < min) return min;
    else return in_val;
}




