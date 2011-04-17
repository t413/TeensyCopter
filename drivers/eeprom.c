//
//  eeprom.c
//  teensy_copter
//
//  Created by Tim O'Brien on 4/17/11.
//  Copyright 2011 t413.com. All rights reserved.
//

#include "eeprom.h"
#include <avr/io.h>
#include <inttypes.h>


/* These functions assume that interrupts are controlled so that no interrupts will occur during their execution. 
 * again NO INTERRUPTS can occur. use sei() and cli();
 */

void EEPROM_write(unsigned int uiAddress, unsigned char ucData) {
    /* Wait for completion of previous write */ 
    while(EECR & (1<<EEPE)); 
    /* Set up address and Data Registers */ 
    EEAR = uiAddress; 
    EEDR = ucData; 
    /* Write logical one to EEMPE */ 
    EECR |= (1<<EEMPE); 
    /* Start eeprom write by setting EEPE */ 
    EECR |= (1<<EEPE);
}


unsigned char EEPROM_read(unsigned int uiAddress) {
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE));
    /* Set up address register */ 
    EEAR = uiAddress; 
    /* Start eeprom read by writing EERE */ 
    EECR |= (1<<EERE); 
    /* Return data from Data Register */ 
    return EEDR;
}


void EEPROM_write_16(unsigned int addr, uint16_t data){
    EEPROM_write( addr,     ((data >> 0) & 0xFF) );
    EEPROM_write( addr + 1, ((data >> 8) & 0xFF) );
}

uint16_t EEPROM_read_16(unsigned int addr){
    return ((EEPROM_read(addr) << 0) & 0xFF) + ((EEPROM_read(addr+1) << 8) & 0xFF00);
}