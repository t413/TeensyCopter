//
//  eeprom.h
//  teensy_copter
//
//  Created by Tim O'Brien on 4/17/11.
//  Copyright 2011 t413.com. All rights reserved.
//


#ifndef EEPROM_H_
#define EEPROM_H_
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif
    

#include <inttypes.h>


void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);


void EEPROM_write_16(unsigned int addr, uint16_t data);
uint16_t EEPROM_read_16(unsigned int addr);


#ifdef __cplusplus
}
#endif
#endif