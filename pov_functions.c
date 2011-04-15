/*
 *  led-matrix.c
 *  teensy
 *
 *  Created by Tim O'Brien on 8/4/10.
 *  Copyright 2010 t413.com. All rights reserved.
 *
 */

#include "pov_functions.h"

#define POV_PIXEL_WIDTH 60
uint8_t screen[POV_PIXEL_WIDTH] = "";

void updatePOV(uint8_t line) {
	//update the row of leds all at once. Huzzah for being able to use the entire port!
	PORTA = screen[line]; 
}


void w_str(const char *s, uint8_t x) {
	char c;
	uint8_t i = 0;
	while (1) {
		c = pgm_read_byte(s++);
		if (!c) break;
		writeASCII(c, x+6*i);
		++i;
	}
}

//write an ascii character to the pov screen buffer
// input: -char to write
//        -x position on the display.
void writeASCII(char ch, uint8_t x) {
 	print("writing:");
	pchar(ch);
	print("\n");
	
	//if (ch < 32 || ch > 126) ch = 32;
	//else (ch - 32);
	for (uint8_t i = 0; i<5; i++) {
		if ((i+x) < POV_PIXEL_WIDTH) // don't over-write our memory
			screen[i+x] = pgm_read_byte( &(f7x5[ch-32][i]) );
		//screen[i+x] = font[ch-32][i];
	}
}

void drawMan(uint8_t x) {
	const uint8_t man[] = {0x88,0x64,0x9F,0x64};
	for (uint8_t i = 0; i<4; i++) {
		if ((i+x) < POV_PIXEL_WIDTH) // don't over-write our memory
			screen[i+x] = man[i];
	}
}

void drawCar(uint8_t x) {
	const uint8_t car[] = {0x20,0x38,0x79,0xFC,0x7E,0x3E,0x3C,0x78,0xF8,0x70,0x20};
	for (uint8_t i = 0; i<4; i++) {
		if ((i+x) < POV_PIXEL_WIDTH) // don't over-write our memory
			screen[i+x] = car[i];
	}
}

void printDisplay() {
 	print("here's the display:\n\n");
	for (uint8_t i = 0; i<8; i++) { //first loop goes down the rows
		for (uint8_t j = 0; j<POV_PIXEL_WIDTH; j++) { //inner loop goes across columns
			pchar( ((screen[j] >> i)&1) ? '#':'.' );
		}
		print("\n");
	}
 	print("\ndone\n\n");
}

void clearDisplay() {
	print("clear display\n");
	for (int i = 0; i < POV_PIXEL_WIDTH; i++)
		screen[i]=0;
}

/*
void allON() {
	for (int i = 0; i < 8; i++)
		screen[i]=0xFF;
}

void on(uint8_t row, uint8_t column) {
    screen[column] |= (1<<(row)); 
}

void off(uint8_t row, uint8_t column) {
    screen[column] &= ~(1<<(row)); 
}
*/