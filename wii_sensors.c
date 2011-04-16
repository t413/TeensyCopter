/*
 * wii_sensors.c
 *
 *  Created on: Dec 5, 2010
 *      Author: timo
 */

#include <util/delay.h>
#include <inttypes.h>
#include "uart.h"
#include "twi.h"
#include "wii_sensors.h"

#include "usb_debug_only.h"
#include "print.h"


uint8_t init_wii_sensors(void){

	// cold-initialize wii motion+ with nunchuck attached
    uint8_t i2c_data[] = {0xFE, 0x04, 0xFE, 0x05};

	//if ( i2c_send_byte(0xA6, 0xF0, 0x55) && i2c_send_byte(0xA6, 0xFE, 0x05) ) //(nunchuck & mw+) mode
	if ( twi_writeTo(0x53, i2c_data, 2, 1) && twi_writeTo(0x53, &(i2c_data[2]), 2, 1) ) //(mw+) mode
	{ return 1; }
	//already initialized motion plus is ready to go.
	else if (twi_writeTo(0x52, i2c_data, 0, 1))
	{ return 2; }

	else return 0; //error
}


uint8_t zero_wii_sensors(SENSOR_DATA *zero_data){
	uint8_t successful_reads = 0;
	SENSOR_DATA vals = {0};
	*zero_data = vals; //zero the data.
	SENSOR_DATA dummy_zero = { 0 };

	for (int i=0;i<15;i++)
	{
		if (update_wii_data(&vals, &dummy_zero) == 1)
		{
			zero_data->yaw   += vals.yaw;
			zero_data->pitch += vals.pitch;
			zero_data->roll  += vals.roll;
			successful_reads++;
		}
        _delay_ms(10);
		//vTaskDelay(10);
	}
	zero_data->yaw   = zero_data->yaw   / successful_reads;
	zero_data->pitch = zero_data->pitch / successful_reads;
	zero_data->roll  = zero_data->roll  / successful_reads;
	if (successful_reads<1){
		zero_data->yaw = 0;
		zero_data->pitch = 0;
		zero_data->roll = 0;
	}
	return successful_reads;
}

uint8_t update_wii_data(SENSOR_DATA *vals, SENSOR_DATA *zer0){
	uint8_t data[6] = "";
    twi_writeTo(0x52, data, 1, 1); //send zero to ask for data.
	if (twi_readFrom(0x52,data,6))
	{
		if ( data[5]&0x02 ) //wm+ data
		{
			vals->yaw = (((data[3]>>2)<<8)+data[0]);
			vals->pitch = (((data[4]>>2)<<8)+data[1]);
			vals->roll = (((data[5]>>2)<<8)+data[2]);
			//use the slow/fast mode data
			vals->yaw   = ((signed)((data[3]&0x02)>>1  ? vals->yaw/5   : vals->yaw)) - (signed)zer0->yaw;
			vals->pitch = ((signed)((data[4]&0x02)>>1  ? vals->pitch/5 : vals->pitch)) - (signed)zer0->pitch;
			vals->roll  = ((signed)((data[3]&0x01)     ? vals->roll/5  : vals->roll)) - (signed)zer0->roll;
			return 1; //got wm+ data successfully
		}
		else //this is nunchuck data
		{
			vals->x = ( (data[2]<<2)        + ((data[5]>>3)&0x2) ) - zer0->x;
			vals->y = ( (data[3]<<2)        + ((data[5]>>4)&0x2) ) - zer0->y;
			vals->z = ( ((data[4]&0xFE)<<2) + ((data[5]>>5)&0x6) ) - zer0->z;
			return 2; //got nunchuck data successfully
		}
	}
	vals->yaw = 0;
	vals->pitch = 0;
	vals->roll = 0;
	vals->x = 0;
	vals->y = 0;
	vals->z = 0;
	return 0; //data not received successfully.
}
