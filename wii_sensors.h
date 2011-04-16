/*
 * wii_sensors.h
 *
 *  Created on: Dec 5, 2010
 *      Author: timo
 */

#ifndef WII_SENSORS_H_
#define WII_SENSORS_H_
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif

    

#define MAX_WII_SENSOR_POLLING_RATE (5) //5ms

typedef struct {
	int16_t roll, pitch, yaw;
	int16_t x, y, z;
}SENSOR_DATA;

uint8_t init_wii_sensors(void);
uint8_t zero_wii_sensors(SENSOR_DATA *zero_data);
uint8_t update_wii_data(SENSOR_DATA *vals, SENSOR_DATA *zer0);

    
    
#ifdef __cplusplus
}
#endif
#endif /* WII_SENSORS_H_ */
