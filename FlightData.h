#ifndef FLIGHTDATA_H
#define FLIGHTDATA_H

#include "pid.h"
#include <inttypes.h>


/*----flying_mode----*/
#define X_MODE 0
#define PLUS_MODE 1

/*----armed----*/
#define SAFTY_ARMING 1
#define ARMED_AND_DANGEROUS 3
#define LOST_COMMUNICATION 5

/*----control----*/
#define MIN_CONTROL 1000
#define MID_CONTROL 1500
#define MAX_CONTROL 2000
#define MIN_SAFETY MIN_CONTROL + 100
#define MAX_SAFETY MAX_CONTROL - 100
#define MIN_THROTTLE MIN_CONTROL + 100


typedef struct {
    
    unsigned char armed;
    unsigned char telem_mode;
    int tx_throttle, tx_yaw, tx_pitch, tx_roll;
    unsigned int command_used_number;
    unsigned char please_update_sensors;
    unsigned int user_feedback;
    
    struct {
        PID * pid_roll;
        PID * pid_pitch;
        PID * pid_yaw;
        uint8_t flying_mode; //X_MODE or PLUS_MODE
        uint8_t led_mode;
        uint16_t pitch_roll_tx_scale;
        uint16_t yaw_tx_scale;
    } config;
    
} FlightData;


#endif /* COMMONDEFS_H_ */
