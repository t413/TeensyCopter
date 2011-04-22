#ifndef FLIGHTDATA_H
#define FLIGHTDATA_H

#include "pid.h"
#include "wii_sensors.h"


/*----flying_mode----*/
#define X_MODE 0
#define PLUS_MODE 1
#define TRICOPTER_MODE 2

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


class FlightData{
public:
    unsigned char armed;
    SENSOR_DATA zero_data;
    
    int16_t tx_throttle, tx_yaw, tx_pitch, tx_roll;
    unsigned int command_used_number;
    unsigned char please_update_sensors;
    unsigned int user_feedback;
    
    struct {
        PID * pid_roll;
        PID * pid_pitch;
        PID * pid_yaw;
        uint8_t flying_mode; //X_MODE or PLUS_MODE
        uint8_t led_mode;
        int16_t pitch_roll_tx_scale;
        int16_t yaw_tx_scale;
    } config;
    
    /*----functions----*/
    FlightData(void);
    void load_from_eeprom(void);
    void store_to_eeprom(void);
    void store_eeprom_zero_data(void);
    
private:
};



#endif