#ifndef FLIGHTDATA_H
#define FLIGHTDATA_H

#include <inttypes.h>
#include "pid.h"
#include "wii_sensors.h"


/*----flying_mode----*/
typedef enum FlyingMode {
    X_MODE = 0,
    PLUS_MODE,
    TRICOPTER_MODE,
    TWINCOPTER_MODE,
} FlyingMode;

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

//used for tx_values array.
typedef enum TX_channels {
    tx_yaw = 0,
    tx_pitch,
    tx_throttle,
    tx_roll
} TX_channels;

class FlightData{
public:
    uint32_t t_current; //current millis() since boot (or overflow..)
    unsigned char armed;
    SENSOR_DATA zero_data;
    
    int16_t tx_values[4]; //holds processed transmitter values.
    uint32_t last_t_xbee_packet, last_t_24ghz_packet; //times of last successful packet recieve
    uint8_t  last_tx_metadata;
    struct {
        uint32_t lastReset;
        uint16_t packetCount;
    } RxQuality;
    
    unsigned int command_used_number;
    unsigned char please_update_sensors;
    uint16_t user_feedback_m;
    uint16_t user_feedback_i;
    
    PID pitch, roll, yaw, alt;
    struct {
        FlyingMode flying_mode; //X_MODE or PLUS_MODE
        uint8_t led_mode;
        int16_t pitch_roll_tx_scale;
        int16_t yaw_tx_scale;
    } config;
    
    
    /*----functions----*/
    FlightData(void);
    void process_analogs( int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle);
    void load_from_eeprom(void);
    void store_to_eeprom(void);
    void store_eeprom_zero_data(void);
    
private:
};



#endif