

#include <inttypes.h>
#include "pid.h"
#include "FlightData.h"
#include "drivers/eeprom.h"

/* ---- constructor ---- */
FlightData::FlightData(){
    armed = 0;
    
    tx_throttle = MIN_CONTROL;
    tx_yaw = MID_CONTROL;
    tx_pitch = MID_CONTROL;
    tx_roll = MID_CONTROL;
    
    command_used_number = 0;
    please_update_sensors = 0;
    user_feedback = 0;
    
    /*--configs--*/
    PID my_pi, my_ro, my_yaw;
    config.pid_roll = &my_ro;
    config.pid_pitch = &my_pi;
    config.pid_yaw = &my_yaw;
    config.flying_mode = X_MODE; //X_MODE or PLUS_MODE
    config.led_mode = 0;
    config.pitch_roll_tx_scale = 4;
    config.yaw_tx_scale = 4;
}

/*--load_eeprom_config--*/
void FlightData::load_from_eeprom( void ){
    
    config.pid_roll->p = EEPROM_read_16(0);
    config.pid_pitch->p = EEPROM_read_16(2);
    config.pid_yaw->p = EEPROM_read_16(4);
   
    
    config.flying_mode = EEPROM_read(6); //X_MODE or PLUS_MODE
    config.led_mode = EEPROM_read(7); 
    config.pitch_roll_tx_scale = EEPROM_read_16(8);
    config.yaw_tx_scale = EEPROM_read_16(10);
    
}