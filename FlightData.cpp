

#include <inttypes.h>
#include "pid.h"
#include "FlightData.h"
#include "drivers/eeprom.h"
#include <avr/interrupt.h>

/* ---- constructor ---- */
FlightData::FlightData( void ){
    armed = 0;
    
    SENSOR_DATA zd = {0}; //make a temporary zeroed version
    zero_data = zd; //copy it.
    
    tx_throttle = MIN_CONTROL;
    tx_yaw = MID_CONTROL;
    tx_pitch = MID_CONTROL;
    tx_roll = MID_CONTROL;
    
    command_used_number = 0;
    please_update_sensors = 0;
    user_feedback = 0;
    
    /*--configs--*/
    config.flying_mode = X_MODE; //X_MODE or PLUS_MODE
    config.led_mode = 0;
    config.pitch_roll_tx_scale = 4;
    config.yaw_tx_scale = 4;
}

/*--load_eeprom_config--*/
void FlightData::load_from_eeprom( void ){
    cli(); //turn off interrupts
    config.pid_roll->p = EEPROM_read_16(0);
    config.pid_roll->i = EEPROM_read_16(2);
    config.pid_roll->d = EEPROM_read_16(4);
    config.pid_pitch->p = EEPROM_read_16(6);
    config.pid_pitch->i = EEPROM_read_16(8);
    config.pid_pitch->d = EEPROM_read_16(10);
    config.pid_yaw->p = EEPROM_read_16(12);
    config.pid_yaw->i = EEPROM_read_16(14);
    config.pid_yaw->d = EEPROM_read_16(16);
    
    config.flying_mode = EEPROM_read(18); //X_MODE or PLUS_MODE
    config.led_mode = EEPROM_read(19); 
    config.pitch_roll_tx_scale = EEPROM_read_16(20);
    config.yaw_tx_scale = EEPROM_read_16(22);
    
    zero_data.pitch = EEPROM_read_16(24);
    zero_data.roll = EEPROM_read_16(26);
    zero_data.yaw = EEPROM_read_16(28);
    sei(); //re-enable interrupts
}

/*--store the config to eeprom--*/
void FlightData::store_to_eeprom( void ){
    cli(); //turn off interrupts
    EEPROM_write_16(0, config.pid_roll->p);
    EEPROM_write_16(2, config.pid_roll->i);
    EEPROM_write_16(4, config.pid_roll->d);
    EEPROM_write_16(6, config.pid_pitch->p);
    EEPROM_write_16(8, config.pid_pitch->i);
    EEPROM_write_16(10, config.pid_pitch->d);
    EEPROM_write_16(12, config.pid_yaw->p);
    EEPROM_write_16(14, config.pid_yaw->i);
    EEPROM_write_16(16, config.pid_yaw->d);
    
    EEPROM_write(18, config.flying_mode); //X_MODE or PLUS_MODE
    EEPROM_write(19, config.led_mode); 
    EEPROM_write_16(20, config.pitch_roll_tx_scale);
    EEPROM_write_16(22, config.yaw_tx_scale);
    sei(); //re-enable interrupts
}
 
 void FlightData::store_eeprom_zero_data( void ){
     cli(); //turn off interrupts
     EEPROM_write_16(24, zero_data.pitch );
     EEPROM_write_16(26, zero_data.roll );
     EEPROM_write_16(28, zero_data.yaw );
     sei(); //re-enable interrupts
 }
 
