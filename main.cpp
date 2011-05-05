#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
extern "C" {
    #include "drivers/usb_debug_only.h"
    #include "drivers/print.h"
    #include "drivers/analog.h"
}   
#include "drivers/uart.h"
#include "drivers/twi.h"
#include "drivers/pwm.h"
#include "altitude_sonar.h"
#include "pid.h"
#include "ser_pkt.h"
#include "wii_sensors.h"
#include "FlightData.h"
#include "process_uart_commands.h"
#include "ser_pkt.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
int16_t limit(int16_t, int16_t, int16_t);
#define FakePWM0(port,val,period) PORTD = ((i %(period))>(val)) ? (PORTD|(1<<(port))) : (PORTD&~(1<<(port)))


int main(void)
{
    CPU_PRESCALE(0);  // set for 16 MHz clock
    /*--setup busses--*/
    pwm_init();
    usb_init();
    twi_init();
    uart_init(115200);
    timer0_init();
    DDRD |= (1<<6); //LED port as output.
    _delay_ms(10);
    
    /*--setup devices--*/
    init_wii_sensors();
    SENSOR_DATA sensor_vals;
    
    /*--setup data storage--*/
    FlightData fd;  //create the control and settings object. the constructor fills in defaults
    PID pitch, roll, yaw, alt; //create PID control system objects
    fd.config.pid_pitch = &pitch; //give flight settings reference to the PID objects ..
    fd.config.pid_roll = &roll;
    fd.config.pid_yaw = &yaw;
    fd.config.pid_alt = &alt;
    fd.load_from_eeprom(); //pulls stored values from eeprom
    if (fd.config.flying_mode == TRICOPTER_MODE) write_motors(SERVO_MID,SERVO_MIN,SERVO_MIN,SERVO_MIN);
    else write_motors_zero();
    
    print("pitch ["); printNumber( pitch.p ,DEC); print(",");
    printNumber( pitch.i ,DEC); print(",");
    printNumber( pitch.d ,DEC); print("]");
    print("flying mode = ");printNumber(fd.config.flying_mode,DEC);print("\n");
    
	short pitch_offset = 0,roll_offset = 0,yaw_offset = 0;
    unsigned char packet[128] = "";
    unsigned char packet_position = 0;
    unsigned long i = 0;
    
    while(1){
        /* ---- Communications ---- */
        unsigned char done = process_incoming_packet( packet , &packet_position );
        if (done == 0) process_packet( packet, &fd );
        //if (done == 0) { printNumber(fd.tx_throttle,DEC); print("\n"); }
        
        //PORTD = (i%300 > 100)? (PORTD|(1<<6)) : (PORTD & ~(1<<6));
        if ((i/20/10)%2) FakePWM0(6,((i/20)%10),10); //fade in
        else FakePWM0(6,10-((i/20)%10),10); //fade out
        
        /* ---- Control System ---- */
        if (i%5==0){ //only update every 5ms.
            unsigned char data_type = update_wii_data(&sensor_vals, &fd.zero_data);
            if (data_type == 1){
                //pitch_offset =  ((signed)(fd.tx_pitch-1500)-sensor_vals.pitch) * 300 /1500.00;
                //roll_offset  =  ((signed)(fd.tx_roll -1500)+sensor_vals.roll ) * 30 /150.00;
                //yaw_offset   = -((signed)(fd.tx_yaw  -1500)-sensor_vals.yaw  ) * 40 /150.00;
                
                pitch_offset =  pitch.update(sensor_vals.pitch, (signed)(fd.tx_pitch-1500) );
                roll_offset  =  roll.update(-sensor_vals.roll,  (signed)(fd.tx_roll -1500) );
                yaw_offset   = -yaw.update(  sensor_vals.yaw,   (signed)(fd.tx_yaw  -1500) );
            }
            /* ---- Motor Control ---- */
            if (fd.armed >= 3){
                if (fd.config.flying_mode == PLUS_MODE){
                    write_servo(0, fd.tx_throttle + pitch_offset - yaw_offset); //front
                    write_servo(1, fd.tx_throttle - pitch_offset - yaw_offset); //back
                    write_servo(2, fd.tx_throttle + roll_offset + yaw_offset);  //left
                    write_servo(3, fd.tx_throttle - roll_offset + yaw_offset);  //right
                }
                else if (fd.config.flying_mode == X_MODE){
                    write_servo(0, fd.tx_throttle + pitch_offset + roll_offset - yaw_offset); //Front = Front/Right
                    write_servo(1, fd.tx_throttle - pitch_offset - roll_offset - yaw_offset); //Back = Left/Rear
                    write_servo(2, fd.tx_throttle - pitch_offset + roll_offset + yaw_offset); //Left = Front/Left
                    write_servo(3, fd.tx_throttle + pitch_offset - roll_offset + yaw_offset); //Right = Right/Rear
                }
                else if (fd.config.flying_mode == TRICOPTER_MODE){
                    write_servo(0, 1500 - yaw_offset); //servo
                    write_servo(1, fd.tx_throttle - pitch_offset); //Back
                    write_servo(2, fd.tx_throttle + pitch_offset/2 - roll_offset ); //left
                    write_servo(3, fd.tx_throttle + pitch_offset/2 + roll_offset ); //right
                }
                
                if (fd.command_used_number++ > 100) { //loss of communication.
                    fd.armed = 5; //slow shut down mode
                    fd.tx_yaw = 1500; fd.tx_pitch = 1500; fd.tx_roll = 1500; //neutral control values.
                    
                    //subtract from the throttle (see auto_land_plot.numbers document)
                    fd.tx_throttle -= (fd.command_used_number-100)/(1<<6);
                    if (fd.tx_throttle < 1100) {
                        if (fd.config.flying_mode == TRICOPTER_MODE) write_motors(SERVO_MID,SERVO_MIN,SERVO_MIN,SERVO_MIN);
                        else write_motors_zero();
                        fd.armed = 0;
                    } //TODO: change the 2^___ (a set-able variable)
                }
            }
            else { //not armed.
                if (i%50==0) {
                    if (fd.config.flying_mode == TRICOPTER_MODE) write_motors(SERVO_MID,SERVO_MIN,SERVO_MIN,SERVO_MIN);
                    else write_motors_zero();
                }
                if (fd.please_update_sensors){ //allows the user interface task to zero the sensor values.
                    fd.please_update_sensors = 0;
                    zero_wii_sensors(&fd.zero_data);
                    fd.store_eeprom_zero_data();
                    //pulse_motors(3, 200);
                }
                fd.command_used_number++;
            }
        }
        
        if (i%100 == 0) { //this is 1/100th = 40Hz, max sonar refresh is 20Hz if you want
            uint16_t sonar_val = analogRead(8);
            
            uint8_t data[2] = { sonar_val, (sonar_val >> 8)  };
            send_packet(TELEM_FEEDBACK, ALTITUDE, data, 2 );
        }
        
        i++;
        _delay_ms(1);
    }
    return 0;
}

int16_t limit(int16_t in, int16_t bottom, int16_t upper){
	if (in<bottom) return bottom;
	else if (in>upper) return upper;
	return in;
}
