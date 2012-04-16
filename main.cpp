#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <string.h>
extern "C" {
#include "drivers/usb_debug_only.h"
#include "drivers/print.h"
#include "drivers/analog.h"
#include "sprintf.h"
}   
#include "drivers/uart.h"
#include "drivers/twi.h"
#include "drivers/pwm.h"
#include "interrupt_timer.h"
#include "pid.h"
#include "wii_sensors.h"
#include "FlightData.h"
#include "drivers/xbee_api.h"
#include "drivers/xbee_protocol.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
void process_packet( uint8_t * packet, FlightData * fd );
int16_t limit(int16_t, int16_t, int16_t);
long map(long x, long in_min, long in_max, long out_min, long out_max);
#define FakePWM0(port,pin,val,period,incrcng_cntr) (port = ((incrcng_cntr %(period))>(val)) ? (port|(1<<(pin))) : (port&~(1<<(pin))))

#define zero_motors_and_servos() {\
if (fd.config.flying_mode == TRICOPTER_MODE) { write_motors(SERVO_MIN,SERVO_MIN,SERVO_MIN,SERVO_MID); } \
else if (fd.config.flying_mode == TWINCOPTER_MODE) { write_motors(SERVO_MIN,SERVO_MID,SERVO_MIN,SERVO_MID); }\
else { write_motors_zero(); } }

#define REMOTE_XBEE_ADDR_H 0x13a200
#define REMOTE_XBEE_ADDR_L 0x403D693F
#define LED_W_FRONT 6 //PORTF

int main(void)
{
    CPU_PRESCALE(0);  // set for 16 MHz clock
    /*--setup busses--*/
    pwm_init();
    usb_init();
    twi_init(100000);
    uart_init(115200);
    timer0_init();
    ppm_timing_read_init();
    
    DDRD |= (1<<6); //LED port as output.
    DDRF |= (1<<LED_W_FRONT);
    _delay_ms(10);
    
    /*--setup devices--*/
    init_wii_sensors();
    SENSOR_DATA sensor_vals;
    
    /*--setup data storage--*/
    FlightData fd;  //create the control and settings object. the constructor fills in defaults
    fd.load_from_eeprom(); //pulls stored values from eeprom
    zero_motors_and_servos();
    fd.config.pitch_roll_tx_scale = 2;
    fd.config.yaw_tx_scale = 4;
    fd.config.flying_mode = TRICOPTER_MODE;
    enableFastPWMforServos0_1_2();
    //enableFastPWMforServos3();
    /*fd.config.flying_mode = TRICOPTER_MODE; //TEMPORARY!!!
     fd.pitch.p = fd.roll.p = 300;
     fd.pitch.i = fd.roll.i = 3;
     fd.yaw.p = 900;*/
    /*print("settings loaded: pitch:"); printNumber(fd.pitch.p,DEC);
     print(" roll:"); printNumber(fd.roll.p,DEC);
     print(" yaw:"); printNumber(fd.yaw.p,DEC);
     print("\nmode:"); printNumber(fd.config.flying_mode,DEC);
     print("\n0ero pitch:"); printNumber(fd.zero_data.pitch,DEC);
     print(" roll:"); printNumber(fd.zero_data.roll,DEC);
     print(" yaw:"); printNumber(fd.zero_data.yaw,DEC);
     print("\n");*/
    
	short pitch_offset = 0,roll_offset = 0,yaw_offset = 0, throttle_offset = 0;
    //int32_t alt_error = 0;
    unsigned char packet[128] = "";
    unsigned char packet_position = 0;
    unsigned long i = 0;
    unsigned long last_comm=0, last_sensors=0;
    uint8_t led_clk_scl = 4; //used for LED mode
    uint8_t old_resets = 0;
    
    while(1){
        unsigned long t_tics = tics();
        fd.t_current = t_tics >> 11; //better than dividing by 2000 to get ms
        
        /* ---- Serial Communication ---- */
        unsigned char done = xbee_api_recieve( packet , &packet_position );
        if (done == 0) process_packet( packet, &fd );
        
        
        /* ---- PPM Radio Communication ---- */
        //if we haven't gotten xbee data in the last 200ms OR if we're holding down the trainer button (and still getting xbee data)
        if ( !((fd.t_current - fd.last_t_xbee_packet) < 200) || (fd.last_tx_metadata && ((fd.t_current-fd.last_t_xbee_packet)<200))) {
            if ((fd.t_current-last_comm) >= 30) { //only update every 30ms.
                unsigned long timing_array[9] = {0};
                int8_t resets = get_ppm_timings(timing_array);
                //                if (resets == -2) {
                //                    print("sonar: "); printNumber(timing_array[0],DEC); print("\n");
                //                }else 
                if ((resets != old_resets)&&(timing_array[0] != 0)) {
                    old_resets = resets;
                    fd.last_t_24ghz_packet = fd.t_current; //mark when this packet was recieved
                    
                    //const int16_t min_scl[] = {3679,3583,3677,2534};
                    //const int16_t max_scl[] = {2494,2632,2365,3665};
                    uint16_t rx[6];
                    for (uint8_t i = 0; i<6; i++) {
                        rx[i] = limit(map(timing_array[i],2022,4090,MIN_CONTROL,MAX_CONTROL), MIN_CONTROL,MAX_CONTROL);
                    }
                    
                    if (0) {
                        print("2.4ghz data\t");
                        for (uint8_t i = 0; i<9; i++) { printNumber(rx[i],DEC);print(","); }
                        print("\n");
                    }
                    
                    
                    //if you're holding the button and we've gotten 2.4ghz data in the last 100ms
                    
                    fd.process_analogs( rx[3], rx[1], rx[0], rx[2]); //takes: roll, pitch, yaw, throttle
                    
                    
                    led_clk_scl = 2; //change LED fade speed to be slower.
                } else { led_clk_scl = 4; }
                last_comm = fd.t_current;
            }
        }
        
        
        //PORTD = (i%300 > 100)? (PORTD|(1<<6)) : (PORTD & ~(1<<6));
#define pwm_steps 20
        if (((fd.t_current >> led_clk_scl)/pwm_steps)%2) FakePWM0(PORTD,6,(((fd.t_current >> led_clk_scl))%pwm_steps),pwm_steps,t_tics); //fade in
        else FakePWM0(PORTD,6,pwm_steps-(((fd.t_current >> led_clk_scl))%pwm_steps),pwm_steps,t_tics); //fade out
        
        /* ---- Control System ---- */
        if ((fd.t_current-last_sensors) >= 5) { //only update every 5ms.
            unsigned char data_type = update_wii_data(&sensor_vals, &fd.zero_data);
            if (data_type == 1){
                //pitch_offset =  (-(float)(fd.tx_values[tx_pitch]-1500)-(float)sensor_vals.pitch) * 380.00 / 1500.00;
                //roll_offset  =  (-(float)(fd.tx_values[tx_roll] -1500)+(float)sensor_vals.roll ) * 380.00 / 1500.00;
                //yaw_offset   = -(-(float)(fd.tx_values[tx_yaw]  -1500)+(float)sensor_vals.yaw  ) * 700.00 / 1500.00;
                
                pitch_offset =  fd.pitch.update(sensor_vals.pitch,-(signed)(fd.tx_values[tx_pitch]-1500) );
                roll_offset  =  fd.roll.update(-sensor_vals.roll, -(signed)(fd.tx_values[tx_roll] -1500) );
                yaw_offset   = -fd.yaw.update( -sensor_vals.yaw,  -(signed)(fd.tx_values[tx_yaw]  -1500) );
                
                if (i%150==0) {
                    print("snsrs:<p,r,y> [");
                    printNumber(sensor_vals.pitch,DEC); print(",");
                    printNumber(sensor_vals.roll,DEC); print(",");
                    printNumber(sensor_vals.yaw,DEC); print("]\t\t");
                    print("pid: [");
                    printNumber(pitch_offset,DEC); print(",");
                    printNumber(roll_offset,DEC); print(",");
                    printNumber(yaw_offset,DEC); print("]  ");
                    printNumber(OCR1A,DEC); print(",");
                    printNumber(OCR1B,DEC); print(",");
                    printNumber(OCR1C,DEC); print(",");
                    printNumber(OCR3A,DEC); print(" ");
                    print("tx: r"); 
                    printNumber(fd.tx_values[tx_roll],DEC); print(" p");
                    printNumber(fd.tx_values[tx_pitch],DEC); print(" y");
                    printNumber(fd.tx_values[tx_yaw],DEC); print(" t");
                    printNumber(fd.tx_values[tx_throttle],DEC); print("\n");
                }
            }
            
            //altitude control mode
            /*if ((i%50==0) && fd.tx_metadata) {  
             unsigned long timing_array[8] = {0};
             int8_t resets = get_ppm_timings(timing_array);
             if (resets == -2) { //the sonar module is actually plugged in
             int16_t current_alt = timing_array[0]>>3;
             if (!fd.last_tx_metadata) { //leading edge triggered, set target altitude to this.
             fd.target_alt = current_alt;
             alt_error = 0;
             fd.last_tx_metadata = 1;
             print("target alt: "); printNumber( fd.target_alt ,DEC); print("\n");
             }
             //control system:
             int16_t error = limit( (fd.target_alt - current_alt), -60,60);
             alt_error = limit(alt_error + error, -100, 100);
             throttle_offset = 0.4*error;// + 10*(error - fd.last_alt_error);// + 0.05*alt_error;
             fd.last_alt_error = error;
             print("alt-offset: "); printNumber( throttle_offset ,DEC); 
             print(" (at "); printNumber( current_alt ,DEC); print(") \n");
             }
             } else if (!fd.tx_metadata) { throttle_offset = 0; }*/
            
            
            /* ---- Motor Control ---- */
            if (fd.armed >= 3){
                if (fd.config.flying_mode == PLUS_MODE){
                    //print("PLUS_MODE:");
                    write_servo(0, fd.tx_values[tx_throttle] + pitch_offset - yaw_offset); //front =(right bottom port)
                    write_servo(1, fd.tx_values[tx_throttle] - pitch_offset - yaw_offset); //back  (right top port)
                    write_servo(2, fd.tx_values[tx_throttle] + roll_offset + yaw_offset);  //left  (left bottom port)
                    write_servo(3, fd.tx_values[tx_throttle] - roll_offset + yaw_offset);  //right (left top port)
                }
                else if (fd.config.flying_mode == X_MODE){
                    write_servo(0, fd.tx_values[tx_throttle] + throttle_offset - pitch_offset + roll_offset + yaw_offset); //Right/Rear
                    write_servo(1, fd.tx_values[tx_throttle] + throttle_offset + pitch_offset + roll_offset - yaw_offset); //Right/Front
                    write_servo(2, fd.tx_values[tx_throttle] + throttle_offset - pitch_offset - roll_offset - yaw_offset); //Left/Rear
                    write_servo(3, fd.tx_values[tx_throttle] + throttle_offset + pitch_offset - roll_offset + yaw_offset); //Left/Front
                }
                else if (fd.config.flying_mode == TRICOPTER_MODE){
                    write_servo(0, fd.tx_values[tx_throttle] + pitch_offset/2 + roll_offset ); //right
                    write_servo(1, fd.tx_values[tx_throttle] - pitch_offset); //Back
                    write_servo(2, fd.tx_values[tx_throttle] + pitch_offset/2 - roll_offset ); //left
                    write_servo(3, fd.tri_servo_center + yaw_offset ); //servo
                }
                //pitch down, roll right, yaw right (looking from above) are all positive offset.
                else if (fd.config.flying_mode == TWINCOPTER_MODE){
                    write_servo(0, 1500 + pitch_offset); //servo right
                    write_servo(1, fd.tx_values[tx_throttle] + roll_offset); //motor right
                    write_servo(2, 1500 - yaw_offset + pitch_offset ); //servo left
                    write_servo(3, fd.tx_values[tx_throttle] - roll_offset ); //motor left
                }
                
                if (fd.command_used_number++ > 100) { //loss of communication when still armed
                    fd.armed = 5; //slow shut down mode
                    fd.tx_values[tx_yaw] = fd.tx_values[tx_pitch] = fd.tx_values[tx_roll] = MID_CONTROL; //neutral control values.
                    
                    //subtract from the throttle (see auto_land_plot.numbers document)
                    fd.tx_values[tx_throttle] -= (fd.command_used_number-100)/(1<<6);
                    if (fd.tx_values[tx_throttle] < 1100) {
                        zero_motors_and_servos();
                        fd.armed = 0;
                    } //TODO: change the 2^___ (a set-able variable)
                } 
                //recovery: if we re-establish communication while it's in armed==5 mode then go back to armed=3.
                else if ((fd.armed == 5) && (fd.command_used_number < 5)) { 
                    fd.armed = 3; 
                }
            }
#define ZERO_SENSORS 0xfe
            else { //not armed.
                //motor notification feedback
                if (fd.user_feedback_i) {
                    //sensor calibration feedback.
                    if ((fd.user_feedback_m == ZERO_SENSORS) && (i%100==0)){
                        //print("user feedback! "); printNumber(fd.user_feedback_i,DEC); print("\n");
                        if (fd.user_feedback_i & 0x01) {
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors(SERVO_Notif,SERVO_Notif,SERVO_Notif,SERVO_MID+100);
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_MIN, SERVO_Notif, SERVO_MIN, SERVO_Notif ); 
                        } else zero_motors_and_servos(); //TODO: fix for tricopter..
                        fd.user_feedback_i -= 1;
                    }
                    //pitch feedback
                    else if ((fd.user_feedback_m == P_PITCH) && (i%150==0)) {
                        if (fd.user_feedback_i & 0x01) { 
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors( SERVO_Notif, SERVO_MIN, SERVO_Notif, SERVO_MID );
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_MIN, SERVO_Notif, SERVO_MIN, SERVO_Notif ); 
                        } else {
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors( SERVO_MIN, SERVO_Notif ,SERVO_MIN, SERVO_MID );
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_Notif, SERVO_MIN, SERVO_Notif, SERVO_MIN ); 
                        }
                        fd.user_feedback_i -= 1;
                    }
                    //roll feedback
                    else if ((fd.user_feedback_m == P_ROLL) && (i%150==0)) {
                        if (fd.user_feedback_i & 0x01) { 
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors( SERVO_Notif, SERVO_MIN, SERVO_MIN, SERVO_MID );
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_MIN, SERVO_Notif, SERVO_Notif, SERVO_Notif ); 
                        } else {
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors( SERVO_MIN, SERVO_MIN ,SERVO_Notif, SERVO_MID );
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_Notif, SERVO_Notif, SERVO_MIN, SERVO_MIN ); 
                        }
                        fd.user_feedback_i -= 1;
                    }
                    //yaw feedback
                    else if ((fd.user_feedback_m == P_YAW) && (i%150==0)) {
                        if (fd.user_feedback_i & 0x01) { 
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors( SERVO_MIN, SERVO_MIN, SERVO_MIN, SERVO_MID-100 );
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_MIN, SERVO_Notif, SERVO_Notif, SERVO_MIN ); 
                        } else {
                            if (fd.config.flying_mode == TRICOPTER_MODE) write_motors( SERVO_MIN, SERVO_MIN ,SERVO_MIN, SERVO_MID+100 );
                            else if (fd.config.flying_mode == X_MODE) write_motors( SERVO_Notif, SERVO_MIN, SERVO_MIN, SERVO_Notif ); 
                        }
                        fd.user_feedback_i -= 1;
                    }
                    else if ((fd.user_feedback_m == TRISERVO_CNTR) && (i%200==0)) {
                        if (fd.config.flying_mode == TRICOPTER_MODE) {
                            if ((fd.user_feedback_i % 4) == 0) {
                                write_motors( SERVO_MIN, SERVO_MIN ,SERVO_MIN, SERVO_MID-100 );
                            } else if ((fd.user_feedback_i % 4) == 2) { 
                                write_motors( SERVO_MIN, SERVO_MIN, SERVO_MIN, SERVO_MID+100 );
                            } else { 
                                write_motors( SERVO_MIN, SERVO_MIN, SERVO_MIN, SERVO_MID );
                            }
                        }
                        fd.user_feedback_i -= 1;
                    }
                    else if ((fd.user_feedback_m == LED_MODE) && (i%100==0)) {
                        if (i & 1) { OCR4D = OCR4A = 0; }
                        else { OCR4D = OCR4A = 0xFF; }
                        fd.user_feedback_i -= 1;
                    }
                }
                
                else if (i%50==0) {
                    zero_motors_and_servos();
                }
                if (fd.please_update_sensors){ //allows the user interface task to zero the sensor values.
                    fd.please_update_sensors = 0;
                    char successful_reads = zero_wii_sensors(&fd.zero_data);
                    fd.store_eeprom_zero_data();
                    fd.user_feedback_m = ZERO_SENSORS;
                    fd.user_feedback_i = 6;
                    
                    print("sensors updated, ");
                    printNumber(successful_reads,DEC); 
                    print(" reads, ");
                    printNumber(fd.zero_data.pitch,DEC); print(",");
                    printNumber(fd.zero_data.roll,DEC); print(",");
                    printNumber(fd.zero_data.yaw,DEC); print("\n");
                    
                    fd.store_to_eeprom(); //used to save on-the-fly settings when we zero the sensors h
                }
                fd.command_used_number++;
            }
            
            { /* -- led effects, etc -- */
                uint16_t led_val = fd.config.led_mode % 0xFF;
                int8_t led_type = fd.config.led_mode / 0xFF;
                //        if (fd.t_current < 1500) { //fade in the first 1500ms
                //            OCR4D = map( fd.t_current, 0,1500, 0,limit( (fd.config.led_mode/10), 0, 0xFF ) );
                //        } else {
                if ((fd.user_feedback_m == LED_MODE)&&(fd.user_feedback_i)) { }
                else {
                    if (led_type%4 == 0) { OCR4D = OCR4A = led_val; } //fade in together.
                    else if (led_type%4 == 1) { OCR4D = led_val; OCR4A = (0xFF - led_val); } //fade in opposite of eachother.
                    else if (led_type%4 == 2) { OCR4D = 0xFF;    OCR4A = led_val; }
                    else { OCR4D = led_val; OCR4A = 0xFF; }
                }
            }
            
            i+=5;
            last_sensors = fd.t_current;
        }
        
        
        /*if (i%100 == 0) { //this is 1/100th = 40Hz, max sonar refresh is 20Hz if you want
         uint16_t sonar_val = analogRead(8);
         uint8_t data[2] = { sonar_val, (sonar_val >> 8)  };
         send_packet(TELEM_FEEDBACK, ALTITUDE, data, 2 );
         }*/
        
    }
    return 0;
}

void process_packet( uint8_t * packet, FlightData * fd ) {
    if (packet[3] == 0x88) { //AT Command Response
        xbee_at_cmd_response_t* cmdrx = (xbee_at_cmd_response_t *) &(packet[3]);
        print("localAT");
        uart_putchar(cmdrx->ap_cmd0);uart_putchar(cmdrx->ap_cmd1);
        if (cmdrx->cmd_status) { print(" error "); }
        if (cmdrx->ap_cmd0=='D' && cmdrx->ap_cmd1=='B') { 
            print(" "); printNumber( (cmdrx->data<<8)|(*(&cmdrx->data+1)) ,DEC); print("dBm\n");
        }
    }
    else if (packet[3] == 0x97) { //Remote AT Command Response
        xbee_at_remote_cmd_response_t* cmdrx = (xbee_at_remote_cmd_response_t *) &(packet[3]);
        if (cmdrx->ap_cmd0=='D' && cmdrx->ap_cmd1=='B') { 
            print("recieved at "); printNumber( (cmdrx->data<<8)|(*(&cmdrx->data+1)) ,DEC); print("dBm\n");
        }
    }
    else if (packet[3] == 0x90) { //Packet Received
        //xbee_receive_packet_t* cmdrx = (xbee_receive_packet_t *) &(packet[3]);
        xbee_pkt_tx_t* txp = (xbee_pkt_tx_t*) &(packet[15]); //assume it's a TX packet
        if (txp->type == XBEE_PKT_TX) {
            fd->last_tx_metadata = fd->tx_metadata;
            fd->tx_metadata = txp->meta;
            
            //if you're holding the button and we've gotten 2.4ghz data in the last 100ms
            if ((fd->last_tx_metadata) && ((fd->t_current - fd->last_t_24ghz_packet) < 100)) {} //then do nothing
            else { fd->process_analogs( txp->vals[0],txp->vals[1],txp->vals[2],txp->vals[3]); } //otherwise use these values.
            
            //measure the last recieved communication
            fd->last_t_xbee_packet = fd->t_current; //mark when this packet was recieved
            
            //measure average packet period
            uint16_t dt = (fd->t_current - fd->RxQuality.lastReset);
            if (!(fd->RxQuality.packetCount++)) { fd->RxQuality.lastReset = fd->t_current; }
            else if (dt > 5000) { //every 5 seconds gather stats
                char msg[70] = { XBEE_PKT_TEXT };
                tfp_sprintf(msg+1,(char*)"avrage dt/tx = %d ms\n",dt/fd->RxQuality.packetCount);
                for (uint8_t i=0; i < strlen(msg+1); i++) {
                    usb_debug_putchar( msg[1+i] );
                }
                xbee_api_transmit_request( REMOTE_XBEE_ADDR_H, REMOTE_XBEE_ADDR_L, 3, 1, 0, 0, (uint8_t*)msg, strlen(msg+1)+1);
                fd->RxQuality.packetCount = 0;
                fd->RxQuality.lastReset = fd->t_current;
            }
        }
        else if (txp->type == CONFIG_VAL_RQST) {
            config_val_rqst_t* rx = (config_val_rqst_t*) &(packet[15]);
            print("CONFIG_VAL_RQST "); printNumber(rx->config_type,DEC); print(" (");printNumber(rx->type,DEC); print(")\n");
            
            if (rx->notify){
                fd->user_feedback_i = 6;
                fd->user_feedback_m = rx->config_type;
            }
            int16_t val = 0;
            if (rx->config_type == P_PITCH) val = fd->pitch.p;
            else if (rx->config_type == P_ROLL) val = fd->roll.p;
            else if (rx->config_type == P_YAW) val = fd->yaw.p;
            else if (rx->config_type == TRISERVO_CNTR) { val = fd->tri_servo_center; fd->user_feedback_i = 8; }
            else if (rx->config_type == FLIGHT_MODE) val = fd->config.flying_mode;
            else if (rx->config_type == LED_MODE) val = fd->config.led_mode;
            config_val_send_t tx = {CONFIG_VAL_SEND, rx->config_type, val};
            xbee_api_transmit_request( REMOTE_XBEE_ADDR_H, REMOTE_XBEE_ADDR_L, 0,1,1,0, (uint8_t*)&tx, sizeof(tx) );
        }
        else if (txp->type == CONFIG_VAL_SEND) {
            config_val_send_t* rx = (config_val_send_t*) &(packet[15]);
            print("CONFIG_VAL_SEND recieved ["); printNumber(rx->config_type,DEC); print("] of ");printNumber(rx->value,DEC); print("\n");
            char msg[70] = { XBEE_PKT_TEXT };
            tfp_sprintf(msg+1,(char*)"CONFIG_VAL_SEND recieved [%d] : (%d) \n",rx->config_type, rx->value);
            xbee_api_transmit_request( REMOTE_XBEE_ADDR_H, REMOTE_XBEE_ADDR_L, 3, 1, 0, 0, (uint8_t*)msg, strlen(msg+1)+1);
            
            if (rx->config_type == P_PITCH) fd->pitch.p = rx->value;
            else if (rx->config_type == P_ROLL) fd->roll.p = rx->value;
            else if (rx->config_type == P_YAW) fd->yaw.p = rx->value;
            else if (rx->config_type == TRISERVO_CNTR) fd->tri_servo_center = rx->value;
            else if (rx->config_type == LED_MODE) fd->config.led_mode = rx->value;
            //else if (rx->config_type == FLIGHT_MODE) val = fd->config.flying_mode = rx->value;
        }
    }
    else {
        print("pkt [");
        /*if (packet[2] == SETTINGS_COMM) { print("SETTINGS_COMM,"); uart_putchar(packet[3]); } 
         else if (packet[3] == FULL_REMOTE) print("FULL_REMOTE");
         else {*/
        print("packet: ");
        for (int i = 0; i < (3 + packet[2] + 1); i++){ printNumber(packet[i],HEX); print(" "); }
        //}
        print("\n"); 
    }
}


int16_t limit(int16_t in, int16_t bottom, int16_t upper){
	if (in<bottom) return bottom;
	else if (in>upper) return upper;
	return in;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
