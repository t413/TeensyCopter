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
#define FakePWM0(port,val,period,incrcng_cntr) PORTD = ((incrcng_cntr %(period))>(val)) ? (PORTD|(1<<(port))) : (PORTD&~(1<<(port)))

#define zero_motors_and_servos() {\
if (fd.config.flying_mode == TRICOPTER_MODE) { write_motors(SERVO_MID,SERVO_MIN,SERVO_MIN,SERVO_MIN); } \
else if (fd.config.flying_mode == TWINCOPTER_MODE) { write_motors(SERVO_MID,SERVO_MIN,SERVO_MID,SERVO_MIN); }\
else { write_motors_zero(); } }


int main(void)
{
    CPU_PRESCALE(0);  // set for 16 MHz clock
    /*--setup busses--*/
    pwm_init();
    usb_init();
    twi_init(10000);
    uart_init(115200);
    timer0_init();
    ppm_timing_read_init();
    
    DDRD |= (1<<6); //LED port as output.
    _delay_ms(10);
    
    /*--setup devices--*/
    init_wii_sensors();
    SENSOR_DATA sensor_vals;
    
    /*--setup data storage--*/
    FlightData fd;  //create the control and settings object. the constructor fills in defaults
    //fd.load_from_eeprom(); //pulls stored values from eeprom
    fd.config.flying_mode = X_MODE; //TEMPORARY!!!
    zero_motors_and_servos();
    
    fd.pitch.p = fd.roll.p = 300;
    fd.pitch.i = fd.roll.i = 3;
    fd.yaw.p = 900;

    
	short pitch_offset = 0,roll_offset = 0,yaw_offset = 0;
    unsigned char packet[128] = "";
    unsigned char packet_position = 0;
    unsigned long i = 0;
    unsigned long last_comm=0, last_sensors=0;
    uint8_t led_clk_scl = 4; //used for LED mode
    uint8_t old_resets = 0;
    
    while(1){
        unsigned long t_tics = tics();
        unsigned long t_current = t_tics >> 11; //better than dividing by 2000 to get ms
        
        /* ---- Serial Communication ---- */
        unsigned char done = xbee_api_recieve( packet , &packet_position );
        if (done == 0) process_packet( packet, &fd );
        
        /* ---- PPM Radio Communication ---- */
        if ((t_current-last_comm) >= 50) { //only update every 50ms.
            unsigned long timing_array[8] = {0};
            uint8_t resets = get_ppm_timings(timing_array);
            if ((resets != old_resets)&&(resets >= 0)&&(timing_array[0] != 0)) {
                old_resets = resets;
                
                const int16_t min_scl[] = {3679,3583,3677,2534};
                const int16_t max_scl[] = {2494,2632,2365,3665};
                for (uint8_t i = 0; i<4; i++) {
                    fd.tx_values[i] = limit(map(timing_array[i],min_scl[i],max_scl[i],MIN_CONTROL,MAX_CONTROL), MIN_CONTROL,MAX_CONTROL);
                }
                
                //print values.
                printNumber(timing_array[0],DEC); print(",");
                printNumber(timing_array[1],DEC); print(",");
                printNumber(timing_array[2],DEC); print(",");
                printNumber(timing_array[3],DEC); print(",");
                print("\n");
                
                
                //code pulled from process_packet()
                if (1) { 
                    fd.command_used_number = 0;
                    if (fd.armed == 5) {fd.armed = 3;} //lost communication, no longer!
                    if (fd.tx_values[tx_throttle] < MIN_SAFETY) {
                        fd.roll.zero(); fd.pitch.zero(); fd.yaw.zero(); //0 integral error
                        if (fd.tx_values[tx_yaw] > MAX_SAFETY && fd.armed == 1) { // enable flying (arm it) when yaw-> throttle==min.
                            fd.armed = 3; //armed=3 means ready to fly
                            fd.user_feedback_i = fd.user_feedback_m = 0; //clear any user feedback.
                            print("armed\n");
                        }
                        if (fd.tx_values[tx_yaw] > MAX_SAFETY) fd.armed |= 1;
                        if (fd.tx_values[tx_yaw] < MIN_SAFETY)  {  //disarm when yaw
                            print("DIS-armed\n");
                            fd.armed = 0;
                            zero_motors_and_servos();
                        }
                        if ((fd.tx_values[tx_yaw] < MIN_SAFETY) && (fd.tx_values[tx_roll] > MAX_SAFETY) && (fd.tx_values[tx_pitch] < MIN_SAFETY))
                        { fd.please_update_sensors = 1; } //zero sensors.
                    }
                }
                led_clk_scl = 2; //change LED fade speed to be slower.
            } else { led_clk_scl = 4; }
            last_comm = t_current;
        }

        
        //PORTD = (i%300 > 100)? (PORTD|(1<<6)) : (PORTD & ~(1<<6));
        #define pwm_steps 20
        if (((t_current >> led_clk_scl)/pwm_steps)%2) FakePWM0(6,(((t_current >> led_clk_scl))%pwm_steps),pwm_steps,t_tics); //fade in
        else FakePWM0(6,pwm_steps-(((t_current >> led_clk_scl))%pwm_steps),pwm_steps,t_tics); //fade out
        
        /* ---- Control System ---- */
        if ((t_current-last_sensors) >= 5) { //only update every 5ms.
            unsigned char data_type = update_wii_data(&sensor_vals, &fd.zero_data);
            if (data_type == 1){
                //pitch_offset =  (-(float)(fd.tx_values[tx_pitch]-1500)-(float)sensor_vals.pitch) * 380.00 / 1500.00;
                //roll_offset  =  (-(float)(fd.tx_values[tx_roll] -1500)+(float)sensor_vals.roll ) * 380.00 / 1500.00;
                //yaw_offset   = -(-(float)(fd.tx_values[tx_yaw]  -1500)+(float)sensor_vals.yaw  ) * 700.00 / 1500.00;
                
                pitch_offset =  fd.pitch.update(sensor_vals.pitch,-(signed)(fd.tx_values[tx_pitch]-1500) );
                roll_offset  =  fd.roll.update(-sensor_vals.roll, -(signed)(fd.tx_values[tx_roll] -1500) );
                yaw_offset   = -fd.yaw.update( -sensor_vals.yaw,  -(signed)(fd.tx_values[tx_yaw]  -1500) );
                
                if (0){//i%150==0) {
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
                    printNumber(OCR3A,DEC); print("\n");
                }
            }
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
                    write_servo(0, fd.tx_values[tx_throttle] - pitch_offset + roll_offset + yaw_offset); //Right/Rear
                    write_servo(1, fd.tx_values[tx_throttle] + pitch_offset + roll_offset - yaw_offset); //Right/Front
                    write_servo(2, fd.tx_values[tx_throttle] - pitch_offset - roll_offset - yaw_offset); //Left/Rear
                    write_servo(3, fd.tx_values[tx_throttle] + pitch_offset - roll_offset + yaw_offset); //Left/Front
                }
                /*right bottom port
                 right top port   
                 left bottom port 
                 left top port    */
                else if (fd.config.flying_mode == TRICOPTER_MODE){
                    write_servo(0, 1500 - yaw_offset); //servo
                    write_servo(1, fd.tx_values[tx_throttle] - pitch_offset); //Back
                    write_servo(2, fd.tx_values[tx_throttle] + pitch_offset/2 - roll_offset ); //left
                    write_servo(3, fd.tx_values[tx_throttle] + pitch_offset/2 + roll_offset ); //right
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
            else { //not armed.
                if (fd.user_feedback_i) {
                    if ((fd.user_feedback_m == 0) && (i%100==0)){
                        //print("user feedback! "); printNumber(fd.user_feedback_i,DEC); print("\n");
                        if (fd.user_feedback_i & 0x01) write_motors( SERVO_Notif, SERVO_Notif, SERVO_Notif, SERVO_Notif );
                        else zero_motors_and_servos(); //TODO: fix for tricopter..
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
                    fd.user_feedback_i = 6;

                    print("sensors updated, ");
                    printNumber(successful_reads,DEC); 
                    print(" reads, ");
                    printNumber(fd.zero_data.pitch,DEC); print(",");
                    printNumber(fd.zero_data.roll,DEC); print(",");
                    printNumber(fd.zero_data.yaw,DEC); print("\n");
                }
                fd.command_used_number++;
            }
            i+=5;
            last_sensors = t_current;
        }

        
        /*if (i%100 == 0) { //this is 1/100th = 40Hz, max sonar refresh is 20Hz if you want
            uint16_t sonar_val = analogRead(8);
            uint8_t data[2] = { sonar_val, (sonar_val >> 8)  };
            send_packet(TELEM_FEEDBACK, ALTITUDE, data, 2 );
        }*/
        
        _delay_us(100);
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
            int16_t*values = txp->vals;
            fd->tx_values[tx_roll]  = (values[0]-1500)*4/fd->config.pitch_roll_tx_scale+1500;
            fd->tx_values[tx_pitch] = (values[1]-1500)*4/fd->config.pitch_roll_tx_scale+1500;
            fd->tx_values[tx_yaw]   = (values[2]-1500)*4/fd->config.yaw_tx_scale+1500;
            fd->tx_values[tx_throttle] = values[3];
            
            
            fd->command_used_number = 0;
            
            if (fd->armed == 5) //lost communication, no longer!
            {fd->armed = 3;}
            
            if (fd->tx_values[tx_throttle] < MIN_SAFETY) {
                fd->roll.zero(); //zero the integral error
                fd->pitch.zero(); //zero the integral error
                fd->yaw.zero(); //zero the integral error
                
                // enable flying (arm it) when yaw-> throttle==min.
                if (values[2] > MAX_SAFETY && fd->armed == 1) {
                    fd->armed = 3; //armed=3 means ready to fly
                    fd->user_feedback_i = fd->user_feedback_m = 0; //clear any user feedback.
                    //if (fd->telem_mode) rprintf("armed\n");
                }
                //armed=1 means we've gotten one packet with yaw>MINCHECK
                if (values[2] > MAX_SAFETY) fd->armed |= 1;
                
                if (values[2] < MIN_SAFETY)  {  //disarm when yaw
                    fd->armed = 0;
                    if (fd->config.flying_mode == TRICOPTER_MODE) write_motors(SERVO_MID,SERVO_MIN,SERVO_MIN,SERVO_MIN);
                    else write_motors_zero();
                }
                
                if ((values[2] < MIN_SAFETY) && (values[0] > MAX_SAFETY) && (values[1] < MIN_SAFETY))
                { fd->please_update_sensors = 1; } //zero sensors.
            }
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
