//
//  process_uart_commands.c
//  teensy_copter
//
//  Created by Tim O'Brien on 4/13/11.
//  Copyright 2011 t413.com. All rights reserved.
//

#include "process_uart_commands.h"
#include "ser_pkt.h"
#include "drivers/pwm.h"
#include "FlightData.h"

extern "C" {
#include "drivers/usb_debug_only.h"
#include "drivers/print.h"
}   

#define MINIMUM(a,b)		(((a)>(b))? (b):(a))
#define MATCH(a,b)		(strcmp(a,b) == 0)


void process_packet( uint8_t * packet, FlightData * fd ) {
    if (packet[2] == USER_CONTROL){
        if (packet[3] == FULL_REMOTE){
            int16_t values[4] = {0};
            decode_some_int16s(packet+5, values, 4 ); //should be 9.
            fd->tx_roll  =  (values[0]-1500)*4/fd->config.pitch_roll_tx_scale+1500;
            fd->tx_pitch = -(values[1]-1500)*4/fd->config.pitch_roll_tx_scale+1500;
            fd->tx_yaw   =  (values[2]-1500)*4/fd->config.yaw_tx_scale+1500;
            fd->tx_throttle = values[3];
            
            fd->command_used_number = 0;
            
            if (fd->armed == 5) //lost communication, no longer!
            {fd->armed = 3;}
            
            if (fd->tx_throttle < MIN_SAFETY) {
                fd->config.pid_roll->zero(); //zero the integral error
                fd->config.pid_pitch->zero(); //zero the integral error
                fd->config.pid_yaw->zero(); //zero the integral error
                
                // enable flying (arm it) when yaw-> throttle==min.
                if (values[2] > MAX_SAFETY && fd->armed == 1) {
                    fd->armed = 3; //armed=3 means ready to fly
                    //if (fd->telem_mode) rprintf("armed\n");
                }
                //armed=1 means we've gotten one packet with yaw>MINCHECK
                if (values[2] > MAX_SAFETY) fd->armed |= 1;
                
                if (values[2] < MIN_SAFETY)  {  //disarm when yaw
                    fd->armed = 0;
                    write_motors_zero();
                }
                
                if ((values[2] < MIN_SAFETY) && (values[0] > MAX_SAFETY) && (values[1] < MIN_SAFETY))
                { fd->please_update_sensors = 1; } //zero sensors.
            }
            /*if (fd->telem_mode) {
                rprintf("got data! : r:%i p:%i y:%i t:%i\n",
                        fd->tx_roll,
                        fd->tx_pitch,
                        fd->tx_yaw,
                        fd->tx_throttle);
            }*/
        }
    }
    else if (packet[2] == SETTINGS_COMM)
    {
        switch (packet[3])
        {
            case 'X':	//kill signal
            {
                print("kill signal\n");
                fd->armed = 0;
                write_motors_zero();
                break;
            }
            case 'z':	// Zero sensors
            {
                if (! fd->armed)
                    fd->please_update_sensors = 1;
                break;
            }
            /*case 'm':	// pulse single motor
            {
                //FRONTMOTORPIN,REARMOTORPIN, LEFTMOTORPIN, RIGHTMOTORPIN
                uint8_t whichm = *(uint8_t *) (packet+5);
                fd->user_feedback = 10;
                break;
            }*/
            case REMOTE_2_QUAD_SETTINGS: {   // update PID values
                print("REMOTE_2_QUAD_SETTINGS\n");
                int16_t values[15] = {0};
                decode_some_int16s(packet+5, values, packet[4]/2 ); //should be 9.
                
                fd->config.pid_pitch->p = values[0];
                fd->config.pid_pitch->i = values[1];
                fd->config.pid_pitch->d = values[2];
                fd->config.pid_roll->p = values[3];
                fd->config.pid_roll->i = values[4];
                fd->config.pid_roll->d = values[5];
                fd->config.pid_yaw->p = values[6];
                fd->config.pid_yaw->i = values[7];
                fd->config.pid_yaw->d = values[8];
                
                fd->config.pitch_roll_tx_scale = values[9];
                fd->config.yaw_tx_scale = values[10];
                
                fd->config.flying_mode = values[11];
                fd->config.led_mode = values[12];
                
                fd->store_to_eeprom();
                
                print("pitch[] = "); printNumber(values[0],DEC); print(" "); printNumber(values[1],DEC); print(" "); printNumber(values[2],DEC);  print("\n");
                
                /*if (fd->telem_mode) {
                    rprintf("\n updated pids to(*10): %i,%i,%i,%i,%i,%i,%i,%i,%i",
                            fd->config.pid_pitch->p, fd->config.pid_pitch->i, fd->config.pid_pitch->d,
                            fd->config.pid_roll->p, fd->config.pid_roll->i, fd->config.pid_roll->d,
                            fd->config.pid_yaw->p, fd->config.pid_yaw->i, fd->config.pid_yaw->d );
                    rprintf("\nflying_mode=%i led_mode=%i xy-scale:%i,yaw:%i\n\n",fd->config.flying_mode, fd->config.led_mode,
                            fd->config.pitch_roll_tx_scale, fd->config.yaw_tx_scale);
                }*/
                if (! fd->armed) { fd->user_feedback = 12; }
                
                break; // <- NO BREAK, I want it to send the PIDs back.
            }
            case 'p':{   // request for PID values
                print("request for PID values\n");
                int16_t values[] = {
                    fd->config.pid_pitch->p, fd->config.pid_pitch->i, fd->config.pid_pitch->d,
                    fd->config.pid_roll->p, fd->config.pid_roll->i, fd->config.pid_roll->d,
                    fd->config.pid_yaw->p, fd->config.pid_yaw->i, fd->config.pid_yaw->d,
                    fd->config.pitch_roll_tx_scale,
                    fd->config.yaw_tx_scale,
                    fd->config.flying_mode,
                    fd->config.led_mode
                };
                send_some_int16s(SETTINGS_COMM,QUAD_2_REMOTE_SETTINGS,values, sizeof(values));
                break;
            }
            case '$':{	//TEMPORARY!!! pulse pattern to test pin direction.
                fd->user_feedback = 100;
                break;
            }
        }
    }
}

