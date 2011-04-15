#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
extern "C" {
    #include "usb_debug_only.h"
    #include "print.h"
}   
#include "uart.h"
#include "pwm.h"
#include "pid.h"
#include "ser_pkt.h"
#include "FlightData.h"
#include "process_uart_commands.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
int16_t limit(int16_t, int16_t, int16_t);

int main(void)
{
    // set for 16 MHz clock
    CPU_PRESCALE(0);
    pwm_init();
    usb_init();
    uart_init(115200);
    
    _delay_ms(1000);
    
    print("yo\n");
    
    PID pitch, roll, yaw(10,0,0);
    
    FlightData fd = {0};
    fd.config.pid_pitch = &pitch;
    fd.config.pid_roll = &roll;
    fd.config.pid_yaw = &yaw;
    
    

    unsigned char packet[128] = "";
    unsigned char packet_position = 0;
    unsigned long i = 0;
    while(1){
        /* ---- Communications ---- */
        unsigned char done = process_incoming_packet( packet , &packet_position );
        if (done == 0) process_packet( packet, &fd );
        
        //short val = yaw.update(1000,1000);
        //val = 2+val;
        write_servo(0, 1000 + (i/3+250)%1000);
        write_servo(1, 1000 + (i/3+500)%1000);
        write_servo(2, 1000 + (i/3+750)%1000);
        write_servo(3, 1000 + (i/3)%1000);
        //OCR1A = (i*40)%40000; //for full led pwm
        
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
