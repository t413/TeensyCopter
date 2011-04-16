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
#include "twi.h"
#include "pwm.h"
#include "pid.h"
#include "ser_pkt.h"
#include "wii_sensors.h"
#include "FlightData.h"
#include "process_uart_commands.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
int16_t limit(int16_t, int16_t, int16_t);

void scan(){
    uint8_t data = 0; // not used, just a ptr to feed to twi_writeTo()
    for( uint8_t addr = 1; addr <= 200; addr++ ) {
        uint8_t rc = twi_writeTo(addr, &data, 0, 1);
        if( rc == 0 ) {
            print("\n device found at address ");
            printNumber(addr,HEX); print("\n");
        } else {
            print(" ");
            printNumber(addr,HEX);
        }
        if (addr%5==4) print("\n");
    }
    print("\n");
}

int main(void)
{
    CPU_PRESCALE(0);  // set for 16 MHz clock
    /*--setup busses--*/
    pwm_init();
    usb_init();
    twi_init();
    uart_init(115200);
    _delay_ms(1000);
    print("teensy_copter, welcome.\n");
    
    
    /*--setup devices--*/
    uint8_t res = init_wii_sensors();
    printNumber(res,DEC);
    print(" was res. \n");
    
    SENSOR_DATA zero_data = {0};
    zero_wii_sensors( &zero_data );
    
    
    PID pitch, roll, yaw(10,0,0);
    FlightData fd = {0};
    fd.config.pid_pitch = &pitch;
    fd.config.pid_roll = &roll;
    fd.config.pid_yaw = &yaw;
    

    unsigned char packet[128] = "";
    unsigned char packet_position = 0;
    unsigned long i = 0;
    while(1){
        
        if (i%500 == 0){
            SENSOR_DATA vals;
            res = update_wii_data( &vals, &zero_data);
            
            printNumber(vals.pitch,DEC); print(" ");
            printNumber(vals.roll,DEC); print(" ");
            printNumber(vals.yaw,DEC); print("\n");
        }
        
        /* ---- Communications ---- */
        unsigned char done = process_incoming_packet( packet , &packet_position );
        if (done == 0) process_packet( packet, &fd );
        
        if (done == 0){
            print("th: ");
            printNumber(fd.tx_throttle,DEC);
            print(" ya: ");
            printNumber(fd.tx_yaw,DEC);
            print(" pitch: ");
            printNumber(fd.tx_pitch,DEC);
            print(" roll: ");
            printNumber(fd.tx_roll,DEC);
            print("\n");
        }
        
        //short val = yaw.update(1000,1000);
        //val = 2+val;
        write_servo(0, fd.tx_throttle);
        write_servo(1, fd.tx_throttle);
        write_servo(2, fd.tx_throttle);
        write_servo(3, fd.tx_throttle);
        
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
