// Automatic Altitude control from ultrasonic sensor
// by Tim O'Brien (t413.com), Sept. 16, 2010.
// some code from http://diydrones.com/profiles/blogs/arduimu-quadcopter-part-iii


#include <avr/interrupt.h> 
#include "altitude_sonar.h"





#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

void timer0_init(void){
    TCCR0A = (1<<WGM01);
	TCCR0A = (1<<WGM00);
    
    TCCR0B = (1<<CS01);
	TCCR0B = (1<<CS00);
	
    TIMSK0 = (1<<TOIE0);
}

SIGNAL(TIMER0_OVF_vect) {
	unsigned long m = timer0_millis; // copy these to local variables so they can be stored in registers
	unsigned char f = timer0_fract; // (volatile variables must be read from memory on every access)
    
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}
	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis(void) {
	unsigned long m;
	uint8_t oldSREG = SREG;
    
	cli();
	m = timer0_millis;
	SREG = oldSREG;
    
	return m;
}







uint8_t altitudeAdjustTo = 0; //0 means disabled. 

#define ALTITUDE_SONAR_INPUT_PIN 8

#define HOVER_THROTTLE 1610     // tested throttle value at hover point

// Sonar variables
//  _ms values record the overflowed counter which in arduino counts milliseconds
//  _t0 values record the counter which overflows every millisecond.
static volatile unsigned long sonar_start_ms;  // saves start of pulse
static volatile unsigned char sonar_start_t0;
static volatile unsigned long sonar_pulse_start_ms; //used after end of pulse
static volatile unsigned char sonar_pulse_start_t0;
static volatile unsigned long sonar_pulse_end_ms;
static volatile unsigned char sonar_pulse_end_t0;
static volatile uint8_t sonar_status = 0;  // true when incoming pulse is high
static volatile uint8_t sonar_new_data = 0; // true when there's new ultrasonic data

extern volatile unsigned long timer0_overflow_count;




/* 
 * Sonar_Init  -  setup inturrupt for to read sonar value
 */
void Sonar_Init(void) {
	//enable pin change inturrupt on Digital Pin 8
	PCMSK0 |= (1 << PCINT0); //enable port 8's interrupt spesifically
	PCICR |= (1 << PCIE0);  //Any change on any enabled PCINT7..0 pin will cause an interrupt now.
}

/* 
 * interrupt function runs when interrupt port 0 (which has P8) is changed
 * enabled by PCICR with individual ports enabled with PCMSK0
 */
ISR(PCINT0_vect) {
	if (PINB & (1<<0)) {  // Pulse start
		sonar_status = 1;
		sonar_start_ms = timer0_overflow_count;
		sonar_start_t0 = TCNT0;
	}
	else if (sonar_status && (PINB & (1<<0))==0) {  // Pulse end?
		sonar_pulse_start_ms = sonar_start_ms;
		sonar_pulse_start_t0 = sonar_start_t0;
		sonar_pulse_end_ms = timer0_overflow_count;
		sonar_pulse_end_t0 = TCNT0;
		sonar_status = 0;
		sonar_new_data = 1;
	}
}

/* 
 * Get_Sonar_Pulse  -  returns integer ultrasonic measurement value
 * returns 0 if there's no new data since last checked
 * returns unsigned 16 bit integer of measurement
 * equation to convert to feet: (0.4336*sonar_value-7.284)/12
 */
uint16_t Get_Sonar_Pulse(void) {
	uint16_t temp;
	if (!sonar_new_data)
		return 0;
	temp = sonar_pulse_end_ms - sonar_pulse_start_ms;   // First calculate Timer0_overflows...
	temp = temp<<8;
	temp += (256-(int)sonar_pulse_start_t0) + (int)sonar_pulse_end_t0;  // Then Timer0 cycles (4us)
	temp = temp/16;  // Convert value (in 4us steps) to centimeters
	//sonar_new_data = 0; // this data's not new anymore
	return (temp);
}

