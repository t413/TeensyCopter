#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    int i;
    DDRD |= (1 << PD6);
    TCCR0A |= (1 << WGM00); // phase correct pwm
    TCCR0A |= (1 << COM0A1); // non-inverting mode
    TCCR0B |= (1 << CS01); // prescale factor of 8
    for(;;) {
    	for (i = 0; i < 256; i++) {
            OCR0A = i; // set duty cycle
            _delay_ms(10);
        }
    }
    return 0;
}
