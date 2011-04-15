#include <avr/io.h>
#include "twi.h"

#define ENABTW ((1<<TWINT)|(1<<TWEN)|(0<<TWIE)) // 0x80 4 1
#define START TWCR = (ENABTW|(1<<TWSTA))        // 0x20
#define STOP TWCR = (ENABTW|(1<<TWSTO)) // 0x10
#define SEND(x)  TWDR = x;  TWCR = ENABTW;
#define RECV(ack) TWCR = ENABTW | (ack? (1<<TWEA) : 0 );
unsigned char twista;
unsigned twitmo;
#define WAIT twitmo=0; while (!((twista = TWCR) & (1 << TWINT)) && ++twitmo);

/////===================================////////////////////
void twi_init(unsigned int speed){
  DDRD &= ~((1<<0) | (1<<1)); //input for pins D0 and D1
  PORTD |= (1<<0) | (1<<1);  //pullup for pins D0 and D1

  TWBR = (((F_CPU/(speed*1000))-16)/2); // 400 khz
  TWCR |= (1 << TWEN);
}

void twi_exchange(unsigned char *msg){
    unsigned int mlen, rdwrf;

    while ((mlen = *msg++)) {
      rdwrf = *msg & 1;//1 mean read 0 mean write
      START;
      WAIT;
      do {
          SEND(*msg++);
          WAIT;
          // should check for ACK - twista == SAWA or SDWA
      } while (--mlen && !rdwrf);
      // read
      while (mlen--) {
	RECV(mlen);
	WAIT;
	*msg++ = TWDR;
      }
    }
    STOP;
}
