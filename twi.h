
/* original code from http://subversion.isdc.unige.ch/polar/trac/browser/trunk/eqm/software/teensy/twi.c?rev=314 */

#ifndef TWI_H_
#define TWI_H_
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif
    

void twi_init(unsigned int speed);
void twi_exchange(unsigned char *msg);

    
    
#ifdef __cplusplus
}
#endif
#endif
