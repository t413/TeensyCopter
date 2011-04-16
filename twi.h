

#ifndef TWI_H_
#define TWI_H_
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif
    

#include <inttypes.h>
    
    
#ifndef TWI_BUFFER_LENGTH
#define TWI_BUFFER_LENGTH 32
#endif
    
#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4
    
    void twi_init(void);
    void twi_setAddress(uint8_t);
    uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t);
    uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t);
    uint8_t twi_transmit(uint8_t*, uint8_t);
    void twi_attachSlaveRxEvent( void (*)(uint8_t*, int) );
    void twi_attachSlaveTxEvent( void (*)(void) );
    void twi_reply(uint8_t);
    void twi_stop(void);
    void twi_releaseBus(void);
    
#ifdef __cplusplus
}
#endif
#endif
