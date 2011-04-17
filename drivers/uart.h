#ifndef _uart_included_h_
#define _uart_included_h_
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif


#include <inttypes.h>

    
void uart_init(uint32_t baud);
void uart_putchar(uint8_t c);
uint8_t uart_getchar(void);
uint8_t uart_available(void);

    

#ifdef __cplusplus
}
#endif
#endif
