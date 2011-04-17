// Automatic Altitude control from ultrasonic sensor
// by Tim O'Brien (t413.com), Sept. 16, 2010.


#ifndef ALTITUDE_SONAR_H
#define ALTITUDE_SONAR_H
//allow for easy C++ compile
#ifdef __cplusplus
extern "C" {
#endif

    
void timer0_init(void);
unsigned long millis(void);
    
    
void Sonar_Init(void);
uint16_t Get_Sonar_Pulse(void);
#define sonarToInches(x) (0.4336*((float)(x))-7.284)

    

#ifdef __cplusplus
}
#endif
#endif