//
//  process_uart_commands.h
//  teensy_copter
//
//  Created by Tim O'Brien on 4/13/11.
//  Copyright 2011 t413.com. All rights reserved.
//

#ifndef PROCESS_PKT_
#define PROCESS_PKT_
#include <inttypes.h>
#include "FlightData.h"

void process_packet( uint8_t * packet, FlightData * fd );

#endif