#ifndef SBUS_H
#define SBUS_H

#include "comminication/uart.h"
#include "typedefs.h"

void sbus_init();
void sbus_receiver_read(radio_control_t *rc);


#endif