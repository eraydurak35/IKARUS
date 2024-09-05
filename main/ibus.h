#ifndef IBUS_H
#define IBUS_H

#include "uart.h"
#include "typedefs.h"

void ibus_init();
void ibus_receiver_read(radio_control_t *rc);


#endif