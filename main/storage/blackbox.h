#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdio.h>
#include "typedefs.h"

void blackbox_init(flight_t *flt, imu_t *imu);
void blackbox_save();
uint8_t blackbox_open();
uint8_t blackbox_print();



#endif