#ifndef STORAGE_H
#define STORAGE_H

#include <stdio.h>

enum Storage
{
    CONFIG_DATA,
    ACCEL_CALIB_DATA,
    MAG_CALIB_DATA,
    MISSION_DATA
};

uint8_t storage_save(void *ptr, enum Storage type);
uint8_t storage_read(void *ptr, enum Storage type);


#endif