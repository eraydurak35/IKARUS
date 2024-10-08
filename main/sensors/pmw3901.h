#ifndef PMW3901_H
#define PMW3901_H
#include <stdio.h>
#include "comminication/uart.h"
#include "typedefs.h"

//=================================//
//   PMW3901U serial description   //
//=================================//
//  byte0: header (0xFE)           //
//  byte1: data size (fixed 0x04)  //
//  byte2: x-motion low byte       //
//  byte3: x-motion high byte      //
//  byte4: y-motion low byte       //
//  byte5: y-motion high byte      //
//  byte6: checksum                //
//  byte7: surface quality         //
//  byte8: footer (0xAA)?(0XBB)    //
//  baud rate: 19200 bps           //
//  data bits: 8                   //
//  stop bits 1                    //
//  parity: no parity              //
//=================================//

#define PMW3901_HEADER 0xFE
#define PMW3901_FOOTER 0xAA//0xBB

void pmw3901_init();
void pmw3901_read(pmw3901_t *pmw);

#endif