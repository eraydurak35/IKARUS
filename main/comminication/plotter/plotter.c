#include "plotter.h"
#include <string.h>

char plotter_buffer[128];
uint32_t plotter_timestamp_ms;
int plotter_len;
uint32_t plotter_crc;

uint32_t crc32_update(uint32_t crc, uint8_t data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++)
        crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
    return crc;
}

uint32_t crc32_compute(const uint8_t *data, int length)
{
    uint32_t crc = ~0U; // initial value = 0xFFFFFFFF
    for (int i = 0; i < length; i++)
        crc = crc32_update(crc, data[i]);
    return ~crc; // final XOR
}
