#pragma once

#include <stdint.h>

uint32_t crc32_update(uint32_t crc, uint8_t data);
uint32_t crc32_compute(const uint8_t *data, int length);
