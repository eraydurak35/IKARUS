#pragma once

#include <stdint.h>

uint32_t crc32_update(uint32_t crc, uint8_t data);
uint32_t crc32_compute(const uint8_t *data, int length);
extern char plotter_buffer[128];
extern uint32_t plotter_timestamp_ms;
extern int plotter_len;
extern uint32_t plotter_crc;

#define PLOTTER1(a)                                                          \
    do                                                                       \
    {                                                                        \
        plotter_timestamp_ms = esp_timer_get_time() / 1000;                  \
                                                                             \
        plotter_len = snprintf(plotter_buffer, sizeof(plotter_buffer),       \
                               "%lu,%.3f",                                   \
                               (unsigned long)plotter_timestamp_ms,          \
                               (double)(a));                                 \
                                                                             \
        plotter_crc = crc32_compute((uint8_t *)plotter_buffer, plotter_len); \
        printf("%s*%08lX\n", plotter_buffer, (unsigned long)plotter_crc);    \
    } while (0)

#define PLOTTER2(a, b)                                                       \
    do                                                                       \
    {                                                                        \
        plotter_timestamp_ms = esp_timer_get_time() / 1000;                  \
                                                                             \
        plotter_len = snprintf(plotter_buffer, sizeof(plotter_buffer),       \
                               "%lu,%.3f,%.3f",                              \
                               (unsigned long)plotter_timestamp_ms,          \
                               (double)(a), (double)(b));                    \
                                                                             \
        plotter_crc = crc32_compute((uint8_t *)plotter_buffer, plotter_len); \
        printf("%s*%08lX\n", plotter_buffer, (unsigned long)plotter_crc);    \
    } while (0)

#define PLOTTER3(a, b, c)                                                    \
    do                                                                       \
    {                                                                        \
        plotter_timestamp_ms = esp_timer_get_time() / 1000;                  \
                                                                             \
        plotter_len = snprintf(plotter_buffer, sizeof(plotter_buffer),       \
                               "%lu,%.3f,%.3f,%.3f",                         \
                               (unsigned long)plotter_timestamp_ms,          \
                               (double)(a), (double)(b),                     \
                               (double)(c));                                 \
                                                                             \
        plotter_crc = crc32_compute((uint8_t *)plotter_buffer, plotter_len); \
        printf("%s*%08lX\n", plotter_buffer, (unsigned long)plotter_crc);    \
    } while (0)

#define PLOTTER4(a, b, c, d)                                                 \
    do                                                                       \
    {                                                                        \
        plotter_timestamp_ms = esp_timer_get_time() / 1000;                  \
                                                                             \
        plotter_len = snprintf(plotter_buffer, sizeof(plotter_buffer),       \
                               "%lu,%.3f,%.3f,%.3f,%.3f",                    \
                               (unsigned long)plotter_timestamp_ms,          \
                               (double)(a), (double)(b),                     \
                               (double)(c), (double)(d));                    \
                                                                             \
        plotter_crc = crc32_compute((uint8_t *)plotter_buffer, plotter_len); \
        printf("%s*%08lX\n", plotter_buffer, (unsigned long)plotter_crc);    \
    } while (0)