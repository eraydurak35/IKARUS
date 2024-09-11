#include <stdio.h>
#include "sensors/pmw3901.h"
#include "comminication/uart.h"
#include "typedefs.h"

static uart_data_t uart_data;
static uint8_t data_frame_buff[8] = {0};
static uint8_t is_pmw3901_header_found = 0;
static uint8_t pwm3901_byte_counter = 0;

static void parse_pmw3901_data(pmw3901_t *pmw);

void pmw3901_init()
{
    uart_begin(UART_NUM_2, 19200, UART2_PIN_TX, UART2_PIN_RX, UART_PARITY_DISABLE);
}

void pmw3901_read(pmw3901_t *pmw)
{
    uart_read(UART_NUM_2, &uart_data, 5);
    parse_pmw3901_data(pmw);
}

static void parse_pmw3901_data(pmw3901_t *pmw)
{
    for (uint8_t i = 0; i < uart_data.lenght; i++)
    {
        if (is_pmw3901_header_found == 1)
        {
            data_frame_buff[pwm3901_byte_counter] = uart_data.data[i];
            pwm3901_byte_counter++;

            if (pwm3901_byte_counter >= 8)
            {
                pwm3901_byte_counter = 0;
                is_pmw3901_header_found = 0;

                uint8_t checksum = data_frame_buff[1] + data_frame_buff[2] + data_frame_buff[3] + data_frame_buff[4];
                if (checksum == data_frame_buff[5] && data_frame_buff[7] == PMW3901_FOOTER)
                {
                    pmw->raw_x_cpi = data_frame_buff[2] << 8 | data_frame_buff[1];
                    pmw->raw_y_cpi = data_frame_buff[4] << 8 | data_frame_buff[3];
                    // "quality" verisini basit bir alçak geçiren filtreden geçiriyoruz
                    pmw->quality += ((data_frame_buff[6] / 2.0f) - pmw->quality) * 0.1f;
                }
            }
        }
        else if (uart_data.data[i] == PMW3901_HEADER)
        {
            is_pmw3901_header_found = 1;
        }
    }
}

