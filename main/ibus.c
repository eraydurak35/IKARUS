#include "ibus.h"
#include "uart.h"
#include <string.h>
#include "typedefs.h"

static uart_data_t uart_data;
static void process_new_line(uint8_t *bytes, radio_control_t *radio);
static void parse_ibus_data(radio_control_t *radio);

void ibus_init()
{
    uart_begin(UART_NUM_1, 115200, UART1_PIN_RX, UART1_PIN_TX, UART_PARITY_DISABLE);
}

void ibus_receiver_read(radio_control_t *rc)
{
    uart_read(UART_NUM_1, &uart_data, 5);
    parse_ibus_data(rc);
}

static void parse_ibus_data(radio_control_t *radio)
{
    static uint8_t new_line_found = 0;
    static const uint8_t size = 30;
    static uint8_t buff[30] = {0};
    static uint8_t byte_counter = 0;
    static const uint8_t protocol_lenght = 0x20;
    static const uint8_t command_code = 0x40;
    static uint8_t prev_byte = 0;


    for (uint8_t i = 0; i < uart_data.lenght; i++)
    {

        if (new_line_found == 1)
        {
            buff[byte_counter++] = uart_data.data[i];

            if (byte_counter == size)
            {
                byte_counter = 0;
                new_line_found = 0;
                process_new_line(buff, radio);
            }
        }
        else if (uart_data.data[i] == command_code && prev_byte == protocol_lenght)
        {
            new_line_found = 1;
        }
        prev_byte = uart_data.data[i];
    }
}


static void process_new_line(uint8_t *bytes, radio_control_t *radio)
{

 	uint16_t checksum_cal = 0xffff;
	uint16_t checksum_ibus;

    checksum_cal -= 0x20;
    checksum_cal -= 0x40;

	for(int i = 0; i < sizeof(radio_control_t); i++)
	{
		checksum_cal -= bytes[i];
	}

	checksum_ibus = bytes[29] << 8 | bytes[28]; // checksum value from ibus

    if (checksum_ibus == checksum_cal)
    {
        memcpy(radio, bytes, sizeof(radio_control_t));
        //printf("%d\n", radio_ptr->ch0);
    }
}