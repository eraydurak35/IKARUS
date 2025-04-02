#include "sbus.h"
#include "comminication/uart.h"
#include <string.h>
#include "typedefs.h"
#include "setup.h"

static uart_data_t uart_data;
static void process_new_line(uint8_t *bytes, radio_control_t *radio);
static void parse_sbus_data(radio_control_t *radio);
uint16_t scale_rc_channel(uint16_t value);

void sbus_init()
{
    uart_begin(UART_NUM_1, 100000, SETUP_UART_1_TX_PIN, SETUP_UART_1_RX_PIN, UART_PARITY_EVEN, UART_STOP_BITS_2);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
}

void sbus_receiver_read(radio_control_t *rc)
{
    uart_read(UART_NUM_1, &uart_data, 5);
    parse_sbus_data(rc);
}

static void parse_sbus_data(radio_control_t *radio)
{
    static uint8_t new_line_found = 0;
    static const uint8_t size = 24;
    static uint8_t buff[24] = {0};
    static uint8_t byte_counter = 0;
    static const uint8_t start_byte = 0x0F;
    static const uint8_t end_byte = 0x00;

    for (uint8_t i = 0; i < uart_data.lenght; i++)
    {

        if (new_line_found == 1)
        {
            buff[byte_counter++] = uart_data.data[i];

            if (byte_counter == size)
            {
                if (buff[23] == end_byte)
                {
                    process_new_line(buff, radio);
                }

                byte_counter = 0;
                new_line_found = 0;

            }
        }
        else if (uart_data.data[i] == start_byte)
        {
            new_line_found = 1;
        }
    }
}


static void process_new_line(uint8_t *bytes, radio_control_t *radio)
{
    static uint16_t raw_channels[14] = {0};

    raw_channels[0] = ((bytes[1] & 0x07U) << 8U) | bytes[0];
    raw_channels[1] = ((bytes[2] & 0x3FU) << 5U) | ((bytes[1] & 0xF8U) >> 3U);
    raw_channels[2] = ((bytes[4] & 0x01U) << 10U) | ((bytes[3] & 0xFFU) << 2U) | ((bytes[2] & 0xC0U) >> 6U);
    raw_channels[3] = ((bytes[5] & 0x0FU) << 7U) | ((bytes[4] & 0xFEU) >> 1U);
    raw_channels[4] = ((bytes[6] & 0x7FU) << 4U) | ((bytes[5] & 0xF0U) >> 4U);
    raw_channels[5] = ((bytes[8] & 0x03U) << 9U) | ((bytes[7] & 0xFFU) << 1U) | ((bytes[6] & 0x80U) >> 7U);
    raw_channels[6] = ((bytes[9] & 0x1FU) << 6U) | ((bytes[8] & 0xFCU) >> 2U);
    raw_channels[7] = ((bytes[10] & 0xFFU) << 3U) | ((bytes[9] & 0xE0U) >> 5U);
    raw_channels[8] = ((bytes[12] & 0x03U) << 8U) | bytes[11];
    raw_channels[9] = ((bytes[13] & 0x3FU) << 5U) | ((bytes[12] & 0xF8U) >> 3U);
    raw_channels[10] = ((bytes[15] & 0x01U) << 10U) | ((bytes[14] & 0xFFU) << 2U) | ((bytes[13] & 0xC0U) >> 6U);
    raw_channels[11] = ((bytes[16] & 0x0FU) << 7U) | ((bytes[15] & 0xFEU) >> 1U);
    raw_channels[12] = ((bytes[17] & 0x7FU) << 4U) | ((bytes[16] & 0xF0U) >> 4U);
    raw_channels[13] = ((bytes[19] & 0x03U) << 9U) | ((bytes[18] & 0xFFU) << 1U) | ((bytes[17] & 0x80U) >> 7U);

    for (uint8_t i = 0; i < 14; i++)
    {
        radio->channel[i] = scale_rc_channel(raw_channels[i]);
    }

/*     static int i = 0;
    i++;
    if (i > 14)
    {
        i = 0;
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", radio->channel[0], radio->channel[1], radio->channel[2], radio->channel[3], radio->channel[4], radio->channel[5], radio->channel[6], radio->channel[7], radio->channel[8], radio->channel[9], radio->channel[10], radio->channel[11], radio->channel[12], radio->channel[13]);
    } */
    // 240 min / 1023 middle / 1807 max
    //printf("%d,%d,%d,%d\n", radio->channel[0], radio->channel[1], radio->channel[2], radio->channel[3]);
    //printf("%d,%d,%d,%d\n", radio->channel[4], radio->channel[5], radio->channel[6], radio->channel[7]);
    //printf("%d,%d,%d,%d\n", radio->channel[5], radio->channel[6], radio->channel[7], radio->channel[8]);
}


// Fonksiyon: Bir değeri bir aralıktan diğerine ölçeklendirir
uint16_t scale_rc_channel(uint16_t value)
{
    // Eski aralıktaki değeri 0-1 aralığına normalize et
    float normalized = (float)(value - 240) / (float)(1807 - 240);
    
    // Yeni aralığa ölçeklendir
    uint16_t scaled_value = (normalized * (2000 - 1000) + 1000);
    
    // Yeni aralığın sınırlarını kontrol et
    if (scaled_value < 1000) return 1000;
    if (scaled_value > 2000) return 2000;
    
    return scaled_value;
}