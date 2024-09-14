#include "sensors/ublox.h"
#include "comminication/uart.h"
#include "setup.h"


#if SETUP_GNSS_TYPE == GNSS_UBX_M8

    uint8_t set_to_921600_baud[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x10, 0x0E, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x4E};
    // uint8_t Disable_GPGGA[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
    // uint8_t Disable_GPGSA[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
    // uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    // uint8_t Disable_GPGLL[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
    // uint8_t Disable_GPRMC[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F};
    // uint8_t Disable_GPVTG[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
    uint8_t set_to_10hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    uint8_t enable_dop[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x15, 0xCC};
    uint8_t enable_pvt[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
    uint8_t dyn_motion_mod_pedestrian[44] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
                                                    0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x82};
    uint8_t full_power[16] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x5A};


#elif SETUP_GNSS_TYPE == GNSS_UBX_M10

    uint8_t set_baudrate_115200[20] = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0xF4, 0xB1};
    uint8_t disable_nmea_messages[17] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02, 0x00, 0x74, 0x10, 0x00, 0x21, 0xC0}; 
    uint8_t set_output_10_hz[18] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x52, 0xC3};
    uint8_t set_dynamic_model_pedestrian[17] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x03, 0xF0, 0x55};
    uint8_t enable_ubx_pvt_msg[17] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x07, 0x00, 0x91, 0x20, 0x01, 0x54, 0x51};
    uint8_t enable_ubx_dop_msg[17] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x39, 0x00, 0x91, 0x20, 0x01, 0x86, 0x4B};

#endif

static uart_data_t uart_data;
static uint8_t ubx_checksum_verify(uint8_t *data, uint8_t dataLenght);
static void parse_ubx_data(gnss_t *gnss_ptr);

void gnss_init()
{

    #if SETUP_GNSS_TYPE == GNSS_UBX_M8

    uart_begin(UART_NUM_0, 9600, SETUP_UART_0_TX_PIN, SETUP_UART_0_RX_PIN, UART_PARITY_DISABLE);
    vTaskDelay(500);

    uart_write(UART_NUM_0, set_to_921600_baud, sizeof(set_to_921600_baud));
    vTaskDelay(500);

    uart_set_baudrate(UART_NUM_0, 921600);
    vTaskDelay(100);

    uart_write(UART_NUM_0, set_to_10hz, sizeof(set_to_10hz));
    vTaskDelay(100);

    uart_write(UART_NUM_0, enable_pvt, sizeof(enable_pvt));
    vTaskDelay(100);

    uart_write(UART_NUM_0, enable_dop, sizeof(enable_dop));
    vTaskDelay(100);

    uart_write(UART_NUM_0, dyn_motion_mod_pedestrian, sizeof(dyn_motion_mod_pedestrian));
    vTaskDelay(100);

    uart_write(UART_NUM_0, full_power, sizeof(full_power));

    #elif SETUP_GNSS_TYPE == GNSS_UBX_M10

    uart_begin(UART_NUM_0, 38400, SETUP_UART_0_TX_PIN, SETUP_UART_0_RX_PIN, UART_PARITY_DISABLE);
    vTaskDelay(1000);

    uart_write(UART_NUM_0, set_baudrate_115200, sizeof(set_baudrate_115200));
    vTaskDelay(500);

    uart_set_baudrate(UART_NUM_0, 115200);
    vTaskDelay(100);

    uart_write(UART_NUM_0, disable_nmea_messages, sizeof(disable_nmea_messages));
    vTaskDelay(100);

    uart_write(UART_NUM_0, set_output_10_hz, sizeof(set_output_10_hz));
    vTaskDelay(100);

    uart_write(UART_NUM_0, set_dynamic_model_pedestrian, sizeof(set_dynamic_model_pedestrian));
    vTaskDelay(100);

    uart_write(UART_NUM_0, enable_ubx_pvt_msg, sizeof(enable_ubx_pvt_msg));
    vTaskDelay(100);

    uart_write(UART_NUM_0, enable_ubx_dop_msg, sizeof(enable_ubx_dop_msg));
    vTaskDelay(100);

    #endif
}

static void parse_ubx_data(gnss_t *gnss_ptr)
{
    static uint8_t PVT_Message_Found = 0;
    static uint8_t DOP_Message_Found = 0;
    static uint8_t prev_data;
    static uint16_t start_message;
    static uint8_t RecBytes[110];
    static uint8_t byte_counter;
    #if SETUP_GNSS_TYPE == GNSS_UBX_M8 || SETUP_GNSS_TYPE == GNSS_NONE
    static const uint8_t PVT_Message_Size = 90;
    #elif SETUP_GNSS_TYPE == GNSS_UBX_M10
    static const uint8_t PVT_Message_Size = 98;
    #endif
    static const uint8_t DOP_Message_Size = 24;

    for (uint8_t i = 0; i < uart_data.lenght; i++)
    {
        if (PVT_Message_Found == 0 && DOP_Message_Found == 0)
        {
            start_message = uart_data.data[i] | prev_data << 8;
            if (start_message == 260)
            {
                DOP_Message_Found = 1;
                RecBytes[0] = 0x01;
                RecBytes[1] = 0x04;
                byte_counter = 2;
                prev_data = 0;
            }
            else if (start_message == 263)
            {
                PVT_Message_Found = 1;
                RecBytes[0] = 0x01;
                RecBytes[1] = 0x07;
                byte_counter = 2;
                prev_data = 0;
            }
            else
            {
                prev_data = uart_data.data[i];
            }
        }
        else if (DOP_Message_Found == 1)
        {
            RecBytes[byte_counter] = uart_data.data[i];
            byte_counter++;

            if (byte_counter == DOP_Message_Size)
            {
                byte_counter = 0;
                DOP_Message_Found = 0;

                if (ubx_checksum_verify(RecBytes, DOP_Message_Size) == 1)
                {
                    gnss_ptr->vdop = RecBytes[14] | RecBytes[15] << 8;
                    gnss_ptr->hdop = RecBytes[16] | RecBytes[17] << 8;
                }
                else
                {
                    printf("GNSS dop message checksum error\n");
                }
            }
        }
        else if (PVT_Message_Found == 1)
        {
            RecBytes[byte_counter] = uart_data.data[i];
            byte_counter++;

            if (byte_counter == PVT_Message_Size)
            {
                byte_counter = 0;
                PVT_Message_Found = 0;

                if (ubx_checksum_verify(RecBytes, PVT_Message_Size) == 1)
                {
                    gnss_ptr->fix = RecBytes[24];
                    gnss_ptr->satCount = RecBytes[27];
                    gnss_ptr->longitude = RecBytes[28] | RecBytes[29] << 8 | RecBytes[30] << 16 | RecBytes[31] << 24;       // deg
                    gnss_ptr->latitude = RecBytes[32] | RecBytes[33] << 8 | RecBytes[34] << 16 | RecBytes[35] << 24;        // deg
                    gnss_ptr->altitude_mm = RecBytes[40] | RecBytes[41] << 8 | RecBytes[42] << 16 | RecBytes[43] << 24;     // mm
                    gnss_ptr->northVel_mms = RecBytes[52] | RecBytes[53] << 8 | RecBytes[54] << 16 | RecBytes[55] << 24;    // mm/s
                    gnss_ptr->eastVel_mms = RecBytes[56] | RecBytes[57] << 8 | RecBytes[58] << 16 | RecBytes[59] << 24;     // mm/s
                    gnss_ptr->downVel_mms = RecBytes[60] | RecBytes[61] << 8 | RecBytes[62] << 16 | RecBytes[63] << 24;     // mm/s
                    gnss_ptr->headingOfMotion = RecBytes[68] | RecBytes[69] << 8 | RecBytes[70] << 16 | RecBytes[71] << 24; // deg
                }
                else
                {
                    printf("GNSS pvt message checksum error\n");
                }
            }
        }
    }
}

void gnss_read(gnss_t *gnss_ptr)
{
    uart_read(UART_NUM_0, &uart_data, 50);
    //printf("%d\n", uart_data.lenght);
    parse_ubx_data(gnss_ptr);
}

static uint8_t ubx_checksum_verify(uint8_t *data, uint8_t dataLenght)
{
    uint8_t CK_A = 0, CK_B = 0;

    for (uint8_t i = 0; i < dataLenght - 2; i++)
    {
        CK_A = CK_A + data[i];
        CK_B = CK_B + CK_A;
    }

    if (CK_A == data[dataLenght - 2] && CK_B == data[dataLenght - 1])
        return 1;
    return 0;
}
