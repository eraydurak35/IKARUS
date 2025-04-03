#ifndef ESP_NOW_COMM_H
#define ESP_NOW_COMM_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "typedefs.h"

#define CHANNEL 6
#define DATARATE WIFI_PHY_RATE_54M//WIFI_PHY_RATE_24M

#define TELEM_HEADER 0xFF
#define CONF_HEADER 0XFE
#define WP_HEADER 0xFD
#define MTR_TEST_HEADER 0xFC

extern const uint8_t ground_station_mac_address[6];

void esp_now_comm_init();
//void esp_now_send_telemetry();
//void esp_now_send_config(config_t *conf);
//void esp_now_send_mission();
//const uint8_t *get_mag_data();
//const uint8_t *get_acc_data();
//void esp_now_send_motor_test_result(float *result);

#endif