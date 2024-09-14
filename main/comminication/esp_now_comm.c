#include <string.h>
#include "comminication/esp_now_comm.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "esp_timer.h"
#include <math.h>
#include "storage/nv_storage.h"

static const uint8_t drone_mac_address[6] = {0x04, 0x61, 0x05, 0x05, 0x3A, 0xE4};
static const uint8_t ground_station_mac_address[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peerInfo;

static config_t *config_ptr = NULL;
static waypoint_t *waypoint_ptr = NULL;
static telemetry_t *telemetry_ptr = NULL;
static flight_t *flight_ptr = NULL;
static states_t *state_ptr = NULL;
static imu_t *imu_ptr = NULL;
static magnetometer_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static target_t *target_ptr = NULL;
static gnss_t *gnss_ptr = NULL;
static pmw3901_t *flow_ptr = NULL;
static range_finder_t *range_ptr = NULL;
static uint8_t *motor_test_num_ptr = NULL;
static const uint8_t *mag_data;
static const uint8_t *acc_data;
static gamepad_t *gamepad_ptr = NULL;

static uint8_t recieved_command_flag;

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void parse_mission_data(const uint8_t *data, uint8_t len);
static void respond_to_requests();


void esp_now_comm_init(config_t *cfg, waypoint_t *wp, uint8_t *mtr_tst, telemetry_t *telem, flight_t *flt, states_t *stt, imu_t *imu, magnetometer_t *mag, bmp390_t *baro, gnss_t *gnss, pmw3901_t *flow, range_finder_t *range, target_t *target, gamepad_t *gmpd)
{
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set Wi-Fi protocol to long range mode
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, &drone_mac_address[0]));

    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
    memcpy(peerInfo.peer_addr, ground_station_mac_address, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    config_ptr = cfg;
    waypoint_ptr = wp;
    motor_test_num_ptr = mtr_tst;
    gamepad_ptr = gmpd;
    telemetry_ptr = telem;
    flight_ptr = flt;
    state_ptr = stt;
    imu_ptr = imu;
    mag_ptr = mag;
    baro_ptr = baro;
    target_ptr = target;
    gnss_ptr = gnss;
    flow_ptr = flow;
    range_ptr = range;
}

void esp_now_send_telemetry()
{
    telemetry_ptr->battery_voltage = flight_ptr->battery_voltage;
    telemetry_ptr->pitch = state_ptr->pitch_deg;
    telemetry_ptr->roll = state_ptr->roll_deg;
    telemetry_ptr->heading = state_ptr->heading_deg;
    telemetry_ptr->gyro_x_dps = state_ptr->pitch_dps * 100.0f;
    telemetry_ptr->gyro_y_dps = state_ptr->roll_dps * 100.0f;
    telemetry_ptr->gyro_z_dps = state_ptr->yaw_dps * 100.0f;
    telemetry_ptr->acc_x_ms2 = imu_ptr->accel_ms2[X] * 400.0f;
    telemetry_ptr->acc_y_ms2 = imu_ptr->accel_ms2[Y] * 400.0f;
    telemetry_ptr->acc_z_ms2 = imu_ptr->accel_ms2[Z] * 400.0f;
    telemetry_ptr->imu_temperature = imu_ptr->temp_mC;
    telemetry_ptr->mag_x_mgauss = mag_ptr->axis[X];
    telemetry_ptr->mag_y_mgauss = mag_ptr->axis[Y];
    telemetry_ptr->mag_z_mgauss = mag_ptr->axis[Z];
    telemetry_ptr->barometer_pressure = baro_ptr->press * 10.0f;
    telemetry_ptr->barometer_temperature = baro_ptr->temp * 100.0f;
    telemetry_ptr->altitude = baro_ptr->altitude_m * 100.0f;
    telemetry_ptr->altitude_calibrated = state_ptr->altitude_m * 100.0f;
    telemetry_ptr->velocity_x_ms = state_ptr->vel_forward_ms * 1000.0f;
    telemetry_ptr->velocity_y_ms = state_ptr->vel_right_ms * 1000.0f;
    telemetry_ptr->velocity_z_ms = state_ptr->vel_up_ms * 1000.0f;
    telemetry_ptr->target_pitch = target_ptr->pitch_deg;
    telemetry_ptr->target_roll = target_ptr->roll_deg;
    telemetry_ptr->target_heading = target_ptr->heading_deg;
    telemetry_ptr->target_pitch_dps = target_ptr->pitch_dps;
    telemetry_ptr->target_roll_dps = target_ptr->roll_dps;
    telemetry_ptr->target_yaw_dps = target_ptr->yaw_dps;
    telemetry_ptr->tof_distance = range_ptr->range_cm;
    telemetry_ptr->target_altitude = target_ptr->altitude;
    telemetry_ptr->target_velocity_x_ms = target_ptr->velocity_x_ms;
    telemetry_ptr->target_velocity_y_ms = target_ptr->velocity_y_ms;
    telemetry_ptr->target_velocity_z_ms = target_ptr->velocity_z_ms;
    telemetry_ptr->flow_quality = flow_ptr->quality;
    telemetry_ptr->flow_x_velocity = flow_ptr->velocity_x_ms * 1000.0f;
    telemetry_ptr->flow_y_velocity = flow_ptr->velocity_y_ms * 1000.0f;
    telemetry_ptr->gps_fix = gnss_ptr->fix;
    telemetry_ptr->gps_satCount = gnss_ptr->satCount;
    telemetry_ptr->gps_latitude = gnss_ptr->latitude;
    telemetry_ptr->gps_longitude = gnss_ptr->longitude;
    telemetry_ptr->gps_altitude_m = gnss_ptr->altitude_mm;
    telemetry_ptr->gps_northVel_ms = gnss_ptr->northVel_mms;
    telemetry_ptr->gps_eastVel_ms = gnss_ptr->eastVel_mms;
    telemetry_ptr->gps_downVel_ms = gnss_ptr->downVel_mms;
    telemetry_ptr->gps_headingOfMotion = gnss_ptr->headingOfMotion;
    telemetry_ptr->gps_hdop = gnss_ptr->hdop;
    telemetry_ptr->gps_vdop = gnss_ptr->vdop;
    //telemetry_ptr->is_gnss_sanity_check_ok = gnss_sanity_check();
    telemetry_ptr->velocity_ms_2d = sqrtf((state_ptr->vel_forward_ms * state_ptr->vel_forward_ms) + (state_ptr->vel_right_ms * state_ptr->vel_right_ms));
    telemetry_ptr->throttle = target_ptr->throttle;
    telemetry_ptr->target_latitude = target_ptr->latitude;
    telemetry_ptr->target_longitude = target_ptr->longitude;

    // 0 full manual 1 altitude_hold 2 position_hold 3 alt+pos_hold 4 alt+pos+waypoint
    if (flight_ptr->waypoint_mission_status == 1)
    {
        telemetry_ptr->flight_mode = 4;
    }
    else if (flight_ptr->alt_hold_status == 1)
    {
        if (flight_ptr->pos_hold_status == 1)
            telemetry_ptr->flight_mode = 3;
        else
            telemetry_ptr->flight_mode = 1;
    }
    else if (flight_ptr->pos_hold_status == 1)
        telemetry_ptr->flight_mode = 2;
    else
        telemetry_ptr->flight_mode = 0;

    
    uint8_t buffer[sizeof(telemetry_t) + 1];
    buffer[0] = TELEM_HEADER;
    memcpy(buffer + 1, (uint8_t *)telemetry_ptr, sizeof(telemetry_t));
    ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, buffer, sizeof(buffer)));

    respond_to_requests();
}

static void respond_to_requests()
{
    if (recieved_command_flag == 3)
    {
        printf("Config requested\n");
        esp_now_send_config(config_ptr);
        recieved_command_flag = 0;
    }
    else if (recieved_command_flag == 4)
    {
        printf("Mission requested\n");
        esp_now_send_mission();
        recieved_command_flag = 0;
    }
}

void esp_now_send_config(config_t *conf)
{
    uint8_t buffer[sizeof(config_t) + 1];
    buffer[0] = CONF_HEADER;
    memcpy(buffer + 1, (uint8_t *)conf, sizeof(config_t));
    ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, buffer, sizeof(buffer)));
}

void esp_now_send_mission()
{
    uint8_t buffer[228];
    buffer[0] = WP_HEADER;
    buffer[1] = 1;
    memcpy(buffer + 2, waypoint_ptr->latitude, 100);
    memcpy(buffer + 2 + 100, waypoint_ptr->longitude, 100);
    memcpy(buffer + 2 + 100 + 100, waypoint_ptr->altitude, 25);
    buffer[227] = waypoint_ptr->end_of_mission_behaviour;

    ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, buffer, sizeof(buffer)));

    vTaskDelay(200);

    buffer[1] = 2;
    memcpy(buffer + 2, waypoint_ptr->latitude + 25, 100);
    memcpy(buffer + 2 + 100, waypoint_ptr->longitude + 25, 100);
    memcpy(buffer + 2 + 100 + 100, waypoint_ptr->altitude + 25, 25);
    buffer[227] = waypoint_ptr->end_of_mission_behaviour;

    ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, buffer, sizeof(buffer)));
}
void esp_now_send_motor_test_result(float *result)
{
    uint8_t buffer[sizeof(float) * 4 + 1];
    buffer[0] = MTR_TEST_HEADER;
    memcpy(buffer + 1, result, sizeof(float) * 4);
    ESP_ERROR_CHECK(esp_now_send(ground_station_mac_address, buffer, sizeof(buffer)));
}
static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    
    if (data[0] == 0xFF && len == sizeof(gamepad_t) + 1)
    {
        memcpy(gamepad_ptr, data + 1, sizeof(gamepad_t));
    } 
    
    if (data[0] == 0xFE && len == sizeof(config_t) + 1)
    {
        memcpy(config_ptr, data + 1, sizeof(config_t));
        if (storage_save(config_ptr, CONFIG_DATA)) printf("Configuration saved\n");
        else printf("ERROR: Configuration NOT saved!\n");
        recieved_command_flag = 1;
    }
    else if (data[0] == 0xFD && len == 228)
    {
        parse_mission_data(data, len);
    }
    else if (data[0] == 0xFC && len == 2)
    {
        if (data[1] == 10) recieved_command_flag = 3;
        else if (data[1] == 20) recieved_command_flag = 4;
    }
    else if (data[0] == 0xFB && len == 49)
    {
        recieved_command_flag = 5;
        mag_data = data;
    }
    else if (data[0] == 0xFA)
    {
        *motor_test_num_ptr = data[1];
    }
    else if (data[0] == 0xF9)
    {
        recieved_command_flag = 6;
        acc_data = data;
    }

}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

const uint8_t *get_mag_data()
{
    return mag_data;
}
const uint8_t *get_acc_data()
{
    return acc_data;
}


static void parse_mission_data(const uint8_t *data, uint8_t len)
{
    static uint8_t buff[228];
    static int64_t wp_data_recv_time_us;

    if (data[1] == 1) // first wp packet received
    {
        memcpy(&buff, data, len); // save it and wait for the rest
        wp_data_recv_time_us = esp_timer_get_time(); // reset packet timeout 
    }
    else if (data[1] == 2 && (esp_timer_get_time() - wp_data_recv_time_us) < 500000) // second packet received and timeout not triggered
    {
        // parse first (previous) packet
        memcpy(waypoint_ptr->latitude, buff + 2, 100);
        memcpy(waypoint_ptr->longitude, buff + 100 + 2, 100);
        memcpy(waypoint_ptr->altitude, buff + (100 * 2) + 2, 25);

        // parse second (current) packet
        memcpy(waypoint_ptr->latitude + 25, data + 2, 100);
        memcpy(waypoint_ptr->longitude + 25, data + 100 + 2, 100);
        memcpy(waypoint_ptr->altitude + 25, data + (100 * 2) + 2, 25);

        waypoint_ptr->end_of_mission_behaviour = data[227];
        waypoint_ptr->counter = 0;
        waypoint_ptr->is_reached = 1;
        recieved_command_flag = 2;

        storage_save(waypoint_ptr, MISSION_DATA);

        /*      
        for (int i = 0; i < 50; i++) 
        {
            printf("Waypoint %d: Enlem = %ld, Boylam = %ld, Yükseklik = %u\n", i, waypoint_ptr->latitude[i], waypoint_ptr->longitude[i], waypoint_ptr->altitude[i]);
        }  
        */
    }
}