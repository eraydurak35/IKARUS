#include <hitl.h>

static uint8_t buffer[32] = {0};
static mavlink_message_t msg;
static mavlink_status_t status;

static size_t read_count = 0;

static mavlink_hil_gps_t hil_gps_msg;
static mavlink_hil_sensor_t hil_sensor_msg;

static void mavlink_parse_msg(mavlink_message_t *msg);

uint8_t hitl_read()
{
    read_count = fread(buffer, 1, sizeof(buffer), stdin);
    if (read_count > 0)
    {
        for (int i = 0; i < read_count; i++) 
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) 
            {
                mavlink_parse_msg(&msg);
                return 1;
            }
        }
    }
    return 0;
}

void mavlink_send_actuators(float act1, float act2, float act3, float act4) {
    static mavlink_message_t hil_actuators_msg;
    static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    static float controls[16] = {0.0f};

    // Actuator verilerini ayarla (örneğin motor hızları)
    controls[0] = act1 / 1023.0f;
    controls[1] = act2 / 1023.0f;
    controls[2] = act3 / 1023.0f;
    controls[3] = act4 / 1023.0f;

    // Mesajı oluştur
    mavlink_msg_hil_actuator_controls_pack(
        1,              // system_id
        1,              // component_id
        &hil_actuators_msg, // mesaj
        0,      // zaman damgası
        controls,       // actuator verileri
        MAV_MODE_FLAG_HIL_ENABLED, // mode
        0               // flags
    );

    // Mesajı seri hale getir (buffer'a yaz)
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &hil_actuators_msg);

    // UART veya seri port üzerinden gönder (örnek fwrite ile)
    fwrite(buffer, 1, len, stdout);
}

void mavlink_send_heartbeat()
{
    static mavlink_message_t heartbeat_msg;
    static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(1, 1, &heartbeat_msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
    size_t len = mavlink_msg_to_send_buffer(buffer, &heartbeat_msg);

    fwrite(buffer, 1, len, stdout);
}

static void mavlink_parse_msg(mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_HIL_SENSOR:
    {
        mavlink_msg_hil_sensor_decode(msg, &hil_sensor_msg);
        //printf("esp32_sensor_aldi");
        break;
    }
    case MAVLINK_MSG_ID_HIL_GPS:
    {
        //mavlink_msg_hil_gps_decode(msg, &hil_gps_msg);
        //printf("esp32_gps_aldi");
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        //printf("esp32_hartbeat_aldi\n");
        break;
    }
    default:
        //printf("esp32_bilmiyor\n");
        break;
    }
}

void hitl_get_sensors(imu_t *imu, bmp390_t *baro)
{
    imu->gyro_dps[X] = hil_sensor_msg.xgyro * RAD_TO_DEG;
    imu->gyro_dps[Y] = hil_sensor_msg.ygyro * RAD_TO_DEG;
    imu->gyro_dps[Z] = hil_sensor_msg.zgyro * RAD_TO_DEG;

    imu->accel_ms2[X] = -hil_sensor_msg.xacc;
    imu->accel_ms2[Y] = -hil_sensor_msg.yacc;
    imu->accel_ms2[Z] = -hil_sensor_msg.zacc;

    baro->press = hil_sensor_msg.abs_pressure;
    baro->gnd_press = 1013.25f;
}