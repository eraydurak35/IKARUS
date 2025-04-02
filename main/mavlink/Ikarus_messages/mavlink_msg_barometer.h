#pragma once
// MESSAGE BAROMETER PACKING

#define MAVLINK_MSG_ID_BAROMETER 120


typedef struct __mavlink_barometer_t {
 uint16_t pressure_pascal; /*<  Barometer pressure in pascal * 10*/
 int16_t temperature_c; /*<  Barometer temperature in c * 100*/
 int16_t altitude_m; /*<  Barometer altitude * 100*/
} mavlink_barometer_t;

#define MAVLINK_MSG_ID_BAROMETER_LEN 6
#define MAVLINK_MSG_ID_BAROMETER_MIN_LEN 6
#define MAVLINK_MSG_ID_120_LEN 6
#define MAVLINK_MSG_ID_120_MIN_LEN 6

#define MAVLINK_MSG_ID_BAROMETER_CRC 10
#define MAVLINK_MSG_ID_120_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BAROMETER { \
    120, \
    "BAROMETER", \
    3, \
    {  { "pressure_pascal", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_barometer_t, pressure_pascal) }, \
         { "temperature_c", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_barometer_t, temperature_c) }, \
         { "altitude_m", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_barometer_t, altitude_m) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BAROMETER { \
    "BAROMETER", \
    3, \
    {  { "pressure_pascal", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_barometer_t, pressure_pascal) }, \
         { "temperature_c", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_barometer_t, temperature_c) }, \
         { "altitude_m", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_barometer_t, altitude_m) }, \
         } \
}
#endif

/**
 * @brief Pack a barometer message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pressure_pascal  Barometer pressure in pascal * 10
 * @param temperature_c  Barometer temperature in c * 100
 * @param altitude_m  Barometer altitude * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_barometer_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t pressure_pascal, int16_t temperature_c, int16_t altitude_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAROMETER_LEN];
    _mav_put_uint16_t(buf, 0, pressure_pascal);
    _mav_put_int16_t(buf, 2, temperature_c);
    _mav_put_int16_t(buf, 4, altitude_m);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAROMETER_LEN);
#else
    mavlink_barometer_t packet;
    packet.pressure_pascal = pressure_pascal;
    packet.temperature_c = temperature_c;
    packet.altitude_m = altitude_m;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAROMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAROMETER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
}

/**
 * @brief Pack a barometer message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pressure_pascal  Barometer pressure in pascal * 10
 * @param temperature_c  Barometer temperature in c * 100
 * @param altitude_m  Barometer altitude * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_barometer_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t pressure_pascal, int16_t temperature_c, int16_t altitude_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAROMETER_LEN];
    _mav_put_uint16_t(buf, 0, pressure_pascal);
    _mav_put_int16_t(buf, 2, temperature_c);
    _mav_put_int16_t(buf, 4, altitude_m);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAROMETER_LEN);
#else
    mavlink_barometer_t packet;
    packet.pressure_pascal = pressure_pascal;
    packet.temperature_c = temperature_c;
    packet.altitude_m = altitude_m;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAROMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAROMETER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN);
#endif
}

/**
 * @brief Pack a barometer message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure_pascal  Barometer pressure in pascal * 10
 * @param temperature_c  Barometer temperature in c * 100
 * @param altitude_m  Barometer altitude * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_barometer_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t pressure_pascal,int16_t temperature_c,int16_t altitude_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAROMETER_LEN];
    _mav_put_uint16_t(buf, 0, pressure_pascal);
    _mav_put_int16_t(buf, 2, temperature_c);
    _mav_put_int16_t(buf, 4, altitude_m);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BAROMETER_LEN);
#else
    mavlink_barometer_t packet;
    packet.pressure_pascal = pressure_pascal;
    packet.temperature_c = temperature_c;
    packet.altitude_m = altitude_m;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BAROMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BAROMETER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
}

/**
 * @brief Encode a barometer struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param barometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_barometer_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_barometer_t* barometer)
{
    return mavlink_msg_barometer_pack(system_id, component_id, msg, barometer->pressure_pascal, barometer->temperature_c, barometer->altitude_m);
}

/**
 * @brief Encode a barometer struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param barometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_barometer_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_barometer_t* barometer)
{
    return mavlink_msg_barometer_pack_chan(system_id, component_id, chan, msg, barometer->pressure_pascal, barometer->temperature_c, barometer->altitude_m);
}

/**
 * @brief Encode a barometer struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param barometer C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_barometer_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_barometer_t* barometer)
{
    return mavlink_msg_barometer_pack_status(system_id, component_id, _status, msg,  barometer->pressure_pascal, barometer->temperature_c, barometer->altitude_m);
}

/**
 * @brief Send a barometer message
 * @param chan MAVLink channel to send the message
 *
 * @param pressure_pascal  Barometer pressure in pascal * 10
 * @param temperature_c  Barometer temperature in c * 100
 * @param altitude_m  Barometer altitude * 100
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_barometer_send(mavlink_channel_t chan, uint16_t pressure_pascal, int16_t temperature_c, int16_t altitude_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BAROMETER_LEN];
    _mav_put_uint16_t(buf, 0, pressure_pascal);
    _mav_put_int16_t(buf, 2, temperature_c);
    _mav_put_int16_t(buf, 4, altitude_m);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER, buf, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
#else
    mavlink_barometer_t packet;
    packet.pressure_pascal = pressure_pascal;
    packet.temperature_c = temperature_c;
    packet.altitude_m = altitude_m;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER, (const char *)&packet, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
#endif
}

/**
 * @brief Send a barometer message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_barometer_send_struct(mavlink_channel_t chan, const mavlink_barometer_t* barometer)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_barometer_send(chan, barometer->pressure_pascal, barometer->temperature_c, barometer->altitude_m);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER, (const char *)barometer, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
#endif
}

#if MAVLINK_MSG_ID_BAROMETER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_barometer_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t pressure_pascal, int16_t temperature_c, int16_t altitude_m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, pressure_pascal);
    _mav_put_int16_t(buf, 2, temperature_c);
    _mav_put_int16_t(buf, 4, altitude_m);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER, buf, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
#else
    mavlink_barometer_t *packet = (mavlink_barometer_t *)msgbuf;
    packet->pressure_pascal = pressure_pascal;
    packet->temperature_c = temperature_c;
    packet->altitude_m = altitude_m;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BAROMETER, (const char *)packet, MAVLINK_MSG_ID_BAROMETER_MIN_LEN, MAVLINK_MSG_ID_BAROMETER_LEN, MAVLINK_MSG_ID_BAROMETER_CRC);
#endif
}
#endif

#endif

// MESSAGE BAROMETER UNPACKING


/**
 * @brief Get field pressure_pascal from barometer message
 *
 * @return  Barometer pressure in pascal * 10
 */
static inline uint16_t mavlink_msg_barometer_get_pressure_pascal(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field temperature_c from barometer message
 *
 * @return  Barometer temperature in c * 100
 */
static inline int16_t mavlink_msg_barometer_get_temperature_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field altitude_m from barometer message
 *
 * @return  Barometer altitude * 100
 */
static inline int16_t mavlink_msg_barometer_get_altitude_m(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a barometer message into a struct
 *
 * @param msg The message to decode
 * @param barometer C-struct to decode the message contents into
 */
static inline void mavlink_msg_barometer_decode(const mavlink_message_t* msg, mavlink_barometer_t* barometer)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    barometer->pressure_pascal = mavlink_msg_barometer_get_pressure_pascal(msg);
    barometer->temperature_c = mavlink_msg_barometer_get_temperature_c(msg);
    barometer->altitude_m = mavlink_msg_barometer_get_altitude_m(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BAROMETER_LEN? msg->len : MAVLINK_MSG_ID_BAROMETER_LEN;
        memset(barometer, 0, MAVLINK_MSG_ID_BAROMETER_LEN);
    memcpy(barometer, _MAV_PAYLOAD(msg), len);
#endif
}
