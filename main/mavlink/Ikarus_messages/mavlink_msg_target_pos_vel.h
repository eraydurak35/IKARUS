#pragma once
// MESSAGE TARGET_POS_VEL PACKING

#define MAVLINK_MSG_ID_TARGET_POS_VEL 121


typedef struct __mavlink_target_pos_vel_t {
 int32_t latitude; /*<  Target latitude * e-7*/
 int32_t longitude; /*<  Target longitude  * e-7*/
 float altitude; /*<  Target altitude in meters*/
 float vel_x_ms; /*<  Target x velocity in ms*/
 float vel_y_ms; /*<  Target y velocity in ms*/
 float vel_z_ms; /*<  Target z velocity in ms*/
 uint8_t throttle; /*<  Throttle percent*/
} mavlink_target_pos_vel_t;

#define MAVLINK_MSG_ID_TARGET_POS_VEL_LEN 25
#define MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN 25
#define MAVLINK_MSG_ID_121_LEN 25
#define MAVLINK_MSG_ID_121_MIN_LEN 25

#define MAVLINK_MSG_ID_TARGET_POS_VEL_CRC 213
#define MAVLINK_MSG_ID_121_CRC 213



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_POS_VEL { \
    121, \
    "TARGET_POS_VEL", \
    7, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_target_pos_vel_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_target_pos_vel_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_pos_vel_t, altitude) }, \
         { "vel_x_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_pos_vel_t, vel_x_ms) }, \
         { "vel_y_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_pos_vel_t, vel_y_ms) }, \
         { "vel_z_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_pos_vel_t, vel_z_ms) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_target_pos_vel_t, throttle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_POS_VEL { \
    "TARGET_POS_VEL", \
    7, \
    {  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_target_pos_vel_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_target_pos_vel_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_pos_vel_t, altitude) }, \
         { "vel_x_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_pos_vel_t, vel_x_ms) }, \
         { "vel_y_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_pos_vel_t, vel_y_ms) }, \
         { "vel_z_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_pos_vel_t, vel_z_ms) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_target_pos_vel_t, throttle) }, \
         } \
}
#endif

/**
 * @brief Pack a target_pos_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  Target latitude * e-7
 * @param longitude  Target longitude  * e-7
 * @param altitude  Target altitude in meters
 * @param vel_x_ms  Target x velocity in ms
 * @param vel_y_ms  Target y velocity in ms
 * @param vel_z_ms  Target z velocity in ms
 * @param throttle  Throttle percent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pos_vel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, float altitude, float vel_x_ms, float vel_y_ms, float vel_z_ms, uint8_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POS_VEL_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, vel_x_ms);
    _mav_put_float(buf, 16, vel_y_ms);
    _mav_put_float(buf, 20, vel_z_ms);
    _mav_put_uint8_t(buf, 24, throttle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#else
    mavlink_target_pos_vel_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.vel_z_ms = vel_z_ms;
    packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POS_VEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
}

/**
 * @brief Pack a target_pos_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude  Target latitude * e-7
 * @param longitude  Target longitude  * e-7
 * @param altitude  Target altitude in meters
 * @param vel_x_ms  Target x velocity in ms
 * @param vel_y_ms  Target y velocity in ms
 * @param vel_z_ms  Target z velocity in ms
 * @param throttle  Throttle percent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pos_vel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int32_t latitude, int32_t longitude, float altitude, float vel_x_ms, float vel_y_ms, float vel_z_ms, uint8_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POS_VEL_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, vel_x_ms);
    _mav_put_float(buf, 16, vel_y_ms);
    _mav_put_float(buf, 20, vel_z_ms);
    _mav_put_uint8_t(buf, 24, throttle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#else
    mavlink_target_pos_vel_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.vel_z_ms = vel_z_ms;
    packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POS_VEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#endif
}

/**
 * @brief Pack a target_pos_vel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude  Target latitude * e-7
 * @param longitude  Target longitude  * e-7
 * @param altitude  Target altitude in meters
 * @param vel_x_ms  Target x velocity in ms
 * @param vel_y_ms  Target y velocity in ms
 * @param vel_z_ms  Target z velocity in ms
 * @param throttle  Throttle percent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_pos_vel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t latitude,int32_t longitude,float altitude,float vel_x_ms,float vel_y_ms,float vel_z_ms,uint8_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POS_VEL_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, vel_x_ms);
    _mav_put_float(buf, 16, vel_y_ms);
    _mav_put_float(buf, 20, vel_z_ms);
    _mav_put_uint8_t(buf, 24, throttle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#else
    mavlink_target_pos_vel_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.vel_z_ms = vel_z_ms;
    packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POS_VEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
}

/**
 * @brief Encode a target_pos_vel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_pos_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_pos_vel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_pos_vel_t* target_pos_vel)
{
    return mavlink_msg_target_pos_vel_pack(system_id, component_id, msg, target_pos_vel->latitude, target_pos_vel->longitude, target_pos_vel->altitude, target_pos_vel->vel_x_ms, target_pos_vel->vel_y_ms, target_pos_vel->vel_z_ms, target_pos_vel->throttle);
}

/**
 * @brief Encode a target_pos_vel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_pos_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_pos_vel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_pos_vel_t* target_pos_vel)
{
    return mavlink_msg_target_pos_vel_pack_chan(system_id, component_id, chan, msg, target_pos_vel->latitude, target_pos_vel->longitude, target_pos_vel->altitude, target_pos_vel->vel_x_ms, target_pos_vel->vel_y_ms, target_pos_vel->vel_z_ms, target_pos_vel->throttle);
}

/**
 * @brief Encode a target_pos_vel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param target_pos_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_pos_vel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_target_pos_vel_t* target_pos_vel)
{
    return mavlink_msg_target_pos_vel_pack_status(system_id, component_id, _status, msg,  target_pos_vel->latitude, target_pos_vel->longitude, target_pos_vel->altitude, target_pos_vel->vel_x_ms, target_pos_vel->vel_y_ms, target_pos_vel->vel_z_ms, target_pos_vel->throttle);
}

/**
 * @brief Send a target_pos_vel message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude  Target latitude * e-7
 * @param longitude  Target longitude  * e-7
 * @param altitude  Target altitude in meters
 * @param vel_x_ms  Target x velocity in ms
 * @param vel_y_ms  Target y velocity in ms
 * @param vel_z_ms  Target z velocity in ms
 * @param throttle  Throttle percent
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_pos_vel_send(mavlink_channel_t chan, int32_t latitude, int32_t longitude, float altitude, float vel_x_ms, float vel_y_ms, float vel_z_ms, uint8_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POS_VEL_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, vel_x_ms);
    _mav_put_float(buf, 16, vel_y_ms);
    _mav_put_float(buf, 20, vel_z_ms);
    _mav_put_uint8_t(buf, 24, throttle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POS_VEL, buf, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
#else
    mavlink_target_pos_vel_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.vel_z_ms = vel_z_ms;
    packet.throttle = throttle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POS_VEL, (const char *)&packet, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
#endif
}

/**
 * @brief Send a target_pos_vel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_pos_vel_send_struct(mavlink_channel_t chan, const mavlink_target_pos_vel_t* target_pos_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_pos_vel_send(chan, target_pos_vel->latitude, target_pos_vel->longitude, target_pos_vel->altitude, target_pos_vel->vel_x_ms, target_pos_vel->vel_y_ms, target_pos_vel->vel_z_ms, target_pos_vel->throttle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POS_VEL, (const char *)target_pos_vel, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_POS_VEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_pos_vel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t latitude, int32_t longitude, float altitude, float vel_x_ms, float vel_y_ms, float vel_z_ms, uint8_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_float(buf, 8, altitude);
    _mav_put_float(buf, 12, vel_x_ms);
    _mav_put_float(buf, 16, vel_y_ms);
    _mav_put_float(buf, 20, vel_z_ms);
    _mav_put_uint8_t(buf, 24, throttle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POS_VEL, buf, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
#else
    mavlink_target_pos_vel_t *packet = (mavlink_target_pos_vel_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude = altitude;
    packet->vel_x_ms = vel_x_ms;
    packet->vel_y_ms = vel_y_ms;
    packet->vel_z_ms = vel_z_ms;
    packet->throttle = throttle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POS_VEL, (const char *)packet, MAVLINK_MSG_ID_TARGET_POS_VEL_MIN_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN, MAVLINK_MSG_ID_TARGET_POS_VEL_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_POS_VEL UNPACKING


/**
 * @brief Get field latitude from target_pos_vel message
 *
 * @return  Target latitude * e-7
 */
static inline int32_t mavlink_msg_target_pos_vel_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from target_pos_vel message
 *
 * @return  Target longitude  * e-7
 */
static inline int32_t mavlink_msg_target_pos_vel_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from target_pos_vel message
 *
 * @return  Target altitude in meters
 */
static inline float mavlink_msg_target_pos_vel_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field vel_x_ms from target_pos_vel message
 *
 * @return  Target x velocity in ms
 */
static inline float mavlink_msg_target_pos_vel_get_vel_x_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vel_y_ms from target_pos_vel message
 *
 * @return  Target y velocity in ms
 */
static inline float mavlink_msg_target_pos_vel_get_vel_y_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_z_ms from target_pos_vel message
 *
 * @return  Target z velocity in ms
 */
static inline float mavlink_msg_target_pos_vel_get_vel_z_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field throttle from target_pos_vel message
 *
 * @return  Throttle percent
 */
static inline uint8_t mavlink_msg_target_pos_vel_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a target_pos_vel message into a struct
 *
 * @param msg The message to decode
 * @param target_pos_vel C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_pos_vel_decode(const mavlink_message_t* msg, mavlink_target_pos_vel_t* target_pos_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target_pos_vel->latitude = mavlink_msg_target_pos_vel_get_latitude(msg);
    target_pos_vel->longitude = mavlink_msg_target_pos_vel_get_longitude(msg);
    target_pos_vel->altitude = mavlink_msg_target_pos_vel_get_altitude(msg);
    target_pos_vel->vel_x_ms = mavlink_msg_target_pos_vel_get_vel_x_ms(msg);
    target_pos_vel->vel_y_ms = mavlink_msg_target_pos_vel_get_vel_y_ms(msg);
    target_pos_vel->vel_z_ms = mavlink_msg_target_pos_vel_get_vel_z_ms(msg);
    target_pos_vel->throttle = mavlink_msg_target_pos_vel_get_throttle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_POS_VEL_LEN? msg->len : MAVLINK_MSG_ID_TARGET_POS_VEL_LEN;
        memset(target_pos_vel, 0, MAVLINK_MSG_ID_TARGET_POS_VEL_LEN);
    memcpy(target_pos_vel, _MAV_PAYLOAD(msg), len);
#endif
}
