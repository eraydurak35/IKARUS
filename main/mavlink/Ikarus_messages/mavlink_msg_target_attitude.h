#pragma once
// MESSAGE TARGET_ATTITUDE PACKING

#define MAVLINK_MSG_ID_TARGET_ATTITUDE 118


typedef struct __mavlink_target_attitude_t {
 float pitch_degree; /*<  Target pitch degree*/
 float roll_degree; /*<  Target roll degree*/
 float heading_degree; /*<  Target heading degree*/
 float pitch_dps; /*<  Target pitch dps*/
 float roll_dps; /*<  Target roll dps*/
 float yaw_dps; /*<  Target yaw dps*/
} mavlink_target_attitude_t;

#define MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN 24
#define MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN 24
#define MAVLINK_MSG_ID_118_LEN 24
#define MAVLINK_MSG_ID_118_MIN_LEN 24

#define MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC 78
#define MAVLINK_MSG_ID_118_CRC 78



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_ATTITUDE { \
    118, \
    "TARGET_ATTITUDE", \
    6, \
    {  { "pitch_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_attitude_t, pitch_degree) }, \
         { "roll_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_attitude_t, roll_degree) }, \
         { "heading_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_attitude_t, heading_degree) }, \
         { "pitch_dps", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_attitude_t, pitch_dps) }, \
         { "roll_dps", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_attitude_t, roll_dps) }, \
         { "yaw_dps", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_attitude_t, yaw_dps) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_ATTITUDE { \
    "TARGET_ATTITUDE", \
    6, \
    {  { "pitch_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_target_attitude_t, pitch_degree) }, \
         { "roll_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_target_attitude_t, roll_degree) }, \
         { "heading_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_target_attitude_t, heading_degree) }, \
         { "pitch_dps", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_target_attitude_t, pitch_dps) }, \
         { "roll_dps", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_attitude_t, roll_dps) }, \
         { "yaw_dps", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_attitude_t, yaw_dps) }, \
         } \
}
#endif

/**
 * @brief Pack a target_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch_degree  Target pitch degree
 * @param roll_degree  Target roll degree
 * @param heading_degree  Target heading degree
 * @param pitch_dps  Target pitch dps
 * @param roll_dps  Target roll dps
 * @param yaw_dps  Target yaw dps
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pitch_degree, float roll_degree, float heading_degree, float pitch_dps, float roll_dps, float yaw_dps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);
    _mav_put_float(buf, 12, pitch_dps);
    _mav_put_float(buf, 16, roll_dps);
    _mav_put_float(buf, 20, yaw_dps);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#else
    mavlink_target_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;
    packet.pitch_dps = pitch_dps;
    packet.roll_dps = roll_dps;
    packet.yaw_dps = yaw_dps;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
}

/**
 * @brief Pack a target_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch_degree  Target pitch degree
 * @param roll_degree  Target roll degree
 * @param heading_degree  Target heading degree
 * @param pitch_dps  Target pitch dps
 * @param roll_dps  Target roll dps
 * @param yaw_dps  Target yaw dps
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_attitude_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float pitch_degree, float roll_degree, float heading_degree, float pitch_dps, float roll_dps, float yaw_dps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);
    _mav_put_float(buf, 12, pitch_dps);
    _mav_put_float(buf, 16, roll_dps);
    _mav_put_float(buf, 20, yaw_dps);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#else
    mavlink_target_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;
    packet.pitch_dps = pitch_dps;
    packet.roll_dps = roll_dps;
    packet.yaw_dps = yaw_dps;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a target_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitch_degree  Target pitch degree
 * @param roll_degree  Target roll degree
 * @param heading_degree  Target heading degree
 * @param pitch_dps  Target pitch dps
 * @param roll_dps  Target roll dps
 * @param yaw_dps  Target yaw dps
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pitch_degree,float roll_degree,float heading_degree,float pitch_dps,float roll_dps,float yaw_dps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);
    _mav_put_float(buf, 12, pitch_dps);
    _mav_put_float(buf, 16, roll_dps);
    _mav_put_float(buf, 20, yaw_dps);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#else
    mavlink_target_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;
    packet.pitch_dps = pitch_dps;
    packet.roll_dps = roll_dps;
    packet.yaw_dps = yaw_dps;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
}

/**
 * @brief Encode a target_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_attitude_t* target_attitude)
{
    return mavlink_msg_target_attitude_pack(system_id, component_id, msg, target_attitude->pitch_degree, target_attitude->roll_degree, target_attitude->heading_degree, target_attitude->pitch_dps, target_attitude->roll_dps, target_attitude->yaw_dps);
}

/**
 * @brief Encode a target_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_attitude_t* target_attitude)
{
    return mavlink_msg_target_attitude_pack_chan(system_id, component_id, chan, msg, target_attitude->pitch_degree, target_attitude->roll_degree, target_attitude->heading_degree, target_attitude->pitch_dps, target_attitude->roll_dps, target_attitude->yaw_dps);
}

/**
 * @brief Encode a target_attitude struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param target_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_attitude_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_target_attitude_t* target_attitude)
{
    return mavlink_msg_target_attitude_pack_status(system_id, component_id, _status, msg,  target_attitude->pitch_degree, target_attitude->roll_degree, target_attitude->heading_degree, target_attitude->pitch_dps, target_attitude->roll_dps, target_attitude->yaw_dps);
}

/**
 * @brief Send a target_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param pitch_degree  Target pitch degree
 * @param roll_degree  Target roll degree
 * @param heading_degree  Target heading degree
 * @param pitch_dps  Target pitch dps
 * @param roll_dps  Target roll dps
 * @param yaw_dps  Target yaw dps
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_attitude_send(mavlink_channel_t chan, float pitch_degree, float roll_degree, float heading_degree, float pitch_dps, float roll_dps, float yaw_dps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);
    _mav_put_float(buf, 12, pitch_dps);
    _mav_put_float(buf, 16, roll_dps);
    _mav_put_float(buf, 20, yaw_dps);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ATTITUDE, buf, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
#else
    mavlink_target_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;
    packet.pitch_dps = pitch_dps;
    packet.roll_dps = roll_dps;
    packet.yaw_dps = yaw_dps;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a target_attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_attitude_send_struct(mavlink_channel_t chan, const mavlink_target_attitude_t* target_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_attitude_send(chan, target_attitude->pitch_degree, target_attitude->roll_degree, target_attitude->heading_degree, target_attitude->pitch_dps, target_attitude->roll_dps, target_attitude->yaw_dps);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ATTITUDE, (const char *)target_attitude, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pitch_degree, float roll_degree, float heading_degree, float pitch_dps, float roll_dps, float yaw_dps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);
    _mav_put_float(buf, 12, pitch_dps);
    _mav_put_float(buf, 16, roll_dps);
    _mav_put_float(buf, 20, yaw_dps);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ATTITUDE, buf, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
#else
    mavlink_target_attitude_t *packet = (mavlink_target_attitude_t *)msgbuf;
    packet->pitch_degree = pitch_degree;
    packet->roll_degree = roll_degree;
    packet->heading_degree = heading_degree;
    packet->pitch_dps = pitch_dps;
    packet->roll_dps = roll_dps;
    packet->yaw_dps = yaw_dps;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_TARGET_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN, MAVLINK_MSG_ID_TARGET_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_ATTITUDE UNPACKING


/**
 * @brief Get field pitch_degree from target_attitude message
 *
 * @return  Target pitch degree
 */
static inline float mavlink_msg_target_attitude_get_pitch_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field roll_degree from target_attitude message
 *
 * @return  Target roll degree
 */
static inline float mavlink_msg_target_attitude_get_roll_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field heading_degree from target_attitude message
 *
 * @return  Target heading degree
 */
static inline float mavlink_msg_target_attitude_get_heading_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch_dps from target_attitude message
 *
 * @return  Target pitch dps
 */
static inline float mavlink_msg_target_attitude_get_pitch_dps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll_dps from target_attitude message
 *
 * @return  Target roll dps
 */
static inline float mavlink_msg_target_attitude_get_roll_dps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw_dps from target_attitude message
 *
 * @return  Target yaw dps
 */
static inline float mavlink_msg_target_attitude_get_yaw_dps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a target_attitude message into a struct
 *
 * @param msg The message to decode
 * @param target_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_attitude_decode(const mavlink_message_t* msg, mavlink_target_attitude_t* target_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target_attitude->pitch_degree = mavlink_msg_target_attitude_get_pitch_degree(msg);
    target_attitude->roll_degree = mavlink_msg_target_attitude_get_roll_degree(msg);
    target_attitude->heading_degree = mavlink_msg_target_attitude_get_heading_degree(msg);
    target_attitude->pitch_dps = mavlink_msg_target_attitude_get_pitch_dps(msg);
    target_attitude->roll_dps = mavlink_msg_target_attitude_get_roll_dps(msg);
    target_attitude->yaw_dps = mavlink_msg_target_attitude_get_yaw_dps(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN;
        memset(target_attitude, 0, MAVLINK_MSG_ID_TARGET_ATTITUDE_LEN);
    memcpy(target_attitude, _MAV_PAYLOAD(msg), len);
#endif
}
