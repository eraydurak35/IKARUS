#pragma once
// MESSAGE ATTITUDE PACKING

#define MAVLINK_MSG_ID_ATTITUDE 117


typedef struct __mavlink_attitude_t {
 float pitch_degree; /*<  Pitch degree*/
 float roll_degree; /*<  Roll degree*/
 float heading_degree; /*<  Heading degree*/
} mavlink_attitude_t;

#define MAVLINK_MSG_ID_ATTITUDE_LEN 12
#define MAVLINK_MSG_ID_ATTITUDE_MIN_LEN 12
#define MAVLINK_MSG_ID_117_LEN 12
#define MAVLINK_MSG_ID_117_MIN_LEN 12

#define MAVLINK_MSG_ID_ATTITUDE_CRC 17
#define MAVLINK_MSG_ID_117_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
    117, \
    "ATTITUDE", \
    3, \
    {  { "pitch_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_attitude_t, pitch_degree) }, \
         { "roll_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_t, roll_degree) }, \
         { "heading_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_t, heading_degree) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
    "ATTITUDE", \
    3, \
    {  { "pitch_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_attitude_t, pitch_degree) }, \
         { "roll_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_t, roll_degree) }, \
         { "heading_degree", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_t, heading_degree) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch_degree  Pitch degree
 * @param roll_degree  Roll degree
 * @param heading_degree  Heading degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pitch_degree, float roll_degree, float heading_degree)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitch_degree  Pitch degree
 * @param roll_degree  Roll degree
 * @param heading_degree  Heading degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float pitch_degree, float roll_degree, float heading_degree)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitch_degree  Pitch degree
 * @param roll_degree  Roll degree
 * @param heading_degree  Heading degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pitch_degree,float roll_degree,float heading_degree)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Encode a attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->pitch_degree, attitude->roll_degree, attitude->heading_degree);
}

/**
 * @brief Encode a attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack_chan(system_id, component_id, chan, msg, attitude->pitch_degree, attitude->roll_degree, attitude->heading_degree);
}

/**
 * @brief Encode a attitude struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack_status(system_id, component_id, _status, msg,  attitude->pitch_degree, attitude->roll_degree, attitude->heading_degree);
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param pitch_degree  Pitch degree
 * @param roll_degree  Roll degree
 * @param heading_degree  Heading degree
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, float pitch_degree, float roll_degree, float heading_degree)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    mavlink_attitude_t packet;
    packet.pitch_degree = pitch_degree;
    packet.roll_degree = roll_degree;
    packet.heading_degree = heading_degree;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_send_struct(mavlink_channel_t chan, const mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_send(chan, attitude->pitch_degree, attitude->roll_degree, attitude->heading_degree);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)attitude, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pitch_degree, float roll_degree, float heading_degree)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pitch_degree);
    _mav_put_float(buf, 4, roll_degree);
    _mav_put_float(buf, 8, heading_degree);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    mavlink_attitude_t *packet = (mavlink_attitude_t *)msgbuf;
    packet->pitch_degree = pitch_degree;
    packet->roll_degree = roll_degree;
    packet->heading_degree = heading_degree;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field pitch_degree from attitude message
 *
 * @return  Pitch degree
 */
static inline float mavlink_msg_attitude_get_pitch_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field roll_degree from attitude message
 *
 * @return  Roll degree
 */
static inline float mavlink_msg_attitude_get_roll_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field heading_degree from attitude message
 *
 * @return  Heading degree
 */
static inline float mavlink_msg_attitude_get_heading_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude->pitch_degree = mavlink_msg_attitude_get_pitch_degree(msg);
    attitude->roll_degree = mavlink_msg_attitude_get_roll_degree(msg);
    attitude->heading_degree = mavlink_msg_attitude_get_heading_degree(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_LEN;
        memset(attitude, 0, MAVLINK_MSG_ID_ATTITUDE_LEN);
    memcpy(attitude, _MAV_PAYLOAD(msg), len);
#endif
}
