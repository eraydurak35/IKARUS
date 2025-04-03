#pragma once
// MESSAGE RANGE_FINDER PACKING

#define MAVLINK_MSG_ID_RANGE_FINDER 124


typedef struct __mavlink_range_finder_t {
 int16_t range_cm; /*<  Distance in cm*/
} mavlink_range_finder_t;

#define MAVLINK_MSG_ID_RANGE_FINDER_LEN 2
#define MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN 2
#define MAVLINK_MSG_ID_124_LEN 2
#define MAVLINK_MSG_ID_124_MIN_LEN 2

#define MAVLINK_MSG_ID_RANGE_FINDER_CRC 152
#define MAVLINK_MSG_ID_124_CRC 152



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RANGE_FINDER { \
    124, \
    "RANGE_FINDER", \
    1, \
    {  { "range_cm", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_range_finder_t, range_cm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RANGE_FINDER { \
    "RANGE_FINDER", \
    1, \
    {  { "range_cm", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_range_finder_t, range_cm) }, \
         } \
}
#endif

/**
 * @brief Pack a range_finder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param range_cm  Distance in cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_range_finder_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t range_cm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGE_FINDER_LEN];
    _mav_put_int16_t(buf, 0, range_cm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#else
    mavlink_range_finder_t packet;
    packet.range_cm = range_cm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGE_FINDER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
}

/**
 * @brief Pack a range_finder message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param range_cm  Distance in cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_range_finder_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int16_t range_cm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGE_FINDER_LEN];
    _mav_put_int16_t(buf, 0, range_cm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#else
    mavlink_range_finder_t packet;
    packet.range_cm = range_cm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGE_FINDER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#endif
}

/**
 * @brief Pack a range_finder message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param range_cm  Distance in cm
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_range_finder_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t range_cm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGE_FINDER_LEN];
    _mav_put_int16_t(buf, 0, range_cm);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#else
    mavlink_range_finder_t packet;
    packet.range_cm = range_cm;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RANGE_FINDER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
}

/**
 * @brief Encode a range_finder struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param range_finder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_range_finder_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_range_finder_t* range_finder)
{
    return mavlink_msg_range_finder_pack(system_id, component_id, msg, range_finder->range_cm);
}

/**
 * @brief Encode a range_finder struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param range_finder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_range_finder_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_range_finder_t* range_finder)
{
    return mavlink_msg_range_finder_pack_chan(system_id, component_id, chan, msg, range_finder->range_cm);
}

/**
 * @brief Encode a range_finder struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param range_finder C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_range_finder_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_range_finder_t* range_finder)
{
    return mavlink_msg_range_finder_pack_status(system_id, component_id, _status, msg,  range_finder->range_cm);
}

/**
 * @brief Send a range_finder message
 * @param chan MAVLink channel to send the message
 *
 * @param range_cm  Distance in cm
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_range_finder_send(mavlink_channel_t chan, int16_t range_cm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RANGE_FINDER_LEN];
    _mav_put_int16_t(buf, 0, range_cm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGE_FINDER, buf, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
#else
    mavlink_range_finder_t packet;
    packet.range_cm = range_cm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGE_FINDER, (const char *)&packet, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
#endif
}

/**
 * @brief Send a range_finder message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_range_finder_send_struct(mavlink_channel_t chan, const mavlink_range_finder_t* range_finder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_range_finder_send(chan, range_finder->range_cm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGE_FINDER, (const char *)range_finder, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
#endif
}

#if MAVLINK_MSG_ID_RANGE_FINDER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_range_finder_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t range_cm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, range_cm);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGE_FINDER, buf, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
#else
    mavlink_range_finder_t *packet = (mavlink_range_finder_t *)msgbuf;
    packet->range_cm = range_cm;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RANGE_FINDER, (const char *)packet, MAVLINK_MSG_ID_RANGE_FINDER_MIN_LEN, MAVLINK_MSG_ID_RANGE_FINDER_LEN, MAVLINK_MSG_ID_RANGE_FINDER_CRC);
#endif
}
#endif

#endif

// MESSAGE RANGE_FINDER UNPACKING


/**
 * @brief Get field range_cm from range_finder message
 *
 * @return  Distance in cm
 */
static inline int16_t mavlink_msg_range_finder_get_range_cm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Decode a range_finder message into a struct
 *
 * @param msg The message to decode
 * @param range_finder C-struct to decode the message contents into
 */
static inline void mavlink_msg_range_finder_decode(const mavlink_message_t* msg, mavlink_range_finder_t* range_finder)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    range_finder->range_cm = mavlink_msg_range_finder_get_range_cm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RANGE_FINDER_LEN? msg->len : MAVLINK_MSG_ID_RANGE_FINDER_LEN;
        memset(range_finder, 0, MAVLINK_MSG_ID_RANGE_FINDER_LEN);
    memcpy(range_finder, _MAV_PAYLOAD(msg), len);
#endif
}
