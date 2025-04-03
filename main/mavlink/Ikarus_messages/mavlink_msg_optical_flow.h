#pragma once
// MESSAGE OPTICAL_FLOW PACKING

#define MAVLINK_MSG_ID_OPTICAL_FLOW 123


typedef struct __mavlink_optical_flow_t {
 float vel_x_ms; /*<  X velocity in ms*/
 float vel_y_ms; /*<  Y velocity in ms*/
 uint8_t quality; /*<  flow quality percent*/
} mavlink_optical_flow_t;

#define MAVLINK_MSG_ID_OPTICAL_FLOW_LEN 9
#define MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN 9
#define MAVLINK_MSG_ID_123_LEN 9
#define MAVLINK_MSG_ID_123_MIN_LEN 9

#define MAVLINK_MSG_ID_OPTICAL_FLOW_CRC 52
#define MAVLINK_MSG_ID_123_CRC 52



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OPTICAL_FLOW { \
    123, \
    "OPTICAL_FLOW", \
    3, \
    {  { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_optical_flow_t, quality) }, \
         { "vel_x_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_optical_flow_t, vel_x_ms) }, \
         { "vel_y_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_optical_flow_t, vel_y_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OPTICAL_FLOW { \
    "OPTICAL_FLOW", \
    3, \
    {  { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_optical_flow_t, quality) }, \
         { "vel_x_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_optical_flow_t, vel_x_ms) }, \
         { "vel_y_ms", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_optical_flow_t, vel_y_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a optical_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param quality  flow quality percent
 * @param vel_x_ms  X velocity in ms
 * @param vel_y_ms  Y velocity in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_optical_flow_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t quality, float vel_x_ms, float vel_y_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
    _mav_put_float(buf, 0, vel_x_ms);
    _mav_put_float(buf, 4, vel_y_ms);
    _mav_put_uint8_t(buf, 8, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#else
    mavlink_optical_flow_t packet;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
}

/**
 * @brief Pack a optical_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param quality  flow quality percent
 * @param vel_x_ms  X velocity in ms
 * @param vel_y_ms  Y velocity in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_optical_flow_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t quality, float vel_x_ms, float vel_y_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
    _mav_put_float(buf, 0, vel_x_ms);
    _mav_put_float(buf, 4, vel_y_ms);
    _mav_put_uint8_t(buf, 8, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#else
    mavlink_optical_flow_t packet;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif
}

/**
 * @brief Pack a optical_flow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param quality  flow quality percent
 * @param vel_x_ms  X velocity in ms
 * @param vel_y_ms  Y velocity in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_optical_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t quality,float vel_x_ms,float vel_y_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
    _mav_put_float(buf, 0, vel_x_ms);
    _mav_put_float(buf, 4, vel_y_ms);
    _mav_put_uint8_t(buf, 8, quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#else
    mavlink_optical_flow_t packet;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.quality = quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
}

/**
 * @brief Encode a optical_flow struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow)
{
    return mavlink_msg_optical_flow_pack(system_id, component_id, msg, optical_flow->quality, optical_flow->vel_x_ms, optical_flow->vel_y_ms);
}

/**
 * @brief Encode a optical_flow struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow)
{
    return mavlink_msg_optical_flow_pack_chan(system_id, component_id, chan, msg, optical_flow->quality, optical_flow->vel_x_ms, optical_flow->vel_y_ms);
}

/**
 * @brief Encode a optical_flow struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_optical_flow_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow)
{
    return mavlink_msg_optical_flow_pack_status(system_id, component_id, _status, msg,  optical_flow->quality, optical_flow->vel_x_ms, optical_flow->vel_y_ms);
}

/**
 * @brief Send a optical_flow message
 * @param chan MAVLink channel to send the message
 *
 * @param quality  flow quality percent
 * @param vel_x_ms  X velocity in ms
 * @param vel_y_ms  Y velocity in ms
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_optical_flow_send(mavlink_channel_t chan, uint8_t quality, float vel_x_ms, float vel_y_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OPTICAL_FLOW_LEN];
    _mav_put_float(buf, 0, vel_x_ms);
    _mav_put_float(buf, 4, vel_y_ms);
    _mav_put_uint8_t(buf, 8, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    mavlink_optical_flow_t packet;
    packet.vel_x_ms = vel_x_ms;
    packet.vel_y_ms = vel_y_ms;
    packet.quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)&packet, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#endif
}

/**
 * @brief Send a optical_flow message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_optical_flow_send_struct(mavlink_channel_t chan, const mavlink_optical_flow_t* optical_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_optical_flow_send(chan, optical_flow->quality, optical_flow->vel_x_ms, optical_flow->vel_y_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)optical_flow, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#endif
}

#if MAVLINK_MSG_ID_OPTICAL_FLOW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_optical_flow_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t quality, float vel_x_ms, float vel_y_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vel_x_ms);
    _mav_put_float(buf, 4, vel_y_ms);
    _mav_put_uint8_t(buf, 8, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#else
    mavlink_optical_flow_t *packet = (mavlink_optical_flow_t *)msgbuf;
    packet->vel_x_ms = vel_x_ms;
    packet->vel_y_ms = vel_y_ms;
    packet->quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)packet, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, MAVLINK_MSG_ID_OPTICAL_FLOW_CRC);
#endif
}
#endif

#endif

// MESSAGE OPTICAL_FLOW UNPACKING


/**
 * @brief Get field quality from optical_flow message
 *
 * @return  flow quality percent
 */
static inline uint8_t mavlink_msg_optical_flow_get_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field vel_x_ms from optical_flow message
 *
 * @return  X velocity in ms
 */
static inline float mavlink_msg_optical_flow_get_vel_x_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vel_y_ms from optical_flow message
 *
 * @return  Y velocity in ms
 */
static inline float mavlink_msg_optical_flow_get_vel_y_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a optical_flow message into a struct
 *
 * @param msg The message to decode
 * @param optical_flow C-struct to decode the message contents into
 */
static inline void mavlink_msg_optical_flow_decode(const mavlink_message_t* msg, mavlink_optical_flow_t* optical_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    optical_flow->vel_x_ms = mavlink_msg_optical_flow_get_vel_x_ms(msg);
    optical_flow->vel_y_ms = mavlink_msg_optical_flow_get_vel_y_ms(msg);
    optical_flow->quality = mavlink_msg_optical_flow_get_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OPTICAL_FLOW_LEN? msg->len : MAVLINK_MSG_ID_OPTICAL_FLOW_LEN;
        memset(optical_flow, 0, MAVLINK_MSG_ID_OPTICAL_FLOW_LEN);
    memcpy(optical_flow, _MAV_PAYLOAD(msg), len);
#endif
}
