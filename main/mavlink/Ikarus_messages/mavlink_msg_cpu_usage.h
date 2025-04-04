#pragma once
// MESSAGE CPU_USAGE PACKING

#define MAVLINK_MSG_ID_CPU_USAGE 125


typedef struct __mavlink_cpu_usage_t {
 uint8_t core0_percent; /*<  Core 0 usage in percent*/
 uint8_t core1_percent; /*<  Core 1 usage in percent*/
} mavlink_cpu_usage_t;

#define MAVLINK_MSG_ID_CPU_USAGE_LEN 2
#define MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN 2
#define MAVLINK_MSG_ID_125_LEN 2
#define MAVLINK_MSG_ID_125_MIN_LEN 2

#define MAVLINK_MSG_ID_CPU_USAGE_CRC 142
#define MAVLINK_MSG_ID_125_CRC 142



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CPU_USAGE { \
    125, \
    "CPU_USAGE", \
    2, \
    {  { "core0_percent", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_cpu_usage_t, core0_percent) }, \
         { "core1_percent", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_cpu_usage_t, core1_percent) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CPU_USAGE { \
    "CPU_USAGE", \
    2, \
    {  { "core0_percent", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_cpu_usage_t, core0_percent) }, \
         { "core1_percent", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_cpu_usage_t, core1_percent) }, \
         } \
}
#endif

/**
 * @brief Pack a cpu_usage message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param core0_percent  Core 0 usage in percent
 * @param core1_percent  Core 1 usage in percent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_usage_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t core0_percent, uint8_t core1_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_USAGE_LEN];
    _mav_put_uint8_t(buf, 0, core0_percent);
    _mav_put_uint8_t(buf, 1, core1_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#else
    mavlink_cpu_usage_t packet;
    packet.core0_percent = core0_percent;
    packet.core1_percent = core1_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CPU_USAGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
}

/**
 * @brief Pack a cpu_usage message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param core0_percent  Core 0 usage in percent
 * @param core1_percent  Core 1 usage in percent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_usage_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t core0_percent, uint8_t core1_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_USAGE_LEN];
    _mav_put_uint8_t(buf, 0, core0_percent);
    _mav_put_uint8_t(buf, 1, core1_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#else
    mavlink_cpu_usage_t packet;
    packet.core0_percent = core0_percent;
    packet.core1_percent = core1_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CPU_USAGE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#endif
}

/**
 * @brief Pack a cpu_usage message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param core0_percent  Core 0 usage in percent
 * @param core1_percent  Core 1 usage in percent
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_usage_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t core0_percent,uint8_t core1_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_USAGE_LEN];
    _mav_put_uint8_t(buf, 0, core0_percent);
    _mav_put_uint8_t(buf, 1, core1_percent);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#else
    mavlink_cpu_usage_t packet;
    packet.core0_percent = core0_percent;
    packet.core1_percent = core1_percent;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPU_USAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CPU_USAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
}

/**
 * @brief Encode a cpu_usage struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cpu_usage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpu_usage_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cpu_usage_t* cpu_usage)
{
    return mavlink_msg_cpu_usage_pack(system_id, component_id, msg, cpu_usage->core0_percent, cpu_usage->core1_percent);
}

/**
 * @brief Encode a cpu_usage struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cpu_usage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpu_usage_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cpu_usage_t* cpu_usage)
{
    return mavlink_msg_cpu_usage_pack_chan(system_id, component_id, chan, msg, cpu_usage->core0_percent, cpu_usage->core1_percent);
}

/**
 * @brief Encode a cpu_usage struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param cpu_usage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpu_usage_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_cpu_usage_t* cpu_usage)
{
    return mavlink_msg_cpu_usage_pack_status(system_id, component_id, _status, msg,  cpu_usage->core0_percent, cpu_usage->core1_percent);
}

/**
 * @brief Send a cpu_usage message
 * @param chan MAVLink channel to send the message
 *
 * @param core0_percent  Core 0 usage in percent
 * @param core1_percent  Core 1 usage in percent
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cpu_usage_send(mavlink_channel_t chan, uint8_t core0_percent, uint8_t core1_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_USAGE_LEN];
    _mav_put_uint8_t(buf, 0, core0_percent);
    _mav_put_uint8_t(buf, 1, core1_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_USAGE, buf, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
#else
    mavlink_cpu_usage_t packet;
    packet.core0_percent = core0_percent;
    packet.core1_percent = core1_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_USAGE, (const char *)&packet, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
#endif
}

/**
 * @brief Send a cpu_usage message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cpu_usage_send_struct(mavlink_channel_t chan, const mavlink_cpu_usage_t* cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cpu_usage_send(chan, cpu_usage->core0_percent, cpu_usage->core1_percent);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_USAGE, (const char *)cpu_usage, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CPU_USAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cpu_usage_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t core0_percent, uint8_t core1_percent)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, core0_percent);
    _mav_put_uint8_t(buf, 1, core1_percent);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_USAGE, buf, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
#else
    mavlink_cpu_usage_t *packet = (mavlink_cpu_usage_t *)msgbuf;
    packet->core0_percent = core0_percent;
    packet->core1_percent = core1_percent;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_USAGE, (const char *)packet, MAVLINK_MSG_ID_CPU_USAGE_MIN_LEN, MAVLINK_MSG_ID_CPU_USAGE_LEN, MAVLINK_MSG_ID_CPU_USAGE_CRC);
#endif
}
#endif

#endif

// MESSAGE CPU_USAGE UNPACKING


/**
 * @brief Get field core0_percent from cpu_usage message
 *
 * @return  Core 0 usage in percent
 */
static inline uint8_t mavlink_msg_cpu_usage_get_core0_percent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field core1_percent from cpu_usage message
 *
 * @return  Core 1 usage in percent
 */
static inline uint8_t mavlink_msg_cpu_usage_get_core1_percent(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a cpu_usage message into a struct
 *
 * @param msg The message to decode
 * @param cpu_usage C-struct to decode the message contents into
 */
static inline void mavlink_msg_cpu_usage_decode(const mavlink_message_t* msg, mavlink_cpu_usage_t* cpu_usage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cpu_usage->core0_percent = mavlink_msg_cpu_usage_get_core0_percent(msg);
    cpu_usage->core1_percent = mavlink_msg_cpu_usage_get_core1_percent(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CPU_USAGE_LEN? msg->len : MAVLINK_MSG_ID_CPU_USAGE_LEN;
        memset(cpu_usage, 0, MAVLINK_MSG_ID_CPU_USAGE_LEN);
    memcpy(cpu_usage, _MAV_PAYLOAD(msg), len);
#endif
}
