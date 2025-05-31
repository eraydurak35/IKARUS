#pragma once
// MESSAGE PARAMETER_SET PACKING

#define MAVLINK_MSG_ID_PARAMETER_SET 128


typedef struct __mavlink_parameter_set_t {
 float value; /*<  parameter value*/
 char param_name[16]; /*<  parameter name*/
} mavlink_parameter_set_t;

#define MAVLINK_MSG_ID_PARAMETER_SET_LEN 20
#define MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN 20
#define MAVLINK_MSG_ID_128_LEN 20
#define MAVLINK_MSG_ID_128_MIN_LEN 20

#define MAVLINK_MSG_ID_PARAMETER_SET_CRC 129
#define MAVLINK_MSG_ID_128_CRC 129

#define MAVLINK_MSG_PARAMETER_SET_FIELD_PARAM_NAME_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAMETER_SET { \
    128, \
    "PARAMETER_SET", \
    2, \
    {  { "param_name", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_parameter_set_t, param_name) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_parameter_set_t, value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAMETER_SET { \
    "PARAMETER_SET", \
    2, \
    {  { "param_name", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_parameter_set_t, param_name) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_parameter_set_t, value) }, \
         } \
}
#endif

/**
 * @brief Pack a parameter_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_name  parameter name
 * @param value  parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parameter_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *param_name, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_SET_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_char_array(buf, 4, param_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#else
    mavlink_parameter_set_t packet;
    packet.value = value;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAMETER_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
}

/**
 * @brief Pack a parameter_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_name  parameter name
 * @param value  parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parameter_set_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *param_name, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_SET_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_char_array(buf, 4, param_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#else
    mavlink_parameter_set_t packet;
    packet.value = value;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAMETER_SET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#endif
}

/**
 * @brief Pack a parameter_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_name  parameter name
 * @param value  parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parameter_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *param_name,float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_SET_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_char_array(buf, 4, param_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#else
    mavlink_parameter_set_t packet;
    packet.value = value;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAMETER_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
}

/**
 * @brief Encode a parameter_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param parameter_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parameter_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_parameter_set_t* parameter_set)
{
    return mavlink_msg_parameter_set_pack(system_id, component_id, msg, parameter_set->param_name, parameter_set->value);
}

/**
 * @brief Encode a parameter_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param parameter_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parameter_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_parameter_set_t* parameter_set)
{
    return mavlink_msg_parameter_set_pack_chan(system_id, component_id, chan, msg, parameter_set->param_name, parameter_set->value);
}

/**
 * @brief Encode a parameter_set struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param parameter_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parameter_set_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_parameter_set_t* parameter_set)
{
    return mavlink_msg_parameter_set_pack_status(system_id, component_id, _status, msg,  parameter_set->param_name, parameter_set->value);
}

/**
 * @brief Send a parameter_set message
 * @param chan MAVLink channel to send the message
 *
 * @param param_name  parameter name
 * @param value  parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_parameter_set_send(mavlink_channel_t chan, const char *param_name, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_SET_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_char_array(buf, 4, param_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER_SET, buf, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
#else
    mavlink_parameter_set_t packet;
    packet.value = value;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER_SET, (const char *)&packet, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
#endif
}

/**
 * @brief Send a parameter_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_parameter_set_send_struct(mavlink_channel_t chan, const mavlink_parameter_set_t* parameter_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_parameter_set_send(chan, parameter_set->param_name, parameter_set->value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER_SET, (const char *)parameter_set, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAMETER_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_parameter_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *param_name, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, value);
    _mav_put_char_array(buf, 4, param_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER_SET, buf, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
#else
    mavlink_parameter_set_t *packet = (mavlink_parameter_set_t *)msgbuf;
    packet->value = value;
    mav_array_memcpy(packet->param_name, param_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER_SET, (const char *)packet, MAVLINK_MSG_ID_PARAMETER_SET_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_SET_LEN, MAVLINK_MSG_ID_PARAMETER_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAMETER_SET UNPACKING


/**
 * @brief Get field param_name from parameter_set message
 *
 * @return  parameter name
 */
static inline uint16_t mavlink_msg_parameter_set_get_param_name(const mavlink_message_t* msg, char *param_name)
{
    return _MAV_RETURN_char_array(msg, param_name, 16,  4);
}

/**
 * @brief Get field value from parameter_set message
 *
 * @return  parameter value
 */
static inline float mavlink_msg_parameter_set_get_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a parameter_set message into a struct
 *
 * @param msg The message to decode
 * @param parameter_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_parameter_set_decode(const mavlink_message_t* msg, mavlink_parameter_set_t* parameter_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    parameter_set->value = mavlink_msg_parameter_set_get_value(msg);
    mavlink_msg_parameter_set_get_param_name(msg, parameter_set->param_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAMETER_SET_LEN? msg->len : MAVLINK_MSG_ID_PARAMETER_SET_LEN;
        memset(parameter_set, 0, MAVLINK_MSG_ID_PARAMETER_SET_LEN);
    memcpy(parameter_set, _MAV_PAYLOAD(msg), len);
#endif
}
