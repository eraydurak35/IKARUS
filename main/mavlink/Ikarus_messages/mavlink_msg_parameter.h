#pragma once
// MESSAGE PARAMETER PACKING

#define MAVLINK_MSG_ID_PARAMETER 127


typedef struct __mavlink_parameter_t {
 float value; /*<  parameter value*/
 char param_name[16]; /*<  parameter name*/
 uint8_t type; /*<  parameter type*/
} mavlink_parameter_t;

#define MAVLINK_MSG_ID_PARAMETER_LEN 21
#define MAVLINK_MSG_ID_PARAMETER_MIN_LEN 21
#define MAVLINK_MSG_ID_127_LEN 21
#define MAVLINK_MSG_ID_127_MIN_LEN 21

#define MAVLINK_MSG_ID_PARAMETER_CRC 179
#define MAVLINK_MSG_ID_127_CRC 179

#define MAVLINK_MSG_PARAMETER_FIELD_PARAM_NAME_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAMETER { \
    127, \
    "PARAMETER", \
    3, \
    {  { "param_name", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_parameter_t, param_name) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_parameter_t, value) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_parameter_t, type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAMETER { \
    "PARAMETER", \
    3, \
    {  { "param_name", NULL, MAVLINK_TYPE_CHAR, 16, 4, offsetof(mavlink_parameter_t, param_name) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_parameter_t, value) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_parameter_t, type) }, \
         } \
}
#endif

/**
 * @brief Pack a parameter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_name  parameter name
 * @param value  parameter value
 * @param type  parameter type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parameter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *param_name, float value, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_char_array(buf, 4, param_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAMETER_LEN);
#else
    mavlink_parameter_t packet;
    packet.value = value;
    packet.type = type;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAMETER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
}

/**
 * @brief Pack a parameter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_name  parameter name
 * @param value  parameter value
 * @param type  parameter type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parameter_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *param_name, float value, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_char_array(buf, 4, param_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAMETER_LEN);
#else
    mavlink_parameter_t packet;
    packet.value = value;
    packet.type = type;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAMETER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN);
#endif
}

/**
 * @brief Pack a parameter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_name  parameter name
 * @param value  parameter value
 * @param type  parameter type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_parameter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *param_name,float value,uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_char_array(buf, 4, param_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAMETER_LEN);
#else
    mavlink_parameter_t packet;
    packet.value = value;
    packet.type = type;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAMETER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAMETER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
}

/**
 * @brief Encode a parameter struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param parameter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parameter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_parameter_t* parameter)
{
    return mavlink_msg_parameter_pack(system_id, component_id, msg, parameter->param_name, parameter->value, parameter->type);
}

/**
 * @brief Encode a parameter struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param parameter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parameter_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_parameter_t* parameter)
{
    return mavlink_msg_parameter_pack_chan(system_id, component_id, chan, msg, parameter->param_name, parameter->value, parameter->type);
}

/**
 * @brief Encode a parameter struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param parameter C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_parameter_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_parameter_t* parameter)
{
    return mavlink_msg_parameter_pack_status(system_id, component_id, _status, msg,  parameter->param_name, parameter->value, parameter->type);
}

/**
 * @brief Send a parameter message
 * @param chan MAVLink channel to send the message
 *
 * @param param_name  parameter name
 * @param value  parameter value
 * @param type  parameter type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_parameter_send(mavlink_channel_t chan, const char *param_name, float value, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAMETER_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_char_array(buf, 4, param_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER, buf, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
#else
    mavlink_parameter_t packet;
    packet.value = value;
    packet.type = type;
    mav_array_memcpy(packet.param_name, param_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER, (const char *)&packet, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
#endif
}

/**
 * @brief Send a parameter message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_parameter_send_struct(mavlink_channel_t chan, const mavlink_parameter_t* parameter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_parameter_send(chan, parameter->param_name, parameter->value, parameter->type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER, (const char *)parameter, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAMETER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_parameter_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *param_name, float value, uint8_t type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_char_array(buf, 4, param_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER, buf, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
#else
    mavlink_parameter_t *packet = (mavlink_parameter_t *)msgbuf;
    packet->value = value;
    packet->type = type;
    mav_array_memcpy(packet->param_name, param_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAMETER, (const char *)packet, MAVLINK_MSG_ID_PARAMETER_MIN_LEN, MAVLINK_MSG_ID_PARAMETER_LEN, MAVLINK_MSG_ID_PARAMETER_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAMETER UNPACKING


/**
 * @brief Get field param_name from parameter message
 *
 * @return  parameter name
 */
static inline uint16_t mavlink_msg_parameter_get_param_name(const mavlink_message_t* msg, char *param_name)
{
    return _MAV_RETURN_char_array(msg, param_name, 16,  4);
}

/**
 * @brief Get field value from parameter message
 *
 * @return  parameter value
 */
static inline float mavlink_msg_parameter_get_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field type from parameter message
 *
 * @return  parameter type
 */
static inline uint8_t mavlink_msg_parameter_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Decode a parameter message into a struct
 *
 * @param msg The message to decode
 * @param parameter C-struct to decode the message contents into
 */
static inline void mavlink_msg_parameter_decode(const mavlink_message_t* msg, mavlink_parameter_t* parameter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    parameter->value = mavlink_msg_parameter_get_value(msg);
    mavlink_msg_parameter_get_param_name(msg, parameter->param_name);
    parameter->type = mavlink_msg_parameter_get_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAMETER_LEN? msg->len : MAVLINK_MSG_ID_PARAMETER_LEN;
        memset(parameter, 0, MAVLINK_MSG_ID_PARAMETER_LEN);
    memcpy(parameter, _MAV_PAYLOAD(msg), len);
#endif
}
