#pragma once
// MESSAGE RC_CHANNELS PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS 126


typedef struct __mavlink_rc_channels_t {
 int16_t channel_1; /*<  channel 1 (1000 - 2000)*/
 int16_t channel_2; /*<  channel 2 (1000 - 2000)*/
 int16_t channel_3; /*<  channel 3 (1000 - 2000)*/
 int16_t channel_4; /*<  channel 4 (1000 - 2000)*/
 int16_t channel_5; /*<  channel 5 (1000 - 2000)*/
 int16_t channel_6; /*<  channel 6 (1000 - 2000)*/
 int16_t channel_7; /*<  channel 7 (1000 - 2000)*/
 int16_t channel_8; /*<  channel 8 (1000 - 2000)*/
 int16_t channel_9; /*<  channel 9 (1000 - 2000)*/
 int16_t channel_10; /*<  channel 10 (1000 - 2000)*/
 int16_t channel_11; /*<  channel 11 (1000 - 2000)*/
 int16_t channel_12; /*<  channel 12 (1000 - 2000)*/
 int16_t channel_13; /*<  channel 13 (1000 - 2000)*/
 int16_t channel_14; /*<  channel 14 (1000 - 2000)*/
} mavlink_rc_channels_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_LEN 28
#define MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN 28
#define MAVLINK_MSG_ID_126_LEN 28
#define MAVLINK_MSG_ID_126_MIN_LEN 28

#define MAVLINK_MSG_ID_RC_CHANNELS_CRC 103
#define MAVLINK_MSG_ID_126_CRC 103



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RC_CHANNELS { \
    126, \
    "RC_CHANNELS", \
    14, \
    {  { "channel_1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_rc_channels_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_rc_channels_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_rc_channels_t, channel_3) }, \
         { "channel_4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_rc_channels_t, channel_4) }, \
         { "channel_5", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_rc_channels_t, channel_5) }, \
         { "channel_6", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_rc_channels_t, channel_6) }, \
         { "channel_7", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_rc_channels_t, channel_7) }, \
         { "channel_8", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_rc_channels_t, channel_8) }, \
         { "channel_9", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_rc_channels_t, channel_9) }, \
         { "channel_10", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_rc_channels_t, channel_10) }, \
         { "channel_11", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_rc_channels_t, channel_11) }, \
         { "channel_12", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_rc_channels_t, channel_12) }, \
         { "channel_13", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_rc_channels_t, channel_13) }, \
         { "channel_14", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_rc_channels_t, channel_14) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RC_CHANNELS { \
    "RC_CHANNELS", \
    14, \
    {  { "channel_1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_rc_channels_t, channel_1) }, \
         { "channel_2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_rc_channels_t, channel_2) }, \
         { "channel_3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_rc_channels_t, channel_3) }, \
         { "channel_4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_rc_channels_t, channel_4) }, \
         { "channel_5", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_rc_channels_t, channel_5) }, \
         { "channel_6", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_rc_channels_t, channel_6) }, \
         { "channel_7", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_rc_channels_t, channel_7) }, \
         { "channel_8", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_rc_channels_t, channel_8) }, \
         { "channel_9", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_rc_channels_t, channel_9) }, \
         { "channel_10", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_rc_channels_t, channel_10) }, \
         { "channel_11", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_rc_channels_t, channel_11) }, \
         { "channel_12", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_rc_channels_t, channel_12) }, \
         { "channel_13", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_rc_channels_t, channel_13) }, \
         { "channel_14", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_rc_channels_t, channel_14) }, \
         } \
}
#endif

/**
 * @brief Pack a rc_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param channel_1  channel 1 (1000 - 2000)
 * @param channel_2  channel 2 (1000 - 2000)
 * @param channel_3  channel 3 (1000 - 2000)
 * @param channel_4  channel 4 (1000 - 2000)
 * @param channel_5  channel 5 (1000 - 2000)
 * @param channel_6  channel 6 (1000 - 2000)
 * @param channel_7  channel 7 (1000 - 2000)
 * @param channel_8  channel 8 (1000 - 2000)
 * @param channel_9  channel 9 (1000 - 2000)
 * @param channel_10  channel 10 (1000 - 2000)
 * @param channel_11  channel 11 (1000 - 2000)
 * @param channel_12  channel 12 (1000 - 2000)
 * @param channel_13  channel 13 (1000 - 2000)
 * @param channel_14  channel 14 (1000 - 2000)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t channel_4, int16_t channel_5, int16_t channel_6, int16_t channel_7, int16_t channel_8, int16_t channel_9, int16_t channel_10, int16_t channel_11, int16_t channel_12, int16_t channel_13, int16_t channel_14)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
    _mav_put_int16_t(buf, 0, channel_1);
    _mav_put_int16_t(buf, 2, channel_2);
    _mav_put_int16_t(buf, 4, channel_3);
    _mav_put_int16_t(buf, 6, channel_4);
    _mav_put_int16_t(buf, 8, channel_5);
    _mav_put_int16_t(buf, 10, channel_6);
    _mav_put_int16_t(buf, 12, channel_7);
    _mav_put_int16_t(buf, 14, channel_8);
    _mav_put_int16_t(buf, 16, channel_9);
    _mav_put_int16_t(buf, 18, channel_10);
    _mav_put_int16_t(buf, 20, channel_11);
    _mav_put_int16_t(buf, 22, channel_12);
    _mav_put_int16_t(buf, 24, channel_13);
    _mav_put_int16_t(buf, 26, channel_14);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#else
    mavlink_rc_channels_t packet;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    packet.channel_8 = channel_8;
    packet.channel_9 = channel_9;
    packet.channel_10 = channel_10;
    packet.channel_11 = channel_11;
    packet.channel_12 = channel_12;
    packet.channel_13 = channel_13;
    packet.channel_14 = channel_14;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
}

/**
 * @brief Pack a rc_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param channel_1  channel 1 (1000 - 2000)
 * @param channel_2  channel 2 (1000 - 2000)
 * @param channel_3  channel 3 (1000 - 2000)
 * @param channel_4  channel 4 (1000 - 2000)
 * @param channel_5  channel 5 (1000 - 2000)
 * @param channel_6  channel 6 (1000 - 2000)
 * @param channel_7  channel 7 (1000 - 2000)
 * @param channel_8  channel 8 (1000 - 2000)
 * @param channel_9  channel 9 (1000 - 2000)
 * @param channel_10  channel 10 (1000 - 2000)
 * @param channel_11  channel 11 (1000 - 2000)
 * @param channel_12  channel 12 (1000 - 2000)
 * @param channel_13  channel 13 (1000 - 2000)
 * @param channel_14  channel 14 (1000 - 2000)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t channel_4, int16_t channel_5, int16_t channel_6, int16_t channel_7, int16_t channel_8, int16_t channel_9, int16_t channel_10, int16_t channel_11, int16_t channel_12, int16_t channel_13, int16_t channel_14)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
    _mav_put_int16_t(buf, 0, channel_1);
    _mav_put_int16_t(buf, 2, channel_2);
    _mav_put_int16_t(buf, 4, channel_3);
    _mav_put_int16_t(buf, 6, channel_4);
    _mav_put_int16_t(buf, 8, channel_5);
    _mav_put_int16_t(buf, 10, channel_6);
    _mav_put_int16_t(buf, 12, channel_7);
    _mav_put_int16_t(buf, 14, channel_8);
    _mav_put_int16_t(buf, 16, channel_9);
    _mav_put_int16_t(buf, 18, channel_10);
    _mav_put_int16_t(buf, 20, channel_11);
    _mav_put_int16_t(buf, 22, channel_12);
    _mav_put_int16_t(buf, 24, channel_13);
    _mav_put_int16_t(buf, 26, channel_14);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#else
    mavlink_rc_channels_t packet;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    packet.channel_8 = channel_8;
    packet.channel_9 = channel_9;
    packet.channel_10 = channel_10;
    packet.channel_11 = channel_11;
    packet.channel_12 = channel_12;
    packet.channel_13 = channel_13;
    packet.channel_14 = channel_14;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
}

/**
 * @brief Pack a rc_channels message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param channel_1  channel 1 (1000 - 2000)
 * @param channel_2  channel 2 (1000 - 2000)
 * @param channel_3  channel 3 (1000 - 2000)
 * @param channel_4  channel 4 (1000 - 2000)
 * @param channel_5  channel 5 (1000 - 2000)
 * @param channel_6  channel 6 (1000 - 2000)
 * @param channel_7  channel 7 (1000 - 2000)
 * @param channel_8  channel 8 (1000 - 2000)
 * @param channel_9  channel 9 (1000 - 2000)
 * @param channel_10  channel 10 (1000 - 2000)
 * @param channel_11  channel 11 (1000 - 2000)
 * @param channel_12  channel 12 (1000 - 2000)
 * @param channel_13  channel 13 (1000 - 2000)
 * @param channel_14  channel 14 (1000 - 2000)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t channel_1,int16_t channel_2,int16_t channel_3,int16_t channel_4,int16_t channel_5,int16_t channel_6,int16_t channel_7,int16_t channel_8,int16_t channel_9,int16_t channel_10,int16_t channel_11,int16_t channel_12,int16_t channel_13,int16_t channel_14)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
    _mav_put_int16_t(buf, 0, channel_1);
    _mav_put_int16_t(buf, 2, channel_2);
    _mav_put_int16_t(buf, 4, channel_3);
    _mav_put_int16_t(buf, 6, channel_4);
    _mav_put_int16_t(buf, 8, channel_5);
    _mav_put_int16_t(buf, 10, channel_6);
    _mav_put_int16_t(buf, 12, channel_7);
    _mav_put_int16_t(buf, 14, channel_8);
    _mav_put_int16_t(buf, 16, channel_9);
    _mav_put_int16_t(buf, 18, channel_10);
    _mav_put_int16_t(buf, 20, channel_11);
    _mav_put_int16_t(buf, 22, channel_12);
    _mav_put_int16_t(buf, 24, channel_13);
    _mav_put_int16_t(buf, 26, channel_14);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#else
    mavlink_rc_channels_t packet;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    packet.channel_8 = channel_8;
    packet.channel_9 = channel_9;
    packet.channel_10 = channel_10;
    packet.channel_11 = channel_11;
    packet.channel_12 = channel_12;
    packet.channel_13 = channel_13;
    packet.channel_14 = channel_14;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
}

/**
 * @brief Encode a rc_channels struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_t* rc_channels)
{
    return mavlink_msg_rc_channels_pack(system_id, component_id, msg, rc_channels->channel_1, rc_channels->channel_2, rc_channels->channel_3, rc_channels->channel_4, rc_channels->channel_5, rc_channels->channel_6, rc_channels->channel_7, rc_channels->channel_8, rc_channels->channel_9, rc_channels->channel_10, rc_channels->channel_11, rc_channels->channel_12, rc_channels->channel_13, rc_channels->channel_14);
}

/**
 * @brief Encode a rc_channels struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_t* rc_channels)
{
    return mavlink_msg_rc_channels_pack_chan(system_id, component_id, chan, msg, rc_channels->channel_1, rc_channels->channel_2, rc_channels->channel_3, rc_channels->channel_4, rc_channels->channel_5, rc_channels->channel_6, rc_channels->channel_7, rc_channels->channel_8, rc_channels->channel_9, rc_channels->channel_10, rc_channels->channel_11, rc_channels->channel_12, rc_channels->channel_13, rc_channels->channel_14);
}

/**
 * @brief Encode a rc_channels struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rc_channels_t* rc_channels)
{
    return mavlink_msg_rc_channels_pack_status(system_id, component_id, _status, msg,  rc_channels->channel_1, rc_channels->channel_2, rc_channels->channel_3, rc_channels->channel_4, rc_channels->channel_5, rc_channels->channel_6, rc_channels->channel_7, rc_channels->channel_8, rc_channels->channel_9, rc_channels->channel_10, rc_channels->channel_11, rc_channels->channel_12, rc_channels->channel_13, rc_channels->channel_14);
}

/**
 * @brief Send a rc_channels message
 * @param chan MAVLink channel to send the message
 *
 * @param channel_1  channel 1 (1000 - 2000)
 * @param channel_2  channel 2 (1000 - 2000)
 * @param channel_3  channel 3 (1000 - 2000)
 * @param channel_4  channel 4 (1000 - 2000)
 * @param channel_5  channel 5 (1000 - 2000)
 * @param channel_6  channel 6 (1000 - 2000)
 * @param channel_7  channel 7 (1000 - 2000)
 * @param channel_8  channel 8 (1000 - 2000)
 * @param channel_9  channel 9 (1000 - 2000)
 * @param channel_10  channel 10 (1000 - 2000)
 * @param channel_11  channel 11 (1000 - 2000)
 * @param channel_12  channel 12 (1000 - 2000)
 * @param channel_13  channel 13 (1000 - 2000)
 * @param channel_14  channel 14 (1000 - 2000)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_send(mavlink_channel_t chan, int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t channel_4, int16_t channel_5, int16_t channel_6, int16_t channel_7, int16_t channel_8, int16_t channel_9, int16_t channel_10, int16_t channel_11, int16_t channel_12, int16_t channel_13, int16_t channel_14)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
    _mav_put_int16_t(buf, 0, channel_1);
    _mav_put_int16_t(buf, 2, channel_2);
    _mav_put_int16_t(buf, 4, channel_3);
    _mav_put_int16_t(buf, 6, channel_4);
    _mav_put_int16_t(buf, 8, channel_5);
    _mav_put_int16_t(buf, 10, channel_6);
    _mav_put_int16_t(buf, 12, channel_7);
    _mav_put_int16_t(buf, 14, channel_8);
    _mav_put_int16_t(buf, 16, channel_9);
    _mav_put_int16_t(buf, 18, channel_10);
    _mav_put_int16_t(buf, 20, channel_11);
    _mav_put_int16_t(buf, 22, channel_12);
    _mav_put_int16_t(buf, 24, channel_13);
    _mav_put_int16_t(buf, 26, channel_14);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, buf, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    mavlink_rc_channels_t packet;
    packet.channel_1 = channel_1;
    packet.channel_2 = channel_2;
    packet.channel_3 = channel_3;
    packet.channel_4 = channel_4;
    packet.channel_5 = channel_5;
    packet.channel_6 = channel_6;
    packet.channel_7 = channel_7;
    packet.channel_8 = channel_8;
    packet.channel_9 = channel_9;
    packet.channel_10 = channel_10;
    packet.channel_11 = channel_11;
    packet.channel_12 = channel_12;
    packet.channel_13 = channel_13;
    packet.channel_14 = channel_14;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#endif
}

/**
 * @brief Send a rc_channels message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rc_channels_send_struct(mavlink_channel_t chan, const mavlink_rc_channels_t* rc_channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rc_channels_send(chan, rc_channels->channel_1, rc_channels->channel_2, rc_channels->channel_3, rc_channels->channel_4, rc_channels->channel_5, rc_channels->channel_6, rc_channels->channel_7, rc_channels->channel_8, rc_channels->channel_9, rc_channels->channel_10, rc_channels->channel_11, rc_channels->channel_12, rc_channels->channel_13, rc_channels->channel_14);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)rc_channels, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RC_CHANNELS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rc_channels_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t channel_1, int16_t channel_2, int16_t channel_3, int16_t channel_4, int16_t channel_5, int16_t channel_6, int16_t channel_7, int16_t channel_8, int16_t channel_9, int16_t channel_10, int16_t channel_11, int16_t channel_12, int16_t channel_13, int16_t channel_14)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, channel_1);
    _mav_put_int16_t(buf, 2, channel_2);
    _mav_put_int16_t(buf, 4, channel_3);
    _mav_put_int16_t(buf, 6, channel_4);
    _mav_put_int16_t(buf, 8, channel_5);
    _mav_put_int16_t(buf, 10, channel_6);
    _mav_put_int16_t(buf, 12, channel_7);
    _mav_put_int16_t(buf, 14, channel_8);
    _mav_put_int16_t(buf, 16, channel_9);
    _mav_put_int16_t(buf, 18, channel_10);
    _mav_put_int16_t(buf, 20, channel_11);
    _mav_put_int16_t(buf, 22, channel_12);
    _mav_put_int16_t(buf, 24, channel_13);
    _mav_put_int16_t(buf, 26, channel_14);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, buf, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    mavlink_rc_channels_t *packet = (mavlink_rc_channels_t *)msgbuf;
    packet->channel_1 = channel_1;
    packet->channel_2 = channel_2;
    packet->channel_3 = channel_3;
    packet->channel_4 = channel_4;
    packet->channel_5 = channel_5;
    packet->channel_6 = channel_6;
    packet->channel_7 = channel_7;
    packet->channel_8 = channel_8;
    packet->channel_9 = channel_9;
    packet->channel_10 = channel_10;
    packet->channel_11 = channel_11;
    packet->channel_12 = channel_12;
    packet->channel_13 = channel_13;
    packet->channel_14 = channel_14;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)packet, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#endif
}
#endif

#endif

// MESSAGE RC_CHANNELS UNPACKING


/**
 * @brief Get field channel_1 from rc_channels message
 *
 * @return  channel 1 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field channel_2 from rc_channels message
 *
 * @return  channel 2 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field channel_3 from rc_channels message
 *
 * @return  channel 3 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field channel_4 from rc_channels message
 *
 * @return  channel 4 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field channel_5 from rc_channels message
 *
 * @return  channel 5 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field channel_6 from rc_channels message
 *
 * @return  channel 6 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field channel_7 from rc_channels message
 *
 * @return  channel 7 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field channel_8 from rc_channels message
 *
 * @return  channel 8 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field channel_9 from rc_channels message
 *
 * @return  channel 9 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_9(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field channel_10 from rc_channels message
 *
 * @return  channel 10 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_10(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field channel_11 from rc_channels message
 *
 * @return  channel 11 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_11(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field channel_12 from rc_channels message
 *
 * @return  channel 12 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_12(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field channel_13 from rc_channels message
 *
 * @return  channel 13 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_13(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field channel_14 from rc_channels message
 *
 * @return  channel 14 (1000 - 2000)
 */
static inline int16_t mavlink_msg_rc_channels_get_channel_14(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Decode a rc_channels message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_decode(const mavlink_message_t* msg, mavlink_rc_channels_t* rc_channels)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rc_channels->channel_1 = mavlink_msg_rc_channels_get_channel_1(msg);
    rc_channels->channel_2 = mavlink_msg_rc_channels_get_channel_2(msg);
    rc_channels->channel_3 = mavlink_msg_rc_channels_get_channel_3(msg);
    rc_channels->channel_4 = mavlink_msg_rc_channels_get_channel_4(msg);
    rc_channels->channel_5 = mavlink_msg_rc_channels_get_channel_5(msg);
    rc_channels->channel_6 = mavlink_msg_rc_channels_get_channel_6(msg);
    rc_channels->channel_7 = mavlink_msg_rc_channels_get_channel_7(msg);
    rc_channels->channel_8 = mavlink_msg_rc_channels_get_channel_8(msg);
    rc_channels->channel_9 = mavlink_msg_rc_channels_get_channel_9(msg);
    rc_channels->channel_10 = mavlink_msg_rc_channels_get_channel_10(msg);
    rc_channels->channel_11 = mavlink_msg_rc_channels_get_channel_11(msg);
    rc_channels->channel_12 = mavlink_msg_rc_channels_get_channel_12(msg);
    rc_channels->channel_13 = mavlink_msg_rc_channels_get_channel_13(msg);
    rc_channels->channel_14 = mavlink_msg_rc_channels_get_channel_14(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RC_CHANNELS_LEN? msg->len : MAVLINK_MSG_ID_RC_CHANNELS_LEN;
        memset(rc_channels, 0, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
    memcpy(rc_channels, _MAV_PAYLOAD(msg), len);
#endif
}
