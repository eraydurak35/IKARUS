#pragma once
// MESSAGE GNSS PACKING

#define MAVLINK_MSG_ID_GNSS 119


typedef struct __mavlink_gnss_t {
 int32_t latitude; /*<  GNSS latitude degree * e-7*/
 int32_t longitude; /*<  GNSS longitude degree * e-7*/
 int32_t altitude_mm; /*<  GNSS altitude in mm*/
 int32_t vel_north_mms; /*<  GNSS velocity north in mm second*/
 int32_t vel_east_mms; /*<  GNSS velocity east in mm second*/
 int32_t vel_down_mms; /*<  GNSS velocity down in mm second*/
 int32_t cog_degree; /*<  GNSS cog in degrees*/
 uint16_t hdop; /*<  GNSS hdop*/
 uint16_t vdop; /*<  GNSS vdop*/
 uint8_t fix; /*<  GNSS Fix type*/
 uint8_t sat_count; /*<  GNSS satellite count*/
} mavlink_gnss_t;

#define MAVLINK_MSG_ID_GNSS_LEN 34
#define MAVLINK_MSG_ID_GNSS_MIN_LEN 34
#define MAVLINK_MSG_ID_119_LEN 34
#define MAVLINK_MSG_ID_119_MIN_LEN 34

#define MAVLINK_MSG_ID_GNSS_CRC 6
#define MAVLINK_MSG_ID_119_CRC 6



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GNSS { \
    119, \
    "GNSS", \
    11, \
    {  { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gnss_t, fix) }, \
         { "sat_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gnss_t, sat_count) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gnss_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gnss_t, longitude) }, \
         { "altitude_mm", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gnss_t, altitude_mm) }, \
         { "vel_north_mms", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gnss_t, vel_north_mms) }, \
         { "vel_east_mms", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gnss_t, vel_east_mms) }, \
         { "vel_down_mms", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_gnss_t, vel_down_mms) }, \
         { "cog_degree", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_gnss_t, cog_degree) }, \
         { "hdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gnss_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_gnss_t, vdop) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GNSS { \
    "GNSS", \
    11, \
    {  { "fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gnss_t, fix) }, \
         { "sat_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gnss_t, sat_count) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_gnss_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gnss_t, longitude) }, \
         { "altitude_mm", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gnss_t, altitude_mm) }, \
         { "vel_north_mms", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gnss_t, vel_north_mms) }, \
         { "vel_east_mms", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gnss_t, vel_east_mms) }, \
         { "vel_down_mms", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_gnss_t, vel_down_mms) }, \
         { "cog_degree", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_gnss_t, cog_degree) }, \
         { "hdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gnss_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_gnss_t, vdop) }, \
         } \
}
#endif

/**
 * @brief Pack a gnss message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fix  GNSS Fix type
 * @param sat_count  GNSS satellite count
 * @param latitude  GNSS latitude degree * e-7
 * @param longitude  GNSS longitude degree * e-7
 * @param altitude_mm  GNSS altitude in mm
 * @param vel_north_mms  GNSS velocity north in mm second
 * @param vel_east_mms  GNSS velocity east in mm second
 * @param vel_down_mms  GNSS velocity down in mm second
 * @param cog_degree  GNSS cog in degrees
 * @param hdop  GNSS hdop
 * @param vdop  GNSS vdop
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gnss_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t fix, uint8_t sat_count, int32_t latitude, int32_t longitude, int32_t altitude_mm, int32_t vel_north_mms, int32_t vel_east_mms, int32_t vel_down_mms, int32_t cog_degree, uint16_t hdop, uint16_t vdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude_mm);
    _mav_put_int32_t(buf, 12, vel_north_mms);
    _mav_put_int32_t(buf, 16, vel_east_mms);
    _mav_put_int32_t(buf, 20, vel_down_mms);
    _mav_put_int32_t(buf, 24, cog_degree);
    _mav_put_uint16_t(buf, 28, hdop);
    _mav_put_uint16_t(buf, 30, vdop);
    _mav_put_uint8_t(buf, 32, fix);
    _mav_put_uint8_t(buf, 33, sat_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GNSS_LEN);
#else
    mavlink_gnss_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_mm = altitude_mm;
    packet.vel_north_mms = vel_north_mms;
    packet.vel_east_mms = vel_east_mms;
    packet.vel_down_mms = vel_down_mms;
    packet.cog_degree = cog_degree;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.fix = fix;
    packet.sat_count = sat_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GNSS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GNSS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
}

/**
 * @brief Pack a gnss message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param fix  GNSS Fix type
 * @param sat_count  GNSS satellite count
 * @param latitude  GNSS latitude degree * e-7
 * @param longitude  GNSS longitude degree * e-7
 * @param altitude_mm  GNSS altitude in mm
 * @param vel_north_mms  GNSS velocity north in mm second
 * @param vel_east_mms  GNSS velocity east in mm second
 * @param vel_down_mms  GNSS velocity down in mm second
 * @param cog_degree  GNSS cog in degrees
 * @param hdop  GNSS hdop
 * @param vdop  GNSS vdop
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gnss_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t fix, uint8_t sat_count, int32_t latitude, int32_t longitude, int32_t altitude_mm, int32_t vel_north_mms, int32_t vel_east_mms, int32_t vel_down_mms, int32_t cog_degree, uint16_t hdop, uint16_t vdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude_mm);
    _mav_put_int32_t(buf, 12, vel_north_mms);
    _mav_put_int32_t(buf, 16, vel_east_mms);
    _mav_put_int32_t(buf, 20, vel_down_mms);
    _mav_put_int32_t(buf, 24, cog_degree);
    _mav_put_uint16_t(buf, 28, hdop);
    _mav_put_uint16_t(buf, 30, vdop);
    _mav_put_uint8_t(buf, 32, fix);
    _mav_put_uint8_t(buf, 33, sat_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GNSS_LEN);
#else
    mavlink_gnss_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_mm = altitude_mm;
    packet.vel_north_mms = vel_north_mms;
    packet.vel_east_mms = vel_east_mms;
    packet.vel_down_mms = vel_down_mms;
    packet.cog_degree = cog_degree;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.fix = fix;
    packet.sat_count = sat_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GNSS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GNSS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN);
#endif
}

/**
 * @brief Pack a gnss message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fix  GNSS Fix type
 * @param sat_count  GNSS satellite count
 * @param latitude  GNSS latitude degree * e-7
 * @param longitude  GNSS longitude degree * e-7
 * @param altitude_mm  GNSS altitude in mm
 * @param vel_north_mms  GNSS velocity north in mm second
 * @param vel_east_mms  GNSS velocity east in mm second
 * @param vel_down_mms  GNSS velocity down in mm second
 * @param cog_degree  GNSS cog in degrees
 * @param hdop  GNSS hdop
 * @param vdop  GNSS vdop
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gnss_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t fix,uint8_t sat_count,int32_t latitude,int32_t longitude,int32_t altitude_mm,int32_t vel_north_mms,int32_t vel_east_mms,int32_t vel_down_mms,int32_t cog_degree,uint16_t hdop,uint16_t vdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude_mm);
    _mav_put_int32_t(buf, 12, vel_north_mms);
    _mav_put_int32_t(buf, 16, vel_east_mms);
    _mav_put_int32_t(buf, 20, vel_down_mms);
    _mav_put_int32_t(buf, 24, cog_degree);
    _mav_put_uint16_t(buf, 28, hdop);
    _mav_put_uint16_t(buf, 30, vdop);
    _mav_put_uint8_t(buf, 32, fix);
    _mav_put_uint8_t(buf, 33, sat_count);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GNSS_LEN);
#else
    mavlink_gnss_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_mm = altitude_mm;
    packet.vel_north_mms = vel_north_mms;
    packet.vel_east_mms = vel_east_mms;
    packet.vel_down_mms = vel_down_mms;
    packet.cog_degree = cog_degree;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.fix = fix;
    packet.sat_count = sat_count;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GNSS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GNSS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
}

/**
 * @brief Encode a gnss struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gnss C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gnss_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gnss_t* gnss)
{
    return mavlink_msg_gnss_pack(system_id, component_id, msg, gnss->fix, gnss->sat_count, gnss->latitude, gnss->longitude, gnss->altitude_mm, gnss->vel_north_mms, gnss->vel_east_mms, gnss->vel_down_mms, gnss->cog_degree, gnss->hdop, gnss->vdop);
}

/**
 * @brief Encode a gnss struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gnss C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gnss_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gnss_t* gnss)
{
    return mavlink_msg_gnss_pack_chan(system_id, component_id, chan, msg, gnss->fix, gnss->sat_count, gnss->latitude, gnss->longitude, gnss->altitude_mm, gnss->vel_north_mms, gnss->vel_east_mms, gnss->vel_down_mms, gnss->cog_degree, gnss->hdop, gnss->vdop);
}

/**
 * @brief Encode a gnss struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gnss C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gnss_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gnss_t* gnss)
{
    return mavlink_msg_gnss_pack_status(system_id, component_id, _status, msg,  gnss->fix, gnss->sat_count, gnss->latitude, gnss->longitude, gnss->altitude_mm, gnss->vel_north_mms, gnss->vel_east_mms, gnss->vel_down_mms, gnss->cog_degree, gnss->hdop, gnss->vdop);
}

/**
 * @brief Send a gnss message
 * @param chan MAVLink channel to send the message
 *
 * @param fix  GNSS Fix type
 * @param sat_count  GNSS satellite count
 * @param latitude  GNSS latitude degree * e-7
 * @param longitude  GNSS longitude degree * e-7
 * @param altitude_mm  GNSS altitude in mm
 * @param vel_north_mms  GNSS velocity north in mm second
 * @param vel_east_mms  GNSS velocity east in mm second
 * @param vel_down_mms  GNSS velocity down in mm second
 * @param cog_degree  GNSS cog in degrees
 * @param hdop  GNSS hdop
 * @param vdop  GNSS vdop
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gnss_send(mavlink_channel_t chan, uint8_t fix, uint8_t sat_count, int32_t latitude, int32_t longitude, int32_t altitude_mm, int32_t vel_north_mms, int32_t vel_east_mms, int32_t vel_down_mms, int32_t cog_degree, uint16_t hdop, uint16_t vdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_LEN];
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude_mm);
    _mav_put_int32_t(buf, 12, vel_north_mms);
    _mav_put_int32_t(buf, 16, vel_east_mms);
    _mav_put_int32_t(buf, 20, vel_down_mms);
    _mav_put_int32_t(buf, 24, cog_degree);
    _mav_put_uint16_t(buf, 28, hdop);
    _mav_put_uint16_t(buf, 30, vdop);
    _mav_put_uint8_t(buf, 32, fix);
    _mav_put_uint8_t(buf, 33, sat_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS, buf, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
#else
    mavlink_gnss_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude_mm = altitude_mm;
    packet.vel_north_mms = vel_north_mms;
    packet.vel_east_mms = vel_east_mms;
    packet.vel_down_mms = vel_down_mms;
    packet.cog_degree = cog_degree;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.fix = fix;
    packet.sat_count = sat_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS, (const char *)&packet, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
#endif
}

/**
 * @brief Send a gnss message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gnss_send_struct(mavlink_channel_t chan, const mavlink_gnss_t* gnss)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gnss_send(chan, gnss->fix, gnss->sat_count, gnss->latitude, gnss->longitude, gnss->altitude_mm, gnss->vel_north_mms, gnss->vel_east_mms, gnss->vel_down_mms, gnss->cog_degree, gnss->hdop, gnss->vdop);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS, (const char *)gnss, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GNSS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gnss_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t fix, uint8_t sat_count, int32_t latitude, int32_t longitude, int32_t altitude_mm, int32_t vel_north_mms, int32_t vel_east_mms, int32_t vel_down_mms, int32_t cog_degree, uint16_t hdop, uint16_t vdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, latitude);
    _mav_put_int32_t(buf, 4, longitude);
    _mav_put_int32_t(buf, 8, altitude_mm);
    _mav_put_int32_t(buf, 12, vel_north_mms);
    _mav_put_int32_t(buf, 16, vel_east_mms);
    _mav_put_int32_t(buf, 20, vel_down_mms);
    _mav_put_int32_t(buf, 24, cog_degree);
    _mav_put_uint16_t(buf, 28, hdop);
    _mav_put_uint16_t(buf, 30, vdop);
    _mav_put_uint8_t(buf, 32, fix);
    _mav_put_uint8_t(buf, 33, sat_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS, buf, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
#else
    mavlink_gnss_t *packet = (mavlink_gnss_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->altitude_mm = altitude_mm;
    packet->vel_north_mms = vel_north_mms;
    packet->vel_east_mms = vel_east_mms;
    packet->vel_down_mms = vel_down_mms;
    packet->cog_degree = cog_degree;
    packet->hdop = hdop;
    packet->vdop = vdop;
    packet->fix = fix;
    packet->sat_count = sat_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS, (const char *)packet, MAVLINK_MSG_ID_GNSS_MIN_LEN, MAVLINK_MSG_ID_GNSS_LEN, MAVLINK_MSG_ID_GNSS_CRC);
#endif
}
#endif

#endif

// MESSAGE GNSS UNPACKING


/**
 * @brief Get field fix from gnss message
 *
 * @return  GNSS Fix type
 */
static inline uint8_t mavlink_msg_gnss_get_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field sat_count from gnss message
 *
 * @return  GNSS satellite count
 */
static inline uint8_t mavlink_msg_gnss_get_sat_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field latitude from gnss message
 *
 * @return  GNSS latitude degree * e-7
 */
static inline int32_t mavlink_msg_gnss_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from gnss message
 *
 * @return  GNSS longitude degree * e-7
 */
static inline int32_t mavlink_msg_gnss_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude_mm from gnss message
 *
 * @return  GNSS altitude in mm
 */
static inline int32_t mavlink_msg_gnss_get_altitude_mm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field vel_north_mms from gnss message
 *
 * @return  GNSS velocity north in mm second
 */
static inline int32_t mavlink_msg_gnss_get_vel_north_mms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field vel_east_mms from gnss message
 *
 * @return  GNSS velocity east in mm second
 */
static inline int32_t mavlink_msg_gnss_get_vel_east_mms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field vel_down_mms from gnss message
 *
 * @return  GNSS velocity down in mm second
 */
static inline int32_t mavlink_msg_gnss_get_vel_down_mms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field cog_degree from gnss message
 *
 * @return  GNSS cog in degrees
 */
static inline int32_t mavlink_msg_gnss_get_cog_degree(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field hdop from gnss message
 *
 * @return  GNSS hdop
 */
static inline uint16_t mavlink_msg_gnss_get_hdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field vdop from gnss message
 *
 * @return  GNSS vdop
 */
static inline uint16_t mavlink_msg_gnss_get_vdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Decode a gnss message into a struct
 *
 * @param msg The message to decode
 * @param gnss C-struct to decode the message contents into
 */
static inline void mavlink_msg_gnss_decode(const mavlink_message_t* msg, mavlink_gnss_t* gnss)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gnss->latitude = mavlink_msg_gnss_get_latitude(msg);
    gnss->longitude = mavlink_msg_gnss_get_longitude(msg);
    gnss->altitude_mm = mavlink_msg_gnss_get_altitude_mm(msg);
    gnss->vel_north_mms = mavlink_msg_gnss_get_vel_north_mms(msg);
    gnss->vel_east_mms = mavlink_msg_gnss_get_vel_east_mms(msg);
    gnss->vel_down_mms = mavlink_msg_gnss_get_vel_down_mms(msg);
    gnss->cog_degree = mavlink_msg_gnss_get_cog_degree(msg);
    gnss->hdop = mavlink_msg_gnss_get_hdop(msg);
    gnss->vdop = mavlink_msg_gnss_get_vdop(msg);
    gnss->fix = mavlink_msg_gnss_get_fix(msg);
    gnss->sat_count = mavlink_msg_gnss_get_sat_count(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GNSS_LEN? msg->len : MAVLINK_MSG_ID_GNSS_LEN;
        memset(gnss, 0, MAVLINK_MSG_ID_GNSS_LEN);
    memcpy(gnss, _MAV_PAYLOAD(msg), len);
#endif
}
