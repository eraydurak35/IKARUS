/** @file
 *  @brief MAVLink comm protocol generated from Ikarus_messages.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_IKARUS_MESSAGES_H
#define MAVLINK_IKARUS_MESSAGES_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_IKARUS_MESSAGES.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_IKARUS_MESSAGES_XML_HASH 6529897440861072850

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {107, 108, 64, 65, 0, 0, 0}, {113, 124, 36, 39, 0, 0, 0}, {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_IKARUS_MESSAGES

// ENUM DEFINITIONS


/** @brief Flags in the HIL_SENSOR message indicate which fields have updated since the last message */
#ifndef HAVE_ENUM_HIL_SENSOR_UPDATED_FLAGS
#define HAVE_ENUM_HIL_SENSOR_UPDATED_FLAGS
typedef enum HIL_SENSOR_UPDATED_FLAGS
{
   HIL_SENSOR_UPDATED_NONE=0, /* None of the fields in HIL_SENSOR have been updated | */
   HIL_SENSOR_UPDATED_XACC=1, /* The value in the xacc field has been updated | */
   HIL_SENSOR_UPDATED_YACC=2, /* The value in the yacc field has been updated | */
   HIL_SENSOR_UPDATED_ZACC=4, /* The value in the zacc field has been updated | */
   HIL_SENSOR_UPDATED_XGYRO=8, /* The value in the xgyro field has been updated | */
   HIL_SENSOR_UPDATED_YGYRO=16, /* The value in the ygyro field has been updated | */
   HIL_SENSOR_UPDATED_ZGYRO=32, /* The value in the zgyro field has been updated | */
   HIL_SENSOR_UPDATED_XMAG=64, /* The value in the xmag field has been updated | */
   HIL_SENSOR_UPDATED_YMAG=128, /* The value in the ymag field has been updated | */
   HIL_SENSOR_UPDATED_ZMAG=256, /* The value in the zmag field has been updated | */
   HIL_SENSOR_UPDATED_ABS_PRESSURE=512, /* The value in the abs_pressure field has been updated | */
   HIL_SENSOR_UPDATED_DIFF_PRESSURE=1024, /* The value in the diff_pressure field has been updated | */
   HIL_SENSOR_UPDATED_PRESSURE_ALT=2048, /* The value in the pressure_alt field has been updated | */
   HIL_SENSOR_UPDATED_TEMPERATURE=4096, /* The value in the temperature field has been updated | */
   HIL_SENSOR_UPDATED_RESET=2147483648, /* Full reset of attitude/position/velocities/etc was performed in sim (Bit 31). | */
   HIL_SENSOR_UPDATED_FLAGS_ENUM_END=2147483649, /*  | */
} HIL_SENSOR_UPDATED_FLAGS;
#endif

/** @brief These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
#ifndef HAVE_ENUM_MAV_MODE
#define HAVE_ENUM_MAV_MODE
typedef enum MAV_MODE
{
   MAV_MODE_PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
   MAV_MODE_MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
   MAV_MODE_TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
   MAV_MODE_STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
   MAV_MODE_GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
   MAV_MODE_AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints) | */
   MAV_MODE_ENUM_END=221, /*  | */
} MAV_MODE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_hil_controls.h"
#include "./mavlink_msg_hil_actuator_controls.h"
#include "./mavlink_msg_hil_sensor.h"
#include "./mavlink_msg_hil_gps.h"
#include "./mavlink_msg_hil_optical_flow.h"
#include "./mavlink_msg_hil_state_quaternion.h"

// base include
#include "../standard/standard.h"


#if MAVLINK_IKARUS_MESSAGES_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_HIL_CONTROLS, MAVLINK_MESSAGE_INFO_HIL_ACTUATOR_CONTROLS, MAVLINK_MESSAGE_INFO_HIL_SENSOR, MAVLINK_MESSAGE_INFO_HIL_GPS, MAVLINK_MESSAGE_INFO_HIL_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_HIL_STATE_QUATERNION, MAVLINK_MESSAGE_INFO_PROTOCOL_VERSION}
# define MAVLINK_MESSAGE_NAMES {{ "HEARTBEAT", 0 }, { "HIL_ACTUATOR_CONTROLS", 93 }, { "HIL_CONTROLS", 91 }, { "HIL_GPS", 113 }, { "HIL_OPTICAL_FLOW", 114 }, { "HIL_SENSOR", 107 }, { "HIL_STATE_QUATERNION", 115 }, { "PROTOCOL_VERSION", 300 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_IKARUS_MESSAGES_H
