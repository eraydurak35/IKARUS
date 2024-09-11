#ifndef SETUP_H
#define SETUP_H

#define RC_RANGE_MIN               900
#define RC_RANGE_MAX               2100

#define RC_CH_1                    0
#define RC_CH_2                    1
#define RC_CH_3                    2
#define RC_CH_4                    3
#define RC_CH_5                    4
#define RC_CH_6                    5
#define RC_CH_7                    6
#define RC_CH_8                    7
#define RC_CH_9                    8
#define RC_CH_10                   9
#define RC_CH_11                   10
#define RC_CH_12                   11
#define RC_CH_13                   12
#define RC_CH_14                   13

// Araç tipi seçenekleri
#define CRAFT_TYPE_QUADCOPTER      1
#define CRAFT_TYPE_PLANE           2

// GNSS alıcı seçenekleri
#define GNSS_NONE                  1
#define GNSS_UBX_M8                2
#define GNSS_UBX_M10               3

// Manyetik sensör seçenekleri
#define MAG_NONE                   1
#define MAG_QMC5883L               2
#define MAG_HMC5883L               3

// Optik akış sensörü seçenekleri
#define OPT_FLOW_NONE              1
#define OPT_FLOW_PMW3901           2

// LIDAR sensör seçenekleri
#define LIDAR_NONE                 1
#define LIDAR_TF_LUNA              2

// Haberleşme türü seçenekleri
#define USE_WEBCOMM                1
#define USE_RC_LINK                2

// Motor sinyali seçenekleri
#define MOTOR_CORELESS             1
#define MOTOR_BRUSHLESS_PWM        2
#define MOTOR_BRUSHLESS_DSHOT300   3
#define MOTOR_BRUSHLESS_DSHOT600   4            

//----------------------------------------------------------//
//                  DONANIM KONFIGÜRASYONU                  //
//----------------------------------------------------------//
#define SETUP_BLACKBOX            (false)
#define SETUP_MAGNETO_TYPE        (MAG_NONE)
#define SETUP_CRAFT_TYPE          (CRAFT_TYPE_QUADCOPTER)
#define SETUP_COMM_TYPE           (USE_WEBCOMM)
#define SETUP_MOTOR_TYPE          (MOTOR_CORELESS)    // Eğer SETUP_CRAFT_TYPE --> CRAFT_TYPE_PLANE ise SETUP_MOTOR_TYPE ayardan bağımsız olarak MOTOR_BRUSHLESS_PWM olacaktır
#define SETUP_GNSS_TYPE           (GNSS_NONE)
#define SETUP_OPT_FLOW_TYPE       (OPT_FLOW_NONE)
#define SETUP_LIDAR_TYPE          (LIDAR_NONE)

//----------------------------------------------------------//
//          RADYO ALICI İŞLEV VE KANAL ATAMALARI            //
//----------------------------------------------------------//
#define RC_PITCH_CH               (RC_CH_2)
#define RC_ROLL_CH                (RC_CH_1)
#define RC_YAW_CH                 (RC_CH_4)
#define RC_THROTTLE_CH            (RC_CH_3)
#define RC_ARM_CH                 (RC_CH_5)
#define RC_ALT_HOLD_CH            (RC_CH_8)
#define RC_RTH_CH                 (RC_CH_6)
#define RC_POS_HOLD_CH            (RC_CH_7)
#define RC_WAYPOINT_CH            (RC_CH_7)
#define RC_AUX_5                  (RC_CH_10)
#define RC_AUX_6                  (RC_CH_11)
#define RC_AUX_7                  (RC_CH_12)
#define RC_AUX_8                  (RC_CH_13)
#define RC_AUX_9                  (RC_CH_14)

//----------------------------------------------------------//
//       RADYO ALICI İŞLEV AKTİVASYON ARALIK SEÇİMİ         //
//----------------------------------------------------------//
#define ARM_ON_RANGE_MIN                1700
#define ARM_ON_RANGE_MAX                RC_RANGE_MAX

#define ALT_HOLD_ON_RANGE_MIN           1700
#define ALT_HOLD_ON_RANGE_MAX           RC_RANGE_MAX

#define RTH_ON_RANGE_MIN                1400
#define RTH_ON_RANGE_MAX                1600

#define POS_HOLD_ON_RANGE_MIN           1400
#define POS_HOLD_ON_RANGE_MAX           RC_RANGE_MAX

#define WAYPOINT_ON_RANGE_MIN           1800
#define WAYPOINT_ON_RANGE_MAX           RC_RANGE_MAX


#define ARM_ON_CONDITION(x)             (x > ARM_ON_RANGE_MIN && x < ARM_ON_RANGE_MAX)
#define ALT_HOLD_ON_CONDITION(x)        (x > ALT_HOLD_ON_RANGE_MIN && x < ALT_HOLD_ON_RANGE_MAX)
#define RTH_ON_CONDITION(x)             (x > RTH_ON_RANGE_MIN && x < RTH_ON_RANGE_MAX)
#define POS_HOLD_ON_CONDITION(x)        (x > POS_HOLD_ON_RANGE_MIN && x < POS_HOLD_ON_RANGE_MAX)
#define WAYPOINT_ON_CONDITION(x)        (x > WAYPOINT_ON_RANGE_MIN && x < WAYPOINT_ON_RANGE_MAX)

#endif