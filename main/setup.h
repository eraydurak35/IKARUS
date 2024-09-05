#ifndef SETUP_H
#define SETUP_H

#define RC_CH_1                     0
#define RC_CH_2                     1
#define RC_CH_3                     2
#define RC_CH_4                     3
#define RC_CH_5                     4
#define RC_CH_6                     5
#define RC_CH_7                     6
#define RC_CH_8                     7
#define RC_CH_9                     8
#define RC_CH_10                    9
#define RC_CH_11                   10
#define RC_CH_12                   11
#define RC_CH_13                   12
#define RC_CH_14                   13

// SETUP_CRAFT_TYPE configuration
#define CRAFT_TYPE_QUADCOPTER      1
#define CRAFT_TYPE_PLANE           2

// SETUP_GNSS_TYPE configuration
#define GNSS_UBX_M8                1
#define GNSS_UBX_M10               2
#define GNSS_NONE                  3

#define MAG_QMC5883L               1
#define MAG_HMC5883L               2
#define MAG_NONE                   3

// SETUP_COMM_TYPE configuration
#define USE_WEBCOMM                1
#define USE_RC_LINK                2

// SETUP_MOTOR_TYPE configuration
#define MOTOR_CORELESS             1
#define MOTOR_BRUSHLESS_PWM        2
#define MOTOR_BRUSHLESS_DSHOT300   3
#define MOTOR_BRUSHLESS_DSHOT600   4            

// Configuration selection
#define SETUP_MAGNETO_TYPE        (MAG_NONE)
#define SETUP_CRAFT_TYPE          (CRAFT_TYPE_QUADCOPTER)
#define SETUP_COMM_TYPE           (USE_WEBCOMM)
#define SETUP_MOTOR_TYPE          (MOTOR_CORELESS)    // Eğer SETUP_CRAFT_TYPE --> CRAFT_TYPE_PLANE ise SETUP_MOTOR_TYPE ayardan bağımsız olarak MOTOR_BRUSHLESS_PWM olacaktır
#define SETUP_GNSS_TYPE           (GNSS_NONE)

// RC receiver channel mappings
#define RC_PITCH_CH               (RC_CH_2)
#define RC_ROLL_CH                (RC_CH_1)
#define RC_YAW_CH                 (RC_CH_4)
#define RC_THROTTLE_CH            (RC_CH_3)
#define RC_AUX_1                  (RC_CH_5)
#define RC_AUX_2                  (RC_CH_6)
#define RC_AUX_3                  (RC_CH_7)
#define RC_AUX_4                  (RC_CH_8)
#define RC_AUX_5                  (RC_CH_9)
#define RC_AUX_6                  (RC_CH_10)
#define RC_AUX_7                  (RC_CH_11)
#define RC_AUX_8                  (RC_CH_12)
#define RC_AUX_9                  (RC_CH_13)
#define RC_AUX_10                 (RC_CH_14)


#endif