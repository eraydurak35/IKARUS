#ifndef SETUP_H
#define SETUP_H

#define POSITIVE                   1.0f
#define NEGATIVE                  -1.0f

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
#define MOTOR_BRUSHLESS_DSHOT150   3
#define MOTOR_BRUSHLESS_DSHOT300   4
#define MOTOR_BRUSHLESS_DSHOT600   5

//----------------------------------------------------------//
//                  DONANIM KONFIGÜRASYONU                  //
//----------------------------------------------------------//
#define SETUP_USE_BLACKBOX        (false)                   // "true" --> "ARM" kayıt başlar, "DISARM" kayıt durur. "false" --> devre dışı bırakır.
#define SETUP_MAGNETO_TYPE        (MAG_QMC5883L)            // Manyetik sensör seçimi.
#define SETUP_CRAFT_TYPE          (CRAFT_TYPE_QUADCOPTER)   // Araç tipi seçimi
#define SETUP_COMM_TYPE           (USE_RC_LINK)             // Kontrol methodu seçimi
#define SETUP_MOTOR_TYPE          (MOTOR_BRUSHLESS_DSHOT300)          // Motor kontrol sinyali türü seçimi
#define SETUP_GNSS_TYPE           (GNSS_UBX_M10)               // GNSS alıcı seçimi
#define SETUP_OPT_FLOW_TYPE       (OPT_FLOW_PMW3901)           // Optik akış sensörü seçimi
#define SETUP_LIDAR_TYPE          (LIDAR_TF_LUNA)              // LIDAR sensör türü seçimi

//---- UART PIN KONFIGURASYONU ----//
#define SETUP_UART_0_TX_PIN       (43) // Değiştirilemez.
#define SETUP_UART_0_RX_PIN       (44) // Değiştirilemez.

#define SETUP_UART_1_TX_PIN       (47) // Değiştirilebilir.
#define SETUP_UART_1_RX_PIN       (48) // Değiştirilebilir.

#define SETUP_UART_2_TX_PIN       (18) // Değiştirilebilir.
#define SETUP_UART_2_RX_PIN       (21) // Değiştirilebilir.

//---- SPI PIN KONFIGURASYONU ----//
#define SETUP_SPI_2_CLK_PIN       (13) // Değiştirilemez.
#define SETUP_SPI_2_MISO_PIN      (11) // Değiştirilemez.
#define SETUP_SPI_2_MOSI_PIN      (10) // Değiştirilemez.
#define SETUP_ICM42688P_CS_PIN    (12) // Değiştirilemez.
#define SETUP_BMP390_CS_PIN       (14) // Değiştirilemez.

//---- I2C PIN KONFIGURASYONU ----//
#define SETUP_I2C_0_SDA_PIN       (8) // Değiştirilebilir. Pull up direci bağlıdır.
#define SETUP_I2C_0_SCL_PIN       (9) // Değiştirilebilir. Pull up direci bağlıdır.

//---- MOTOR PIN KONFIGURASYONU ----//
#define SETUP_S1_PIN              (16) // Değiştirilebilir. Fırçalı motor sürücü bu pine bağlıdır.
#define SETUP_S2_PIN              (4)  // Değiştirilebilir. Fırçalı motor sürücü bu pine bağlıdır.
#define SETUP_S3_PIN              (38) // Değiştirilebilir. Fırçalı motor sürücü bu pine bağlıdır.
#define SETUP_S4_PIN              (1)  // Değiştirilebilir. Fırçalı motor sürücü bu pine bağlıdır.

//---- DIGER PINLER ----//
#define SETUP_LED_PIN             (2) // Değiştirilebilir.
#define SETUP_BUTTON_PIN          (0) // Değiştirilemez.
#define SETUP_VSENS_PIN           (6) // Değiştirilebilir. (Pin 1-10 dahil, arası kullanılabilir.)



//----------------------------------------------------------//
//                SENSOR EKSEN HIZALANMALARI                //
//----------------------------------------------------------//

//---- MANYETIK SENSOR EKSEN HIZALANMASI ----//
#define ALIGNED_MAG_X_AXIS        (Y)        // Varsayılan "X"
#define ALIGNED_MAG_Y_AXIS        (X)        // Varsayılan "Y"
#define ALIGNED_MAG_Z_AXIS        (Z)        // Varsayılan "Z"

#define ALIGNED_MAG_X_AXIS_SIGN   (POSITIVE) // Varsayılan "POSITIVE"
#define ALIGNED_MAG_Y_AXIS_SIGN   (NEGATIVE) // Varsayılan "NEGATIVE"
#define ALIGNED_MAG_Z_AXIS_SIGN   (NEGATIVE) // Varsayılan "POSITIVE"

//----- IVME SENSORU EKSEN HIZALANMASI -----//
#define ALIGNED_ACC_X_AXIS        (Y)        // Varsayılan "Y"
#define ALIGNED_ACC_Y_AXIS        (X)        // Varsayılan "X"
#define ALIGNED_ACC_Z_AXIS        (Z)        // Varsayılan "Z"

#define ALIGNED_ACC_X_AXIS_SIGN   (POSITIVE) // Varsayılan "POSITIVE"
#define ALIGNED_ACC_Y_AXIS_SIGN   (POSITIVE) // Varsayılan "POSITIVE"
#define ALIGNED_ACC_Z_AXIS_SIGN   (NEGATIVE) // Varsayılan "NEGATIVE"

//---- JIROSKOP SENSOR EKSEN HIZALANMASI ----//
#define ALIGNED_GYR_X_AXIS        (Y)        // Varsayılan "Y"
#define ALIGNED_GYR_Y_AXIS        (X)        // Varsayılan "X"
#define ALIGNED_GYR_Z_AXIS        (Z)        // Varsayılan "Z"

#define ALIGNED_GYR_X_AXIS_SIGN   (POSITIVE) // Varsayılan "POSITIVE"
#define ALIGNED_GYR_Y_AXIS_SIGN   (POSITIVE) // Varsayılan "POSITIVE"
#define ALIGNED_GYR_Z_AXIS_SIGN   (NEGATIVE) // Varsayılan "NEGATIVE"


#define SETUP_MAIN_LOOP_FREQ_HZ   (1000.0f)


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