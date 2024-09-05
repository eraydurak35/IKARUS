//  8888888 888    d8P         d8888 8888888b.  888     888  .d8888b. 
//    888   888   d8P         d88888 888   Y88b 888     888 d88P  Y88b
//    888   888  d8P         d88P888 888    888 888     888 Y88b.     
//    888   888d88K         d88P 888 888   d88P 888     888  "Y888b.  
//    888   8888888b       d88P  888 8888888P"  888     888     "Y88b.
//    888   888  Y88b     d88P   888 888 T88b   888     888       "888
//    888   888   Y88b   d8888888888 888  T88b  Y88b. .d88P Y88b  d88P
//  8888888 888    Y88b d88P     888 888   T88b  "Y88888P"   "Y8888P" 

// Colossal
//FOR ESP32-S3-MINI

// ||############################||
// ||      ESP IDF LIBRARIES     ||
// ||############################||
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <nvs_flash.h>
#include "esp_wifi.h"
#include <stdio.h>
#include "math.h"
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "esp_now_comm.h"

#include "command_line_interface.h"
#include "small_drone_control.h"
#include "state_estimator.h"
#include "calibration.h"
#include "nv_storage.h"
#include "icm42688p.h"
#include "defaults.h"
#include "web_comm.h"
#include "qmc5883l.h"
#include "typedefs.h"
#include "hmc5883l.h"
#include "filters.h"
#include "bmp390.h"
#include "gpio.h"
#include "setup.h"
#include "ublox.h"
#include "ibus.h"

static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static TaskHandle_t task4_handler;
#if SETUP_GNSS_TYPE != GNSS_NONE
static TaskHandle_t task5_handler;
#endif
#if SETUP_COMM_TYPE == USE_RC_LINK
static TaskHandle_t task6_handler;
static TaskHandle_t task7_handler;
#endif
static esp_timer_handle_t timer1;

static imu_t imu;
static magnetometer_t mag;
static bmp390_t barometer;
static pmw3901_t flow;
static range_finder_t range;
static gnss_t gnss;
static calibration_t mag_calibration_data;
static calibration_t accel_calibration_data;
static flight_t flight;
static target_t target;
static config_t config;
static states_t states;

static biquad_lpf_t lowpass[6];
#if SETUP_COMM_TYPE == USE_WEBCOMM
static telemetry_small_integer_t telemetry;
#else 
static telemetry_t telemetry;
static waypoint_t waypoint;
static gamepad_t gamepad;
#endif

static radio_control_t radio = {{1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1500, 1000, 1000, 1000, 1000, 1000}};

// Task 1'i tetikleyen interrupt fonksiyonu
void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}
// GPIO 0'a bağlı butona hem basıldığında hem çekildiğinde bu interrupt tetiklenir
void IRAM_ATTR button_ISR(void *arg)
{
    // Kalibrasyon görevine butonun durum bilgisini gönder
    xTaskNotifyFromISR(task4_handler, gpio_get_level(BUTTON_PIN), eSetValueWithOverwrite, false);
}

void task_1(void *pvParameters);
void task_2(void *pvParameters);
void task_3(void *pvParameters);
void task_4(void *pvParameters);
#if SETUP_GNSS_TYPE != GNSS_NONE
void task_5(void *pvParameters);
#endif
#if SETUP_COMM_TYPE == USE_RC_LINK
void task_6(void *pvParameters);
void task_7(void *pvParameters);
#endif

// Ana görev
void task_1(void *pvParameters)
{
    #if SETUP_COMM_TYPE == USE_WEBCOMM
    web_comm_init(&radio, &states, &flight, &telemetry);
    small_drone_control_init(&radio, &telemetry, &flight, &target, &states, &config);
    #elif SETUP_COMM_TYPE == USE_RC_LINK
    xTaskCreatePinnedToCore(&task_6, "task6", 1024 * 4, NULL, 0, &task6_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_7, "task7", 1024 * 4, NULL, 0, &task7_handler, tskNO_AFFINITY);
    #endif
    // 3 ivme 3 gyro için lowpass yapısını başlat
    biquad_lpf_array_init(6, lowpass, 80.0f, 1000.0f);
    // Kestirim algoritmasını başlatmadan önce biquad alçak geçiren filtrenini buffer'ını doldur.
    for (uint8_t i = 0; i <= 100; i++)
    {
        icm42688p_read(&imu);
        apply_biquad_lpf_to_imu(&imu, lowpass);
        vTaskDelay(3);
    }
    // Duruş kestirim algoritmasını başlat
    ahrs_init(&config, &states, &imu, &mag, &barometer, &flight);
    //control_init(&gamepad, &telem_small, &flight, &target, &states, &config);
    static uint32_t receivedValue = 0;
    while (1)
    {
        // vTaskDelay kullanmadan timer interrup ile tetiklenerek periyodik olarak çalışan fonksiyon
        // burası her 1ms de bir çalışacak.
        if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            // IMU verilerini oku
            icm42688p_read(&imu);
            // IMU verilerini alçak geçiren filtreden geçir.
            apply_biquad_lpf_to_imu(&imu, lowpass);
            // gyroscope integrali alarak duruşu hesapla
            ahrs_predict();
            // ivme ve manyetik sensör ile duruşu güncelle
            ahrs_correct();
            //printf("%.2f,%.2f,%.2f\n", imu.accel_ms2[X], imu.accel_ms2[Y], imu.accel_ms2[Z]);
            // ivme ölçümlerini body frame'den earth frame'e geçir
            earth_frame_acceleration();
            // earth frame'deki ivme bilgisini kullanarak yükseklik hesapla
            altitude_predict();
            //printf("%.2f,%.2f,%.2f\n", states.acc_forward_ms2, states.acc_right_ms2, states.acc_up_ms2);
            #if SETUP_COMM_TYPE == USE_WEBCOMM && SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER
            small_drone_check_flight_mode();
            small_drone_flight_control();
            #elif SETUP_COMM_TYPE == USE_RC_LINK && SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER

            #endif
        }
    }
}


// Gyro kalibrasyon görevi
void task_2(void *pvParameters)
{
    // Durum ledini söndür
    status_led_set_brightness(0);
    uint8_t blink_counter = 0;

    while (1)
    {
        // IMU'dan veri oku
        icm42688p_read(&imu);
        // 250ms de bir LED'i yak söndür
        blink_counter++;
        if (blink_counter == 249) blink_counter = 0;
        else if (blink_counter == 1) status_led_set_brightness(100);
        else if (blink_counter == 125) status_led_set_brightness(0);

        // fonksiyon 1 döndürürse kalibrasyon tamamlanmış demektir.
        if (gyro_calibration(&imu) == 1)
        {
            // LED yanık kalsın
            status_led_set_brightness(70);
            // Ana görevi başlatabiliriz
            xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);
            // Diğer sensörlerdenden veri okuyan görevi başlat
            xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 4, NULL, 1, &task3_handler, tskNO_AFFINITY);
            // İvme ölçer ve manyetik sensör kalibrasyon görevi. Öncelik değeri (Idle = 0) olarak ayarlı
            xTaskCreatePinnedToCore(&task_4, "task4", 1024 * 4, NULL, 0, &task4_handler, tskNO_AFFINITY);
            #if SETUP_GNSS_TYPE != GNSS_NONE
            // GNSS alıcısından veri okuyan görevi başlat
            xTaskCreatePinnedToCore(&task_5, "task5", 1024 * 4, NULL, 1, &task5_handler, tskNO_AFFINITY);
            #endif
            // Timer interrupt kurulumu
            const esp_timer_create_args_t timer1_args =
            {
                .callback = &timer1_callback,
                .arg = NULL,
                .name = "timer1"
            };
            esp_timer_create(&timer1_args, &timer1);
            // 1000us de bir defa çalışsın
            esp_timer_start_periodic(timer1, 1000);
            puts("IKARUS");
            // Bu task ile işimiz kalmadı. Silebiliriz.
            vTaskDelete(NULL);
            // Kod buraya ulaşmamalı.
            printf("THIS SHOULD NOT PRINT\n");
        }

        // 2 ms bekle
        vTaskDelay(2);
    }
}


void task_3(void *pvParameters)
{
    // Yükseklik hesabında kullanılacak olan sıfır sayılacak basıncı kaydet
    baro_set_ground_pressure(&barometer);
    
    while (1)
    {
        get_battery_voltage(&flight.battery_voltage);
        // Manyetik sensör verisini oku
        #if SETUP_MAGNETO_TYPE == MAG_QMC5883L
        qmc5883l_read(&mag, 0);
        #elif SETUP_MAGNETO_TYPE == MAG_HMC5883L
        hmc5883l_read(&mag);
        #endif
        // Barometrik sensör verisini oku
        bmp390_read_spi(&barometer);
        // Yükseklik kestirimini güncelle
        altitude_correct();
        //printf("%.4f\n", barometer.altitude_m);
        //printf("%.2f,%.2f,%.2f\n", mag.axis[X], mag.axis[Y], mag.axis[Z]);
        //printf("%.2f,%.2f\n", states.altitude_m, barometer.altitude_m);
        vTaskDelay(20);
    }
}

// İvme ölçer ve manyetik sensörün kalibrasyonunu gerçekleştiren görev
void task_4(void *pvParameters)
{
    static int64_t button_push_time_difference;
    static int64_t button_push_current_time;
    static uint32_t button_level;

    while (1)
    {
        // Butona basıldığında veya çekildiğinde bu işlev çalışır. Bu durumlar dışında çalışmaz bekler.
        if (xTaskNotifyWait(0, ULONG_MAX, &button_level, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            // Eğer buton bırakıldıysa button level 1'dir. Basıldı ise 0'dır
            // Sadece buton bırakıldığında süre ölçümü yap
            if (button_level == 1)
            {
                // Butona basılması ile bırakılması arasındaki süreyi ölç
                button_push_time_difference = (esp_timer_get_time() - button_push_current_time);

                // Bu süre (button_push_time_difference) 1 saniyeden kısa ise ivme ölçer kalibrasyonu seçilmiştir (1000000 us = 1 sn)
                if (button_push_time_difference < 1000000)
                {
                    // 1. ve 3. görevleri silebiliriz. Çalışmalarına gerek yok.
                    // timer1 durdurulur ve silinir.
                    esp_timer_stop(timer1);
                    esp_timer_delete(timer1);
                    vTaskDelete(task1_handler);
                    vTaskDelete(task3_handler);
                    // Eski kalibrasyon verilerini sıfırla ki yeni kalibrasyon yapabilelim.
                    reset_calibration_data(&accel_calibration_data);
                    // Kalibrasyon fonksiyonu 1 döndürene kadar döngü devam etsin
                    while (!accelerometer_calibration(&imu))
                    {
                        // IMU'dan yeni veri oku
                        icm42688p_read(&imu);
                        // 5ms bekle
                        vTaskDelay(5);
                    }
                    // Bu noktaya ulaştığında kalibrasyon tamamlanmış demektir.
                    // Soft restart at
                    esp_restart();
                }
                #if SETUP_MAGNETO_TYPE != MAG_NONE
                // Bu süre (button_push_time_difference) 1 saniyeden uzun ise manyetik sensör kalibrasyonu seçilmiştir
                else
                {
                    // 1. ve 3. görevleri silebiliriz. Çalışmalarına gerek yok.
                    // timer1 durdurulur ve silinir.
                    esp_timer_stop(timer1);
                    esp_timer_delete(timer1);
                    vTaskDelete(task1_handler);
                    vTaskDelete(task3_handler);
                    // Eski kalibrasyon verilerini sıfırla ki yeni kalibrasyon yapabilelim.
                    reset_calibration_data(&mag_calibration_data);
                    // Kalibrasyon fonksiyonu 1 döndürene kadar döngü devam etsin
                    while (!magnetometer_calibration(&mag))
                    {
                        // Manyetik sensörden yeni veri oku
                        #if SETUP_MAGNETO_TYPE == MAG_QMC5883L
                        qmc5883l_read(&mag, 0);
                        #elif SETUP_MAGNETO_TYPE == MAG_HMC5883L
                        hmc5883l_read(&mag);
                        #endif
                        // 50ms bekle
                        vTaskDelay(50);
                    }
                    // Bu noktaya ulaştığında kalibrasyon tamamlanmış demektir.
                    // Soft restart at
                    esp_restart();
                }
                #endif
            }
            // Butona her basılıp çekildiğinde o anki zamanı kaydet
            // Bu sayede basıp çekme arasındaki zamanı ölçebilelim
            button_push_current_time = esp_timer_get_time();
        }
    }
}

#if SETUP_GNSS_TYPE != GNSS_NONE
// GNSS alıcından veri okuyan görev
void task_5(void *pvParameters)
{
    while (1)
    {
        // GNSS alıcısından UART üzerinden veri oku
        // bu görev için vTaskDelay kullanımına gerek yok
        // UART fonksiyonları zaten timeout süresi kadar burayı blokluyor
        gnss_read(&gnss);
    }
}
#endif

#if SETUP_COMM_TYPE == USE_RC_LINK
void task_6(void *pvParameters)
{
    static uint8_t flg1, flg2;
    esp_now_comm_init(&config, &waypoint, &flg1, &flg2, &telemetry, &flight, &states, &imu, &mag, &barometer, &gnss, &flow, &range, &target, &gamepad);

    while (1)
    {
        esp_now_send_telemetry();
        vTaskDelay(100);
    }
}

void task_7(void *pvParameters)
{
    while (1)
    {
        ibus_receiver_read(&radio);
    }
}
#endif

void app_main(void)
{
    // Non Volatile Storage birimini başlatır.
    nvs_flash_init();
    // Varsa önceden kaydedilmiş kalibrasyon verilerini ve konfigürasyonu oku. Yoksa default değerler ile başlat
    if (!storage_read(&mag_calibration_data, MAG_CALIB_DATA)) reset_calibration_data(&mag_calibration_data);
    if (!storage_read(&accel_calibration_data, ACCEL_CALIB_DATA)) reset_calibration_data(&accel_calibration_data);
    if (!storage_read(&config, CONFIG_DATA)) load_default_config(&config);
    // GPIO pinlerini konfigüre et
    gpio_configure(&config);
    // Buton pinine interrupt service rouutine ekle
    gpio_isr_handler_add(BUTTON_PIN, button_ISR, (void*) BUTTON_PIN);
    #if SETUP_MAGNETO_TYPE == MAG_QMC5883L
    // Manyetik sensörün ayarlarını yap ve başlat
    qmc5883l_setup(&mag_calibration_data);
    #elif SETUP_MAGNETO_TYPE == MAG_HMC5883L
    hmc5883l_setup(&mag_calibration_data);
    #endif 
    // IMU'nun ayarlarını yap ve başlat
    icm42688p_setup(&accel_calibration_data);
    // bmp3xx sensörünün ayarlarını yap ve başlat
    bmp390_setup_spi();
    // GNSS alıcısı seçildiyse başlat
    #if SETUP_GNSS_TYPE != GNSS_NONE
    gnss_init();
    #endif
    #if SETUP_COMM_TYPE == USE_RC_LINK
    ibus_init(&radio);
    #endif
    // Komut satırı arayüzünü başlatır
    cli_begin(&config, &accel_calibration_data, &mag_calibration_data, &imu);
    // Gyro kalibrasyon prosedürünü başlat. Diğer görevler gyro kalibrasyonu tamamlandığında başlatılır.
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
}
