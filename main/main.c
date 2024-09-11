//  8888888 888    d8P         d8888 8888888b.  888     888  .d8888b. 
//    888   888   d8P         d88888 888   Y88b 888     888 d88P  Y88b
//    888   888  d8P         d88P888 888    888 888     888 Y88b.     
//    888   888d88K         d88P 888 888   d88P 888     888  "Y888b.  
//    888   8888888b       d88P  888 8888888P"  888     888     "Y88b.
//    888   888  Y88b     d88P   888 888 T88b   888     888       "888
//    888   888   Y88b   d8888888888 888  T88b  Y88b. .d88P Y88b  d88P
//  8888888 888    Y88b d88P     888 888   T88b  "Y88888P"   "Y8888P" 

// Colossal
// For IKARUS Flight Controller Devkit_V1

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
#include "control/small_drone_control.h"
#include "control/quadcopter_control.h"
#include "comminication/esp_now_comm.h"
#include "comminication/web_comm.h"
#include "command_line_interface.h"
#include "storage/nv_storage.h"
#include "sensors/icm42688p.h"
#include "sensors/qmc5883l.h"
#include "sensors/hmc5883l.h"
#include "storage/blackbox.h"
#include "state_estimator.h"
#include "sensors/tf_luna.h"
#include "sensors/pmw3901.h"
#include "sensors/bmp390.h"
#include "sensors/ublox.h"
#include "calibration.h"
#include "defaults.h"
#include "typedefs.h"
#include "filters.h"
#include "setup.h"
#include "ibus.h"
#include "gpio.h"

static esp_timer_handle_t timer1;
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
#if SETUP_OPT_FLOW_TYPE == OPT_FLOW_PMW3901
static TaskHandle_t task8_handler;
#endif

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
static biquad_notch_filter_t notch[6];
static radio_control_t radio = {{1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1500, 1000, 1000, 1000, 1000, 1000}};

#if SETUP_COMM_TYPE == USE_WEBCOMM
static telemetry_small_integer_t telemetry;
#else 
static telemetry_t telemetry;
static waypoint_t waypoint;
static gamepad_t gamepad;
#endif

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
#if SETUP_OPT_FLOW_TYPE == OPT_FLOW_PMW3901
void task_8(void *pvParameters);
#endif

// Ana görev
void task_1(void *pvParameters)
{
    #if SETUP_COMM_TYPE == USE_WEBCOMM
    web_comm_init(&radio, &states, &flight, &telemetry);
    small_drone_control_init(&radio, &telemetry, &flight, &target, &states, &config);
    #elif SETUP_COMM_TYPE == USE_RC_LINK
    quadcopter_control_init(&radio, &telemetry, &flight, &target, &states, &config, &waypoint, &gnss);
    xTaskCreatePinnedToCore(&task_6, "task6", 1024 * 4, NULL, 0, &task6_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_7, "task7", 1024 * 4, NULL, 1, &task7_handler, tskNO_AFFINITY);
    #endif
    #if SETUP_OPT_FLOW_TYPE == OPT_FLOW_PMW3901
    xTaskCreatePinnedToCore(&task_8, "task8", 1024 * 4, NULL, 0, &task8_handler, tskNO_AFFINITY);
    #endif
    // 3 ivme 3 gyro için lowpass yapısını başlat
    biquad_lpf_array_init(6, lowpass, 80.0f, 1000.0f);
    // F450'nin tepe gürültüsü 72Hz bant genişliği 50Hz
    // Fırçalı dronun tepe gürültüsü 262.0Hz bant genişliği 45Hz
    #if SETUP_MOTOR_TYPE == MOTOR_CORELESS
    biquad_notch_filter_array_init(6, notch, 262.0f, 45.0f, 1000.0f);
    #else 
    biquad_notch_filter_array_init(6, notch, 72.0f, 50.0f, 1000.0f);
    #endif
    // Kestirim algoritmasını başlatmadan önce filtrelerin buffer'ını doldur.
    for (uint8_t i = 0; i <= 100; i++)
    {
        icm42688p_read(&imu);
        apply_biquad_lpf_to_imu(&imu, lowpass);
        apply_biquad_notch_filter_to_imu(&imu, notch);
        vTaskDelay(2);
    }
    // Duruş kestirim algoritmasını başlat
    ahrs_init(&config, &states, &imu, &mag, &barometer, &flight);
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
            // IMU verilerini notch filtreden geçir.
            apply_biquad_notch_filter_to_imu(&imu, notch);
            #if SETUP_BLACKBOX == true
            blackbox_save();
            #endif
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
            small_drone_flight_control();
            #elif SETUP_COMM_TYPE == USE_RC_LINK && SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER
            quadcopter_flight_control();
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
            status_led_set_brightness(100);
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
        #if SETUP_LIDAR_TYPE == LIDAR_TF_LUNA
        tf_luna_read_range(&range, &states);
        //printf("%d\n", range.range_cm);
        #endif
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

                #if SETUP_BLACKBOX == true
                // Bu süre (button_push_time_difference) 1 saniyeden kısa ise blackbox verileri yazdırılır.
                if (button_push_time_difference < 1000000)
                {
                    // Eğer kayıtlı veri varsa (blackbox.bin dosyası bulunduysa) bu fonksiyon o dosyayı açar ve 1 döndürür.
                    // Yoksa 0 döndürür
                    if (blackbox_open())
                    {   
                        // Satır satır tüm veriyi sırayla okuyup konsol ekranına virgül ile ayrılmış biçimde yazdırır.
                        // Okunacak yeni bir satır varsa 1 yoksa 0 döndürür
                        while (blackbox_print())
                        {
                            status_led_breathe(1);
                            // Watchdog tetiklenmesin diye her satırdan sonra 1ms bekle
                            vTaskDelay(1);
                        }
                        status_led_set_brightness(100)
                    }
                }
                #endif
                #if SETUP_MAGNETO_TYPE != MAG_NONE
                // Bu süre (button_push_time_difference) 1 ile 2 saniye arasında ise manyetik sensör kalibrasyonu seçilmiştir
                if (button_push_time_difference >= 1000000 && button_push_time_difference <= 2000000)
                {
                    // diğer görevleri silebiliriz. Çalışmalarına gerek yok.
                    // timer1 durdurulur ve silinir.
                    esp_timer_stop(timer1);
                    esp_timer_delete(timer1);
                    if (task1_handler != NULL) vTaskDelete(task1_handler);
                    if (task3_handler != NULL) vTaskDelete(task3_handler);
                    #if SETUP_GNSS_TYPE != GNSS_NONE
                    if (task5_handler != NULL) vTaskDelete(task5_handler);
                    #endif
                    #if SETUP_COMM_TYPE == USE_RC_LINK
                    if (task6_handler != NULL) vTaskDelete(task6_handler);
                    if (task7_handler != NULL) vTaskDelete(task7_handler);
                    #endif
                    #if SETUP_OPT_FLOW_TYPE != OPT_FLOW_NONE
                    if (task8_handler != NULL) vTaskDelete(task8_handler);
                    #endif
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
                // Bu süre (button_push_time_difference) 2 saniyeden kısa ise ivme ölçer kalibrasyonu seçilmiştir (1000000 us = 1 sn)
                if (button_push_time_difference > 2000000)
                {
                    // diğer görevleri silebiliriz. Çalışmalarına gerek yok.
                    // timer1 durdurulur ve silinir.
                    esp_timer_stop(timer1);
                    esp_timer_delete(timer1);
                    if (task1_handler != NULL) vTaskDelete(task1_handler);
                    if (task3_handler != NULL) vTaskDelete(task3_handler);
                    #if SETUP_GNSS_TYPE != GNSS_NONE
                    if (task5_handler != NULL) vTaskDelete(task5_handler);
                    #endif
                    #if SETUP_COMM_TYPE == USE_RC_LINK
                    if (task6_handler != NULL) vTaskDelete(task6_handler);
                    if (task7_handler != NULL) vTaskDelete(task7_handler);
                    #endif
                    #if SETUP_OPT_FLOW_TYPE != OPT_FLOW_NONE
                    if (task8_handler != NULL) vTaskDelete(task8_handler);
                    #endif
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
    // GNSS alıcısı seçildiyse başlat
    gnss_init();
    while (1)
    {
        // GNSS alıcısından UART üzerinden veri oku
        // bu görev için vTaskDelay kullanımına gerek yok
        // UART fonksiyonları zaten timeout süresi kadar burayı blokluyor
        gnss_read(&gnss);
        //printf("%d\n", gnss.hdop);
    }
}
#endif

#if SETUP_COMM_TYPE == USE_RC_LINK
void task_6(void *pvParameters)
{
    static uint8_t flg2;
    esp_now_comm_init(&config, &waypoint, &flg2, &telemetry, &flight, &states, &imu, &mag, &barometer, &gnss, &flow, &range, &target, &gamepad);

    while (1)
    {
        esp_now_send_telemetry();
        vTaskDelay(100);
    }
}

void task_7(void *pvParameters)
{
    ibus_init();
    while (1)
    {
        ibus_receiver_read(&radio);
    }
}
#endif

#if SETUP_OPT_FLOW_TYPE == OPT_FLOW_PMW3901
void task_8(void *pvParameters)
{
    pmw3901_init();
    while (1)
    {
        pmw3901_read(&flow);
    }
}
#endif

void app_main(void)
{
    //nvs_flash_erase();//  (WIFI ağı görünmüyorsa bir defa bu satırı çalıştır)
    // Non Volatile Storage birimini başlatır.
    nvs_flash_init();
    // Varsa önceden kaydedilmiş kalibrasyon verilerini ve konfigürasyonu oku. Yoksa default değerler ile başlat
    if (!storage_read(&mag_calibration_data, MAG_CALIB_DATA)) reset_calibration_data(&mag_calibration_data);
    if (!storage_read(&accel_calibration_data, ACCEL_CALIB_DATA)) reset_calibration_data(&accel_calibration_data);
    if (!storage_read(&config, CONFIG_DATA)) load_default_config(&config);
    #if SETUP_COMM_TYPE == USE_RC_LINK
    storage_read(&waypoint, MISSION_DATA);
    #endif
    #if SETUP_BLACKBOX == true
    blackbox_init(&flight, &imu);
    #endif
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
    // Komut satırı arayüzünü başlatır (UART0 kullanılıyorken bu işlev çakışmaya neden oluyor)
    // cli_begin(&config, &accel_calibration_data, &mag_calibration_data, &imu);
    // Gyro kalibrasyon prosedürünü başlat. Diğer görevler gyro kalibrasyonu tamamlandığında başlatılır.
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
}
