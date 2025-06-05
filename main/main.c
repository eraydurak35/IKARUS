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
#include "mavlink/Ikarus_messages/mavlink.h"
#include "comminication/streams/stream.h"
#include "control/quadcopter_control.h"
#include "comminication/esp_now_comm.h"
#include "comminication/web_comm.h"
#include "command_line_interface.h"
#include "storage/nv_storage.h"
#include "sensors/icm42688p.h"
#include "sensors/qmc5883l.h"
#include "sensors/hmc5883l.h"
#include "storage/blackbox.h"
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
#include "sbus.h"
#include "gpio.h"
#include "hitl.h"
#include "../components/ecl/ecl_c_wrapper.h"
#include "parameters/param.h"

static esp_timer_handle_t timer1;
static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static TaskHandle_t task4_handler;
#if SETUP_GNSS_TYPE != GNSS_NONE
static TaskHandle_t task5_handler;
static TaskHandle_t task9_handler;
#endif
#if SETUP_COMM_TYPE == USE_RC_LINK
static TaskHandle_t task6_handler;
static TaskHandle_t task7_handler;
#endif
#if SETUP_OPT_FLOW_TYPE != OPT_FLOW_NONE
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
static cpu_usage_t cpu_usage;

#if SETUP_COMM_TYPE == USE_WEBCOMM
static telemetry_small_integer_t telemetry;
#else 
static telemetry_t telemetry;
static waypoint_t waypoint;
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
    xTaskNotifyFromISR(task4_handler, gpio_get_level(SETUP_BUTTON_PIN), eSetValueWithOverwrite, false);
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

void parse_cpu_usage(char* statsBuffer, cpu_usage_t* cpuUsage);

// Ana görev
void task_1(void *pvParameters)
{
    #if SETUP_COMM_TYPE == USE_WEBCOMM
    web_comm_init(&radio, &states, &flight, &telemetry);
    #elif SETUP_COMM_TYPE == USE_RC_LINK
    quadcopter_control_init(&radio, &telemetry, &flight, &target, &states, &config, &waypoint, &gnss);
    xTaskCreatePinnedToCore(&task_6, "task6", 1024 * 4, NULL, 0, &task6_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_7, "task7", 1024 * 4, NULL, 1, &task7_handler, tskNO_AFFINITY);
    #endif
    #if SETUP_OPT_FLOW_TYPE == OPT_FLOW_PMW3901
    xTaskCreatePinnedToCore(&task_8, "task8", 1024 * 4, NULL, 0, &task8_handler, tskNO_AFFINITY);
    #endif
    // 3 ivme 3 gyro için lowpass yapısını başlat
    if (config.lpf_cutoff_hz == 0.0f) {
        biquad_lpf_array_init(6, lowpass, DFLT_LPF_CUTOFF_HZ, SETUP_MAIN_LOOP_FREQ_HZ);
    } else {
        biquad_lpf_array_init(6, lowpass, config.lpf_cutoff_hz, SETUP_MAIN_LOOP_FREQ_HZ);
    }
    // F450'nin tepe gürültüsü 72Hz bant genişliği 50Hz
    // Fırçalı dronun tepe gürültüsü 238.0Hz bant genişliği 45Hz
    biquad_notch_filter_array_init(6, notch, 262.0f, 45.0f, SETUP_MAIN_LOOP_FREQ_HZ);
    barometer.gnd_press = 1013.15f;
    #if SETUP_ENABLE_HITL == false
    // Kestirim algoritmasını başlatmadan önce filtrelerin buffer'ını doldur.
    for (uint8_t i = 0; i <= 100; i++)
    {
        icm42688p_read(&imu);
        apply_biquad_lpf_to_imu(&imu, lowpass);
        apply_biquad_notch_filter_to_imu(&imu, notch);
        vTaskDelay(2);
    }
    #else
    vTaskDelay(500);
    hitl_get_sensors(&imu, &mag, &barometer);
    #endif
    if (ekf_init(esp_timer_get_time()) != 1)
    {
        printf("EKF initialization failed!\n");
        while (1) {
            vTaskDelay(1000);
        }
    }
    static uint32_t receivedValue = 0;
    while (1)
    {
        // vTaskDelay kullanmadan timer interrup ile tetiklenerek periyodik olarak çalışan fonksiyon
        // burası her 1ms de bir çalışacak.
        if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            // IMU verilerini oku
            #if SETUP_ENABLE_HITL == false
            icm42688p_read(&imu);
            #else
            hitl_get_sensors(&imu, &mag, &barometer);
            static uint8_t counter = 0;
            counter++;
            if (counter >= 20){
                counter = 0;
                baro_get_altitude_velocity(&barometer);
            }
            #endif
            // IMU verilerini alçak geçiren filtreden geçir.
            apply_biquad_lpf_to_imu(&imu, lowpass);
            // IMU verilerini notch filtreden geçir.
            apply_biquad_notch_filter_to_imu(&imu, notch);
            #if SETUP_USE_BLACKBOX == true
            // Bu fonksiyon, imu filtrelenmeden önce kaydedilecekse filtreden önce çağırılmalıdır.
            blackbox_save();
            #endif

            static uint32_t imu_counter = 0;
            static float gyx_sum = 0.0f;
            static float gyy_sum = 0.0f;
            static float gyz_sum = 0.0f;
            static float acx_sum = 0.0f;
            static float acy_sum = 0.0f;
            static float acz_sum = 0.0f;
            static int64_t dt_us = 0;
            static int64_t prev_time_us = 0;
            static float dt_sec = 0.001f;

            dt_us = esp_timer_get_time() - prev_time_us;
            prev_time_us = esp_timer_get_time();
            dt_sec = (float)dt_us * 1e-6f;

            gyx_sum += imu.gyro_dps[X];
            gyy_sum += imu.gyro_dps[Y];
            gyz_sum += imu.gyro_dps[Z];
            acx_sum += imu.accel_ms2[X];
            acy_sum += imu.accel_ms2[Y];
            acz_sum += imu.accel_ms2[Z];

            if ((++imu_counter)%10 == 0)
            {
                gyx_sum /= 10.0f;
                gyy_sum /= 10.0f;
                gyz_sum /= 10.0f;
                acx_sum /= 10.0f;
                acy_sum /= 10.0f;
                acz_sum /= 10.0f;

                ekf_set_imu_data(gyx_sum, gyy_sum, gyz_sum, acx_sum, acy_sum, acz_sum, esp_timer_get_time());

                gyx_sum = 0.0f;
                gyy_sum = 0.0f;
                gyz_sum = 0.0f;
                acx_sum = 0.0f;
                acy_sum = 0.0f;
                acz_sum = 0.0f;

                ekf_update();

                states.roll_deg = ekf_get_roll_deg();
                states.pitch_deg = ekf_get_pitch_deg();
                states.heading_deg = ekf_get_heading_deg();
                states.altitude_m = -ekf_get_position_down();
                states.vel_up_ms = -ekf_get_velocity_down();
                states.is_attitude_valid = ekf_is_attitude_valid();
            }
            else
            {
                /* Kalmanı çağırmak masraflı olduğu için son bilgiye göre tahmin et */
                states.roll_deg += imu.gyro_dps[X] * dt_sec;
                states.pitch_deg += imu.gyro_dps[Y] * dt_sec;
                states.heading_deg += imu.gyro_dps[Z] * dt_sec;
            }

            states.roll_dps = imu.gyro_dps[X];
            states.pitch_dps = imu.gyro_dps[Y];
            states.yaw_dps = imu.gyro_dps[Z];
            

            #if SETUP_OPT_FLOW_TYPE != OPT_FLOW_NONE
            optical_flow_velocity_XY();
            #endif
            #if SETUP_COMM_TYPE == USE_RC_LINK && SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER
            quadcopter_flight_control();
            #endif
        }
    }
}


// Gyro kalibrasyon görevi
void task_2(void *pvParameters)
{
    // IMU'nun ayarlarını yap ve başlat
    icm42688p_setup(&accel_calibration_data);
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
            // Diğer sensörlerdenden veri okuyan görevi başlat
            xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 4, NULL, 1, &task3_handler, tskNO_AFFINITY);
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
    #if SETUP_MAGNETO_TYPE == MAG_QMC5883L
    // Manyetik sensörün ayarlarını yap ve başlat
    qmc5883l_setup(&mag_calibration_data);
    qmc5883l_read(&mag, 0);
    #elif SETUP_MAGNETO_TYPE == MAG_HMC5883L
    hmc5883l_setup(&mag_calibration_data);
    hmc5883l_read(&mag);
    #endif
    // bmp3xx sensörünün ayarlarını yap ve başlat
    bmp390_setup_spi();
    // Barometre geçerli veri üretene kadar bir süre bekle
    vTaskDelay(500);
    // İvme ölçer ve manyetik sensör kalibrasyon görevi. Öncelik değeri (Idle = 0) olarak ayarlı
    xTaskCreatePinnedToCore(&task_4, "task4", 1024 * 4, NULL, 0, &task4_handler, tskNO_AFFINITY);
    #if SETUP_GNSS_TYPE != GNSS_NONE
    // GNSS alıcısından veri okuyan görevi başlat
    xTaskCreatePinnedToCore(&task_5, "task5", 1024 * 4, NULL, 1, &task5_handler, tskNO_AFFINITY);
    #endif
    // Ana görevi başlatabiliriz
    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 8, NULL, 1, &task1_handler, tskNO_AFFINITY);
    // Timer interrupt kurulumu
    const esp_timer_create_args_t timer1_args =
    {
        .callback = &timer1_callback,
        .arg = NULL,
        .name = "timer1"
    };
    esp_timer_create(&timer1_args, &timer1);
    // SETUP_MAIN_LOOP_FREQ_HZ için timer başlat
    esp_timer_start_periodic(timer1, (uint64_t)(1000000.0f / SETUP_MAIN_LOOP_FREQ_HZ));
    puts("IKARUS");

    while (1)
    {
        get_battery_voltage(&flight.battery_voltage);
        // Manyetik sensör verisini oku
        #if SETUP_MAGNETO_TYPE == MAG_QMC5883L
        qmc5883l_read(&mag, 0);
        ekf_set_mag_data(mag.gauss[X], mag.gauss[Y], mag.gauss[Z], esp_timer_get_time());
        #elif SETUP_MAGNETO_TYPE == MAG_HMC5883L
        hmc5883l_read(&mag);
        #endif
        // Barometrik sensör verisini oku
        bmp390_read_spi(&barometer);
        ekf_set_baro_data(barometer.altitude_m, esp_timer_get_time());
        #if SETUP_LIDAR_TYPE == LIDAR_TF_LUNA
        tf_luna_read_range(&range, &states);
        #endif
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

                #if SETUP_USE_BLACKBOX == true
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
                        status_led_set_brightness(100);
                    }
                }
                #endif
                #if SETUP_MAGNETO_TYPE != MAG_NONE
                // Bu süre (button_push_time_difference) 1 ile 5 saniye arasında ise manyetik sensör kalibrasyonu seçilmiştir
                if (button_push_time_difference >= 1000000 && button_push_time_difference <= 5000000)
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
                    printf("Manyetik kalibrasyon tamamlandi\n");
                    esp_restart();
                }
                #endif
                // Bu süre (button_push_time_difference) 5 saniyeden kısa ise ivme ölçer kalibrasyonu seçilmiştir (1000000 us = 1 sn)
                if (button_push_time_difference > 5000000)
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
                    printf("İvme kalibrasyonu tamamlandi\n");
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
    char cpu_stats_buffer[512] = {0};
    uint8_t counter = 0;
    esp_now_comm_init();
    start_mavlink_stream(&config, &waypoint, &flight, &states, &imu, &mag, &barometer, &gnss, &flow, &range, &target, &cpu_usage, &radio);
    while (1)
    {
        counter++;
        run_mavlink_stream();

        if (counter > 100)
        {
            counter = 0;
            vTaskGetRunTimeStats(cpu_stats_buffer);
            //printf("%s\n", cpu_stats_buffer);
            parse_cpu_usage(cpu_stats_buffer, &cpu_usage);
            //printf("Core0: %d\nCore1: %d\n\n", cpu_usage.core0_percent, cpu_usage.core1_percent); // cpu0 -> 45% cpu1 -> 1%
        }
        vTaskDelay(10);
    }
}

void task_7(void *pvParameters)
{
    #if SETUP_RC_PROTOCOL == SERIAL_IBUS
    ibus_init();
    while (1)
    {
        ibus_receiver_read(&radio);
    }
    #elif SETUP_RC_PROTOCOL == SERIAL_SBUS
    sbus_init();
    while (1)
    {
        sbus_receiver_read(&radio);
    }
    #endif
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

#if SETUP_ENABLE_HITL == true
void task_9(void *pvParameters)
{
    static uint8_t first_msg_found = 0;
    static uint16_t counter = 0;
    while (1)
    {
        uint8_t ret = hitl_read();

        if (ret == 1 && first_msg_found == 0)
        {
            first_msg_found = 1;
            xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);
            // Timer interrupt kurulumu
            const esp_timer_create_args_t timer1_args =
            {
                .callback = &timer1_callback,
                .arg = NULL,
                .name = "timer1"
            };
            esp_timer_create(&timer1_args, &timer1);
            // SETUP_MAIN_LOOP_FREQ_HZ için timer başlat
            esp_timer_start_periodic(timer1, (uint64_t)(1000000.0f / SETUP_MAIN_LOOP_FREQ_HZ));
        }
        else
        {
            counter++;
            if (counter >= 500)
            {
                counter = 0;
                //printf("1\n");
                mavlink_send_heartbeat();
            }
        }
        vTaskDelay(2);
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
    #if SETUP_GNSS_TYPE != GNSS_NONE
    storage_read(&waypoint, MISSION_DATA);
    #endif
    #if SETUP_USE_BLACKBOX == true
    blackbox_init(&flight, &imu);
    #endif
    // GPIO pinlerini konfigüre et
    gpio_configure(&config);
    // Buton pinine interrupt service rouutine ekle
    gpio_isr_handler_add(SETUP_BUTTON_PIN, button_ISR, (void*) SETUP_BUTTON_PIN);
    // Komut satırı arayüzünü başlatır (UART0 kullanılıyorken bu işlev çakışmaya neden oluyor)
    #if SETUP_GNSS_TYPE == GNSS_NONE
    //cli_begin(&config, &accel_calibration_data, &mag_calibration_data, &imu);
    #endif
    #if SETUP_ENABLE_HITL == true
    xTaskCreatePinnedToCore(&task_9, "task9", 1024 * 4, NULL, 1, &task9_handler, tskNO_AFFINITY);
    #else
    // Gyro kalibrasyon prosedürünü başlat. Diğer görevler gyro kalibrasyonu tamamlandığında başlatılır.
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
    #endif
    
}


void parse_cpu_usage(char* buffer, cpu_usage_t* cpuUsage) 
{
    uint8_t idle_values[2] = {0};
    int idleCount = 0;
    char* line = strtok(buffer, "\n");

    while (line != NULL && idleCount < 2) {
        if (strstr(line, "IDLE") != NULL) {
            char cpuStr[16];
            if (sscanf(line, "%*s %*u %s", cpuStr) == 1) {
                // "%" işaretini kaldır ve tam sayıya çevir
                char* percentChar = strchr(cpuStr, '%');
                if (percentChar) *percentChar = '\0'; // Stringi sonlandır
                
                idle_values[idleCount] = 100 - (uint8_t)atoi(cpuStr);
                idleCount++;
            }
        }
        line = strtok(NULL, "\n");
    }

    // Büyük olanı core0_percent'e ata
    if (idle_values[0] >= idle_values[1]) {
        cpuUsage->core0_percent = idle_values[0];
        cpuUsage->core1_percent = idle_values[1];
    } else {
        cpuUsage->core0_percent = idle_values[1];
        cpuUsage->core1_percent = idle_values[0];
    }
}