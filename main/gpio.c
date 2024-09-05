#include "gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "i2c.h"
#include "driver/i2c.h"
#include "spi.h"
#include "setup.h"
#include "typedefs.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

static adc_cali_handle_t adc1_cali_chan4_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static config_t *config_ptr = NULL;

static adc_channel_t adc_gpio_to_channel(gpio_num_t gpio_num);
static adc_channel_t VSENS_CHANNEL = DEFAULT_VSENS_CHANNEL;


#if SETUP_MOTOR_TYPE == MOTOR_CORELESS || SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM || SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE

static ledc_timer_config_t ledc_timer0 = 
{
    #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM || SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
        .duty_resolution = LEDC_TIMER_11_BIT,
        #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM
            .freq_hz = 490,
        #elif SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
            .freq_hz = 50,
        #endif
    #elif SETUP_MOTOR_TYPE == MOTOR_CORELESS
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 50000,
    #endif
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
static ledc_timer_config_t ledc_timer1 = 
{
    #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM || SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
        .duty_resolution = LEDC_TIMER_11_BIT,
        #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM
            .freq_hz = 490,
        #elif SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
            .freq_hz = 50,
        #endif
    #elif SETUP_MOTOR_TYPE == MOTOR_CORELESS
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 50000,
    #endif
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
static ledc_timer_config_t ledc_timer2 = 
{
    #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM || SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
        .duty_resolution = LEDC_TIMER_11_BIT,
        #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM
            .freq_hz = 490,
        #elif SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
            .freq_hz = 50,
        #endif
    #elif SETUP_MOTOR_TYPE == MOTOR_CORELESS
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 50000,
    #endif
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
static ledc_timer_config_t ledc_timer3 = 
{
    #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM || SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
        .duty_resolution = LEDC_TIMER_11_BIT,
        #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM
            .freq_hz = 490,
        #elif SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
            .freq_hz = 50,
        #endif
    #elif SETUP_MOTOR_TYPE == MOTOR_CORELESS
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 50000,
    #endif
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};

static ledc_channel_config_t ledc_channel0 = 
{
    .gpio_num = S1_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};
static ledc_channel_config_t ledc_channel1 = 
{
    .gpio_num = S2_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};
static ledc_channel_config_t ledc_channel2 = 
{
    .gpio_num = S3_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};
static ledc_channel_config_t ledc_channel3 = 
{
    .gpio_num = S4_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_3,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

#elif SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT300 || SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT600

static rmt_channel_handle_t esc_channel_1 = NULL;
static rmt_channel_handle_t esc_channel_2 = NULL;
static rmt_channel_handle_t esc_channel_3 = NULL;
static rmt_channel_handle_t esc_channel_4 = NULL;

static rmt_encoder_handle_t dshot_encoder_1 = NULL;
static rmt_encoder_handle_t dshot_encoder_2 = NULL;
static rmt_encoder_handle_t dshot_encoder_3 = NULL;
static rmt_encoder_handle_t dshot_encoder_4 = NULL;

static dshot_esc_throttle_t throttle_1;
static dshot_esc_throttle_t throttle_2;
static dshot_esc_throttle_t throttle_3;
static dshot_esc_throttle_t throttle_4;

static rmt_transmit_config_t tx_config = 
{
    .loop_count = -1, // infinite loop
};

#endif


static ledc_timer_config_t ledc_status_led_timer = 
{
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 1000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_1
};
static ledc_channel_config_t ledc_status_led_channel = 
{
    .gpio_num = LED_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_4,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0
};

static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten);
static void status_led_setup();
static void voltage_sens_adc_setup();
static void boot_button_interrupt_setup();
#if SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER
static void quadcopter_motor_setup();
#elif SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
static void plane_four_channel_setup();
#endif
// GPIO pinlerinin konfigürasyonunu yapar
void gpio_configure(config_t *cfg)
{
    config_ptr = cfg;
    #if SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER
    quadcopter_motor_setup();
    #elif SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE
    plane_four_channel_setup();
    #endif
    status_led_setup();
    voltage_sens_adc_setup();
    boot_button_interrupt_setup();
    i2c_master_init(I2C_NUM_0, SDA1, SCL1, I2C_HIGH_SPEED, GPIO_PULLUP_DISABLE);
    spi_master_init(MISO_PIN, MOSI_PIN, CLK_PIN);
    status_led_set_brightness(100);
}



#if SETUP_CRAFT_TYPE == CRAFT_TYPE_QUADCOPTER
// Çekirdeksiz DC motorların hızını ayarlar. (Min: 0 , Max: 1024)
void set_throttle_quadcopter(uint16_t mot1_thr, uint16_t mot2_thr, uint16_t mot3_thr, uint16_t mot4_thr)
{
    #if SETUP_MOTOR_TYPE == MOTOR_CORELESS

    ledc_set_duty(ledc_timer0.speed_mode, ledc_channel0.channel, mot1_thr);
    ledc_update_duty(ledc_timer0.speed_mode, ledc_channel0.channel);

    ledc_set_duty(ledc_timer1.speed_mode, ledc_channel1.channel, mot2_thr);
    ledc_update_duty(ledc_timer1.speed_mode, ledc_channel1.channel);

    ledc_set_duty(ledc_timer2.speed_mode, ledc_channel2.channel, mot3_thr);
    ledc_update_duty(ledc_timer2.speed_mode, ledc_channel2.channel);

    ledc_set_duty(ledc_timer3.speed_mode, ledc_channel3.channel, mot4_thr);
    ledc_update_duty(ledc_timer3.speed_mode, ledc_channel3.channel);

    #elif SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM
    // komut aralığı 0 -- 1000 ancak PWM 1005 -- 2005 aralığı istiyor (11bit)
    ledc_set_duty(ledc_timer0.speed_mode, ledc_channel0.channel, mot1_thr + 1005);
    ledc_update_duty(ledc_timer0.speed_mode, ledc_channel0.channel);

    ledc_set_duty(ledc_timer1.speed_mode, ledc_channel1.channel, mot2_thr + 1005);
    ledc_update_duty(ledc_timer1.speed_mode, ledc_channel1.channel);

    ledc_set_duty(ledc_timer2.speed_mode, ledc_channel2.channel, mot3_thr + 1005);
    ledc_update_duty(ledc_timer2.speed_mode, ledc_channel2.channel);

    ledc_set_duty(ledc_timer3.speed_mode, ledc_channel3.channel, mot4_thr + 1005);
    ledc_update_duty(ledc_timer3.speed_mode, ledc_channel3.channel);

    #elif SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT300 || SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT600

    // komut aralığı 0 -- 1000 ancak esc 0 -- 2000 aralığı istiyor
    throttle_1.throttle = mot1_thr * 2.0f;
    throttle_2.throttle = mot2_thr * 2.0f;
    throttle_3.throttle = mot3_thr * 2.0f;
    throttle_4.throttle = mot4_thr * 2.0f;
    
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_1, dshot_encoder_1, &throttle_1, sizeof(throttle_1), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_2, dshot_encoder_2, &throttle_2, sizeof(throttle_2), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_3, dshot_encoder_3, &throttle_3, sizeof(throttle_3), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_4, dshot_encoder_4, &throttle_4, sizeof(throttle_4), &tx_config));

    ESP_ERROR_CHECK(rmt_disable(esc_channel_1));
    ESP_ERROR_CHECK(rmt_disable(esc_channel_2));
    ESP_ERROR_CHECK(rmt_disable(esc_channel_3));
    ESP_ERROR_CHECK(rmt_disable(esc_channel_4));

    ESP_ERROR_CHECK(rmt_enable(esc_channel_1));
    ESP_ERROR_CHECK(rmt_enable(esc_channel_2));
    ESP_ERROR_CHECK(rmt_enable(esc_channel_3));
    ESP_ERROR_CHECK(rmt_enable(esc_channel_4));

    #endif
}

static void quadcopter_motor_setup()
{
    #if SETUP_MOTOR_TYPE == MOTOR_CORELESS || SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_PWM

    // 4 motor için timer kurulumu
    ledc_timer_config(&ledc_timer0);
    ledc_timer_config(&ledc_timer1);
    ledc_timer_config(&ledc_timer2);
    ledc_timer_config(&ledc_timer3);
    // 4 motor için channel kurulumu
    ledc_channel_config(&ledc_channel0);
    ledc_channel_config(&ledc_channel1);
    ledc_channel_config(&ledc_channel2);
    ledc_channel_config(&ledc_channel3);

    #elif SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT300 || SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT600

    throttle_1.throttle = 0;
    throttle_1.telemetry_req = false;

    throttle_2.throttle = 0;
    throttle_2.telemetry_req = false;

    throttle_3.throttle = 0;
    throttle_3.telemetry_req = false;

    throttle_4.throttle = 0;
    throttle_4.telemetry_req = false;

    rmt_tx_channel_config_t tx_chan_config_1 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = S1_PIN,
        .mem_block_symbols = 48,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_tx_channel_config_t tx_chan_config_2 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = S2_PIN,
        .mem_block_symbols = 48,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_tx_channel_config_t tx_chan_config_3 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = S3_PIN,
        .mem_block_symbols = 48,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    rmt_tx_channel_config_t tx_chan_config_4 = 
    {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = S4_PIN,
        .mem_block_symbols = 48,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_1, &esc_channel_1));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_2, &esc_channel_2));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_3, &esc_channel_3));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_4, &esc_channel_4));


    dshot_esc_encoder_config_t encoder_config = 
    {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        #if SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT600
        .baud_rate = 600000, // DSHOT600 protocol
        #elif  SETUP_MOTOR_TYPE == MOTOR_BRUSHLESS_DSHOT300
        .baud_rate = 300000, // DSHOT300 protocol
        #endif
        .post_delay_us = 50, // extra delay between each frame
    };
    

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_1));
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_2));
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_3));
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder_4));

    ESP_ERROR_CHECK(rmt_enable(esc_channel_1));
    ESP_ERROR_CHECK(rmt_enable(esc_channel_2));
    ESP_ERROR_CHECK(rmt_enable(esc_channel_3));
    ESP_ERROR_CHECK(rmt_enable(esc_channel_4));

    ESP_ERROR_CHECK(rmt_transmit(esc_channel_1, dshot_encoder_1, &throttle_1, sizeof(throttle_1), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_2, dshot_encoder_2, &throttle_2, sizeof(throttle_2), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_3, dshot_encoder_3, &throttle_3, sizeof(throttle_3), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(esc_channel_4, dshot_encoder_4, &throttle_4, sizeof(throttle_4), &tx_config));

    #endif

    // Başlangıçta motor hızları 0 olsun
    set_throttle_quadcopter(0, 0, 0, 0);
}


#endif
#if SETUP_CRAFT_TYPE == CRAFT_TYPE_PLANE

void set_four_channel_plane(uint16_t throttle, uint16_t aileron, uint16_t elevator, uint16_t rudder)
{
    // komut aralığı 0 -- 1000 ancak PWM 1005 -- 2005 aralığı istiyor (11bit)
    ledc_set_duty(ledc_timer0.speed_mode, ledc_channel0.channel, (throttle / 9) + 99);
    ledc_update_duty(ledc_timer0.speed_mode, ledc_channel0.channel);

    ledc_set_duty(ledc_timer1.speed_mode, ledc_channel1.channel, (aileron / 9) + 99);
    ledc_update_duty(ledc_timer1.speed_mode, ledc_channel1.channel);

    ledc_set_duty(ledc_timer2.speed_mode, ledc_channel2.channel, (elevator / 9) + 99);
    ledc_update_duty(ledc_timer2.speed_mode, ledc_channel2.channel);

    ledc_set_duty(ledc_timer3.speed_mode, ledc_channel3.channel, (rudder / 9) + 99);
    ledc_update_duty(ledc_timer3.speed_mode, ledc_channel3.channel);
}

static void plane_four_channel_setup()
{
    // 4 motor için timer kurulumu
    ledc_timer_config(&ledc_timer0);
    ledc_timer_config(&ledc_timer1);
    ledc_timer_config(&ledc_timer2);
    ledc_timer_config(&ledc_timer3);
    // 4 motor için channel kurulumu
    ledc_channel_config(&ledc_channel0);
    ledc_channel_config(&ledc_channel1);
    ledc_channel_config(&ledc_channel2);
    ledc_channel_config(&ledc_channel3);

    set_four_channel_plane(0, 500, 500, 500);
}
#endif




// LED'in parlaklığını yüzde cinsinden ayarlar
void status_led_set_brightness(uint8_t percent)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, percent * 10);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}
// Durum LED'inin belirli bir hızda parlaklığını arttırıp azaltır.
// Hız değeri 100'ün altında olmalıdır.
void status_led_breathe(uint8_t speed)
{
    static uint8_t value = 0;    // Başlangıç değeri
    static uint8_t increment = 1; // Artış miktarı
    value += increment * speed; // Değeri artır veya azalt

    // Değer 100 olduğunda veya 0 olduğunda artış yönünü değiştir
    if (value >= 100) 
    {
        increment = -1;
        value = 100;
    }
    else if (value <= 0) 
    {
        increment = 1;
        value = 0;
    }
    status_led_set_brightness(value);
}

// Durum LED'inin kurulumu
static void status_led_setup()
{
    ledc_timer_config(&ledc_status_led_timer);
    ledc_channel_config(&ledc_status_led_channel);
}

static void voltage_sens_adc_setup()
{
    VSENS_CHANNEL = adc_gpio_to_channel(VSENS_PIN);
    adc_oneshot_unit_init_cfg_t init_config1 = 
    {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = 
    {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc1_handle, VSENS_CHANNEL, &config);
    adc_calibration_init(ADC_UNIT_1, VSENS_CHANNEL, ADC_ATTEN_DB_11);
}

static void boot_button_interrupt_setup()
{
    // GPIO_0'a bağlı kesim (interrupt) kurulumu
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Kesim hem yükselen hem düşen kenarda aktif
    io_conf.mode = GPIO_MODE_INPUT;        // Girdi olarak ayarla
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    // ISR Hendler kurulumu (intr_alloc_flags == 0)
    gpio_install_isr_service(0);
}

static adc_channel_t adc_gpio_to_channel(gpio_num_t gpio_num) 
{
    if (gpio_num == GPIO_NUM_0 || gpio_num > GPIO_NUM_10) 
    {
        printf("Voltage sens for GPIO_%d not available. Available GPIO pins 1 to 10\n", gpio_num);
        printf("Using default_vsens_pin (GPIO_5)\n");
        return DEFAULT_VSENS_CHANNEL;
    }
    return gpio_num - 1; // GPIO_1 den GPIO_10' a ADC_CHANNEL_0' dan ADC_CHANNEL_9' a karşılık geliyor.
}
static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
    adc_cali_curve_fitting_config_t cali_config = 
    {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan4_handle);
}

// Batarya gerilimini döndürür
void get_battery_voltage(float *voltage)
{
    static int adc1_ch4_raw = 0;
    static int volt_raw = 0;
    static uint8_t init = 1;
    adc_oneshot_read(adc1_handle, VSENS_CHANNEL, &adc1_ch4_raw);
    adc_cali_raw_to_voltage(adc1_cali_chan4_handle, adc1_ch4_raw, &volt_raw);
    if (init) 
    {
        *voltage = ((float)(volt_raw / 1000.0f)) * config_ptr->voltage_gain;
        init = 0;
    }
    *voltage = ((float)(volt_raw / 1000.0f)) * config_ptr->voltage_gain * 0.01f + *voltage * 0.99f;
}