#ifndef GPIO_H
#define GPIO_H
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "typedefs.h"

#define DEFAULT_VSENS_CHANNEL ADC_CHANNEL_4 //GPIO_NUM_5
#define DSHOT_ESC_RESOLUTION_HZ 40000000

#define S1_PIN 16
#define S2_PIN 4
#define S3_PIN 38
#define S4_PIN 1
#define LED_PIN 2
#define BUTTON_PIN 0
#define VSENS_PIN 5

void gpio_configure(config_t *cfg);
void set_throttle_quadcopter(uint16_t mot1_thr, uint16_t mot2_thr, uint16_t mot3_thr, uint16_t mot4_thr);
void get_battery_voltage(float *voltage);
void status_led_set_brightness(uint8_t percent);
void status_led_breathe(uint8_t speed);
void set_four_channel_plane(uint16_t throttle, uint16_t aileron, uint16_t elevator, uint16_t rudder);
#endif