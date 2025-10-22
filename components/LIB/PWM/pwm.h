#pragma once

#include "driver/ledc.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_DEFAULT_FREQ_HZ      (5000)                  // 5kHz, align with V4Refer
#define PWM_DEFAULT_RESOLUTION   (LEDC_TIMER_13_BIT)     // 13-bit resolution, align with V4Refer

typedef struct {
    int gpio_num;                     // output pin
    ledc_mode_t mode;                 // LEDC_HIGH_SPEED_MODE / LEDC_LOW_SPEED_MODE
    ledc_channel_t channel;           // LEDC_CHANNEL_0 ..
    ledc_timer_t timer;               // LEDC_TIMER_0 ..
    uint32_t freq_hz;                 // configured frequency
    ledc_timer_bit_t resolution;      // duty resolution
    uint32_t max_duty;                // (1<<resolution) - 1
    bool configured;                  // true after successful config
} pwm_channel_t;

/**
 * Configure an LEDC timer. Safe to call multiple times for the same timer.
 */
esp_err_t pwm_timer_config(ledc_timer_t timer,
                           ledc_mode_t mode,
                           uint32_t freq_hz,
                           ledc_timer_bit_t resolution);

/**
 * Configure one PWM channel and bind it to a GPIO. This will call pwm_timer_config() internally.
 */
esp_err_t pwm_channel_config(pwm_channel_t *ch,
                             int gpio_num,
                             ledc_channel_t channel,
                             ledc_timer_t timer,
                             ledc_mode_t mode,
                             uint32_t freq_hz,
                             ledc_timer_bit_t resolution);

/**
 * Set raw duty (0..max_duty). Will update the duty immediately.
 */
esp_err_t pwm_set_duty(pwm_channel_t *ch, uint32_t duty);

/**
 * Set duty by percentage 0..100. Values outside will be clamped.
 */
esp_err_t pwm_set_duty_percent(pwm_channel_t *ch, float percent);

/**
 * Stop PWM output (duty=0).
 */
esp_err_t pwm_stop(pwm_channel_t *ch);

/**
 * Query max duty for the channel.
 */
uint32_t pwm_max_duty(const pwm_channel_t *ch);

#ifdef __cplusplus
}
#endif