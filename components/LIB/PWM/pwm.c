#include "pwm.h"
#include "esp_log.h"

static const char *TAG = "LIB_PWM";

esp_err_t pwm_timer_config(ledc_timer_t timer,
                           ledc_mode_t mode,
                           uint32_t freq_hz,
                           ledc_timer_bit_t resolution)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = mode,
        .duty_resolution = resolution,
        .timer_num = timer,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", err);
    }
    return err;
}

esp_err_t pwm_channel_config(pwm_channel_t *ch,
                             int gpio_num,
                             ledc_channel_t channel,
                             ledc_timer_t timer,
                             ledc_mode_t mode,
                             uint32_t freq_hz,
                             ledc_timer_bit_t resolution)
{
    if (!ch) return ESP_ERR_INVALID_ARG;

    esp_err_t err = pwm_timer_config(timer, mode, freq_hz, resolution);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ccfg = {
        .gpio_num = gpio_num,
        .speed_mode = mode,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,           // start at 0 duty
        .hpoint = 0,
    };

    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %d", err);
        return err;
    }

    ch->gpio_num = gpio_num;
    ch->mode = mode;
    ch->channel = channel;
    ch->timer = timer;
    ch->freq_hz = freq_hz;
    ch->resolution = resolution;
    ch->max_duty = (1U << resolution) - 1U;
    ch->configured = true;

    ESP_LOGI(TAG, "PWM ch=%d gpio=%d freq=%luHz res=%d-bit (max_duty=%lu)",
             channel, gpio_num, (unsigned long)freq_hz, (int)resolution, (unsigned long)ch->max_duty);

    return ESP_OK;
}

esp_err_t pwm_set_duty(pwm_channel_t *ch, uint32_t duty)
{
    if (!ch || !ch->configured) return ESP_ERR_INVALID_STATE;
    if (duty > ch->max_duty) duty = ch->max_duty;

    esp_err_t err = ledc_set_duty(ch->mode, ch->channel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: %d", err);
        return err;
    }
    err = ledc_update_duty(ch->mode, ch->channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: %d", err);
        return err;
    }
    return ESP_OK;
}

esp_err_t pwm_set_duty_percent(pwm_channel_t *ch, float percent)
{
    if (!ch || !ch->configured) return ESP_ERR_INVALID_STATE;
    if (percent < 0.f) percent = 0.f;
    if (percent > 100.f) percent = 100.f;
    uint32_t duty = (uint32_t)((percent / 100.f) * (float)ch->max_duty + 0.5f);
    return pwm_set_duty(ch, duty);
}

esp_err_t pwm_stop(pwm_channel_t *ch)
{
    return pwm_set_duty(ch, 0);
}

uint32_t pwm_max_duty(const pwm_channel_t *ch)
{
    return ch ? ch->max_duty : 0;
}