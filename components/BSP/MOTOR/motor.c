#include "motor.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "BSP_MOTOR";

esp_err_t motor_init(motor_t *m,
                     int in1_gpio,
                     int in2_gpio,
                     ledc_channel_t in1_channel,
                     ledc_channel_t in2_channel,
                     ledc_timer_t timer,
                     ledc_mode_t mode,
                     uint32_t freq_hz,
                     ledc_timer_bit_t resolution)
{
    if (!m) return ESP_ERR_INVALID_ARG;

    // 配置两路PWM通道（共享同一个定时器/频率/分辨率）
    esp_err_t err = pwm_channel_config(&m->in1, in1_gpio, in1_channel, timer, mode, freq_hz, resolution);
    if (err != ESP_OK) return err;
    err = pwm_channel_config(&m->in2, in2_gpio, in2_channel, timer, mode, freq_hz, resolution);
    if (err != ESP_OK) return err;

    // 默认空转
    pwm_stop(&m->in1);
    pwm_stop(&m->in2);

    m->initialized = true;
    m->drive_mode = MOTOR_DRIVE_COAST; // 默认使用快衰减（COAST），对齐 V4Refer DRV8833 风格

    ESP_LOGI(TAG, "motor init ok: in1_gpio=%d ch=%d, in2_gpio=%d ch=%d, freq=%luHz res=%d-bit",
             in1_gpio, in1_channel, in2_gpio, in2_channel, (unsigned long)freq_hz, (int)resolution);
    return ESP_OK;
}

// 新增：设置驱动模式（BRAKE/COAST）
void motor_set_drive_mode(motor_t *m, motor_drive_mode_t mode)
{
    if (!m) return;
    m->drive_mode = mode;
}

esp_err_t motor_set_permille(motor_t *m, int permille)
{
    if (!m || !m->initialized) return ESP_ERR_INVALID_STATE;

    // 限幅到 -1000..1000
    if (permille > 1000) permille = 1000;
    if (permille < -1000) permille = -1000;

    uint32_t maxduty = pwm_max_duty(&m->in1); // 两路分辨率一致，任选一侧
    uint32_t duty = (uint32_t)((abs(permille) / 1000.0f) * (float)maxduty + 0.5f);

    if (permille > 0) {
        // 正转
        if (m->drive_mode == MOTOR_DRIVE_BRAKE) {
            // 慢衰减（驱动/刹车）：IN1=100%，IN2=PWM(高=刹车，低=驱动)
            pwm_set_duty(&m->in1, maxduty);
            return pwm_set_duty(&m->in2, maxduty - duty);
        } else {
            // 快衰减（驱动/空转）：IN1=PWM（驱动），IN2=0（空转） —— 对齐 V4Refer
            pwm_stop(&m->in2);
            return pwm_set_duty(&m->in1, duty);
        }
    } else if (permille < 0) {
        // 反转
        if (m->drive_mode == MOTOR_DRIVE_BRAKE) {
            // 慢衰减：IN2=100%，IN1=PWM(高=刹车，低=驱动)
            pwm_set_duty(&m->in2, maxduty);
            return pwm_set_duty(&m->in1, maxduty - duty);
        } else {
            // 快衰减：IN1=0（空转），IN2=PWM（驱动） —— 对齐 V4Refer
            pwm_stop(&m->in1);
            return pwm_set_duty(&m->in2, duty);
        }
    } else {
        // 停止（空转）
        return motor_stop_freewheel(m);
    }
}

esp_err_t motor_stop_freewheel(motor_t *m)
{
    if (!m || !m->initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t err = pwm_stop(&m->in1);
    if (err != ESP_OK) return err;
    err = pwm_stop(&m->in2);
    return err;
}

esp_err_t motor_brake(motor_t *m)
{
    if (!m || !m->initialized) return ESP_ERR_INVALID_STATE;
    uint32_t maxduty = pwm_max_duty(&m->in1);
    // 电子刹车：两侧全导通（对H桥等效短路），快速衰减
    esp_err_t err = pwm_set_duty(&m->in1, maxduty);
    if (err != ESP_OK) return err;
    err = pwm_set_duty(&m->in2, maxduty);
    return err;
}

/*
 * 统一小车控制接口（框架）：基于四轮电机与极性
 */
static inline int clamp_permille_local(int p)
{
    if (p > 1000) return 1000;
    if (p < -1000) return -1000;
    return p;
}

static inline int scale_permille_local(int p, int factor_permille)
{
    long v = (long)p * (long)factor_permille;
    return (int)(v / 1000);
}

void robot_init_drive(robot_drive_t *rb,
                      motor_t *fl, motor_t *fr, motor_t *rl, motor_t *rr,
                      int pol_fl, int pol_fr, int pol_rl, int pol_rr)
{
    if (!rb) return;
    rb->fl = fl;
    rb->fr = fr;
    rb->rl = rl;
    rb->rr = rr;
    rb->pol_fl = pol_fl;
    rb->pol_fr = pol_fr;
    rb->pol_rl = pol_rl;
    rb->pol_rr = pol_rr;
    rb->turn_slow_factor_permille = 500; // 默认 50%
}

void robot_set_turn_slow_factor(robot_drive_t *rb, int factor_permille)
{
    if (!rb) return;
    if (factor_permille < 0) factor_permille = 0;
    if (factor_permille > 1000) factor_permille = 1000;
    rb->turn_slow_factor_permille = factor_permille;
}

void robot_wheels_set_permille(robot_drive_t *rb, int permille)
{
    if (!rb || !rb->fl || !rb->fr || !rb->rl || !rb->rr) return;
    int p = clamp_permille_local(permille);
    motor_set_permille(rb->fl, rb->pol_fl * p);
    motor_set_permille(rb->fr, rb->pol_fr * p);
    motor_set_permille(rb->rl, rb->pol_rl * p);
    motor_set_permille(rb->rr, rb->pol_rr * p);
}

void robot_drive_forward(robot_drive_t *rb, int permille)
{
    robot_wheels_set_permille(rb, permille);
}

void robot_drive_backward(robot_drive_t *rb, int permille)
{
    robot_wheels_set_permille(rb, -permille);
}

void robot_turn_left(robot_drive_t *rb, int permille)
{
    if (!rb || !rb->fl || !rb->fr || !rb->rl || !rb->rr) return;
    int p = clamp_permille_local(permille);
    int p_slow = scale_permille_local(p, rb->turn_slow_factor_permille);
    motor_set_permille(rb->fl, rb->pol_fl * p_slow);
    motor_set_permille(rb->rl, rb->pol_rl * p_slow);
    motor_set_permille(rb->fr, rb->pol_fr * p);
    motor_set_permille(rb->rr, rb->pol_rr * p);
}

void robot_turn_right(robot_drive_t *rb, int permille)
{
    if (!rb || !rb->fl || !rb->fr || !rb->rl || !rb->rr) return;
    int p = clamp_permille_local(permille);
    int p_slow = scale_permille_local(p, rb->turn_slow_factor_permille);
    motor_set_permille(rb->fl, rb->pol_fl * p);
    motor_set_permille(rb->rl, rb->pol_rl * p);
    motor_set_permille(rb->fr, rb->pol_fr * p_slow);
    motor_set_permille(rb->rr, rb->pol_rr * p_slow);
}

void robot_rotate_in_place(robot_drive_t *rb, int permille)
{
    if (!rb || !rb->fl || !rb->fr || !rb->rl || !rb->rr) return;
    int p = clamp_permille_local(permille);
    motor_set_permille(rb->fl, rb->pol_fl * -p);
    motor_set_permille(rb->rl, rb->pol_rl * -p);
    motor_set_permille(rb->fr, rb->pol_fr * p);
    motor_set_permille(rb->rr, rb->pol_rr * p);
}

void robot_stop(robot_drive_t *rb)
{
    if (!rb || !rb->fl || !rb->fr || !rb->rl || !rb->rr) return;
    motor_set_permille(rb->fl, 0);
    motor_set_permille(rb->fr, 0);
    motor_set_permille(rb->rl, 0);
    motor_set_permille(rb->rr, 0);
}