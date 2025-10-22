#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#include "pwm.h"  // from components/LIB/PWM

#ifdef __cplusplus
extern "C" {
#endif

// 新增：驱动模式选择（参考 V4 的两种常见用法）
typedef enum {
    MOTOR_DRIVE_BRAKE = 0,  // 驱动/刹车（慢衰减），低速起动更有力
    MOTOR_DRIVE_COAST = 1   // 驱动/空转（快衰减），能耗低、声音小
} motor_drive_mode_t;

// 电机对象，两个输入都使用PWM通道（只在一侧输出PWM，另一侧拉低）
typedef struct {
    pwm_channel_t in1;     // IN1 -> OUT1 控制
    pwm_channel_t in2;     // IN2 -> OUT2 控制
    bool initialized;
    motor_drive_mode_t drive_mode; // 新增：驱动模式（默认 BRAKE）
} motor_t;

// 初始化电机，两路输入共用同一个LEDC定时器与频率/分辨率
esp_err_t motor_init(motor_t *m,
                     int in1_gpio,
                     int in2_gpio,
                     ledc_channel_t in1_channel,
                     ledc_channel_t in2_channel,
                     ledc_timer_t timer,
                     ledc_mode_t mode,
                     uint32_t freq_hz,
                     ledc_timer_bit_t resolution);

// 设定速度（千分比，范围-1000..1000）。正数正转，负数反转，0停止（空转）。
esp_err_t motor_set_permille(motor_t *m, int permille);

// 设置驱动模式（BRAKE/COAST），与 V4Refer 的两类驱动方式一致
void motor_set_drive_mode(motor_t *m, motor_drive_mode_t mode);

// 空转停止（IN1=0, IN2=0）
esp_err_t motor_stop_freewheel(motor_t *m);

// 电子刹车（IN1=100%，IN2=100%）
esp_err_t motor_brake(motor_t *m);

/*
 * 统一小车控制接口（框架）：基于四轮电机与极性，提供方向/转向控制
 */
typedef struct {
    motor_t *fl;  // 左前 Front-Left
    motor_t *fr;  // 右前 Front-Right
    motor_t *rl;  // 左后 Rear-Left
    motor_t *rr;  // 右后 Rear-Right
    int pol_fl;   // 左前极性（+1/-1）
    int pol_fr;   // 右前极性（+1/-1）
    int pol_rl;   // 左后极性（+1/-1）
    int pol_rr;   // 右后极性（+1/-1）
    int turn_slow_factor_permille; // 差速转向时内侧轮的速度比例（0..1000，默认500=50%）
} robot_drive_t;

void robot_init_drive(robot_drive_t *rb,
                      motor_t *fl, motor_t *fr, motor_t *rl, motor_t *rr,
                      int pol_fl, int pol_fr, int pol_rl, int pol_rr);

void robot_set_turn_slow_factor(robot_drive_t *rb, int factor_permille);

void robot_wheels_set_permille(robot_drive_t *rb, int permille);
void robot_drive_forward(robot_drive_t *rb, int permille);
void robot_drive_backward(robot_drive_t *rb, int permille);
void robot_turn_left(robot_drive_t *rb, int permille);
void robot_turn_right(robot_drive_t *rb, int permille);
void robot_rotate_in_place(robot_drive_t *rb, int permille);
void robot_stop(robot_drive_t *rb);

#ifdef __cplusplus
}
#endif