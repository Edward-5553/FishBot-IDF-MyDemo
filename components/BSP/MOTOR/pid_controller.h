#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 通用 PID 控制器（速度/位置均可）
 * 特性：
 * - 支持设定 Kp/Ki/Kd
 * - 支持输出限幅（防止驱动超界）
 * - 支持积分限幅（抗积分饱和）
 * - 支持使用“对测量值微分”（减少设定阶跃带来的导数突变）
 * - 支持可选一阶低通滤波对导数项（通过 alpha 控制，0..1，0=强滤波，1=不过滤）
 *
 * 推荐用法（速度环）：
 * - 输入 setpoint=目标速度（m/s 或 pps），measurement=当前速度
 * - dt 使用实际采样周期（秒）
 * - 输出用于驱动电机的功率指令（例如 permille -1000..1000）
 */

typedef struct {
    float kp;                 // 比例增益
    float ki;                 // 积分增益
    float kd;                 // 微分增益

    float integrator;         // 积分状态
    float prev_error;         // 上次误差（用于 D=对误差微分 的场景）
    float prev_measurement;   // 上次测量值（用于 D=对测量微分 的场景）
    float d_filtered;         // 导数项滤波状态

    float out_min;            // 输出最小值（例如 -1000）
    float out_max;            // 输出最大值（例如 +1000）
    float i_min;              // 积分最小值（用于抗饱和）
    float i_max;              // 积分最大值（用于抗饱和）

    float d_alpha;            // 导数项一阶滤波系数（0..1，越小滤波越强；默认 0.9）
    bool  d_on_measurement;   // true: 对测量微分；false: 对误差微分（默认 true）
} pid_controller_t;

/**
 * 初始化 PID 控制器，设置初始增益与限幅。
 * 注意：输出/积分限幅请根据你的执行器范围设置，例如电机 permille [-1000, 1000]。
 */
static inline void pid_init(pid_controller_t *pid,
                            float kp, float ki, float kd,
                            float out_min, float out_max,
                            float i_min, float i_max)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
    pid->out_min = out_min; pid->out_max = out_max;
    pid->i_min = i_min; pid->i_max = i_max;
    pid->d_alpha = 0.9f;
    pid->d_on_measurement = true;
}

/** 重置内部状态（积分/导数/上一帧数据） */
static inline void pid_reset(pid_controller_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;
}

/** 设置 PID 增益 */
static inline void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
}

/** 设置输出与积分限幅 */
static inline void pid_set_limits(pid_controller_t *pid, float out_min, float out_max,
                                  float i_min, float i_max)
{
    pid->out_min = out_min; pid->out_max = out_max;
    pid->i_min = i_min; pid->i_max = i_max;
}

/** 设置导数滤波强度（0..1），越小滤波越强；默认 0.9 */
static inline void pid_set_d_filter_alpha(pid_controller_t *pid, float alpha)
{
    pid->d_alpha = alpha;
}

/** 设置导数项计算方式：true=对测量微分（默认），false=对误差微分 */
static inline void pid_set_d_on_measurement(pid_controller_t *pid, bool enable)
{
    pid->d_on_measurement = enable;
}

/**
 * 计算 PID 输出
 * @param pid       控制器对象
 * @param setpoint  目标值（速度或位置）
 * @param measurement 测量值（当前速度或位置）
 * @param dt        采样周期（秒），必须>0
 * @return          受限幅后的输出（例如 permille 指令）
 */
static inline float pid_compute(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        // 避免除零：返回上次输出的期望区间内值（此处返回比例项 0）
        return 0.0f;
    }

    // 误差
    const float error = setpoint - measurement;

    // 比例项
    const float p_term = pid->kp * error;

    // 积分项（带限幅抗饱和）
    pid->integrator += pid->ki * error * dt;
    if (pid->integrator > pid->i_max) pid->integrator = pid->i_max;
    if (pid->integrator < pid->i_min) pid->integrator = pid->i_min;
    const float i_term = pid->integrator;

    // 导数项：默认对测量微分（避免设定阶跃造成导数冲击）
    float d_raw;
    if (pid->d_on_measurement) {
        d_raw = -(measurement - pid->prev_measurement) / dt; // d/dt(meas) 的负号
        pid->prev_measurement = measurement;
    } else {
        d_raw = (error - pid->prev_error) / dt;
        pid->prev_error = error;
    }
    // 一阶滤波：d_filtered = alpha*d_filtered + (1-alpha)*d_raw
    pid->d_filtered = pid->d_alpha * pid->d_filtered + (1.0f - pid->d_alpha) * d_raw;
    const float d_term = pid->kd * pid->d_filtered;

    // 合成并输出限幅
    float out = p_term + i_term + d_term;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    return out;
}

#ifdef __cplusplus
}
#endif