// Rotary encoder (A/B phase) interface for FishBot wheels, PCNT-based
// Follows API style from V4Refer/motor/rotary_encoder, adapted into BSP/MOTOR
/*
 * 模块说明：
 *  - 提供统一的旋转编码器接口（A/B 双相，基于 PCNT 外设），用于车轮速度/位移/方向检测。
 *  - 通过 rotary_encoder_t 结构体暴露一组方法（设置毛刺滤波、启动/停止、删除、获取计数、轮速计算）。
 *  - 具体实现见 rotary_encoder_pcnt_ec11.c，遵循 EC11 风格的 A/B 相计数配置。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/*
 * 底层设备句柄：
 *  - 一般用于传入 PCNT 的 unit（pcnt_unit_t），在此统一抽象为 void*。
 *  - 不同实现可以复用该字段承载各自的设备/资源句柄。
 */
typedef void *rotary_encoder_dev_t;

/*
 * 旋转编码器配置（A/B 两相的 GPIO）
 *  - dev:    底层设备句柄（例如 pcnt_unit_t，需强制转换为 rotary_encoder_dev_t）
 *  - phase_a_gpio_num: A 相引脚编号（作为脉冲或控制输入，取决于具体实现）
 *  - phase_b_gpio_num: B 相引脚编号
 *  - flags:  预留标志位，当前未使用
 */
typedef struct {
    /* 底层设备句柄（例如 pcnt_unit_t）*/
    rotary_encoder_dev_t dev;
    /* A 相 GPIO 编号 */
    int phase_a_gpio_num;
    /* B 相 GPIO 编号 */
    int phase_b_gpio_num;
    /* 预留标志位 */
    int flags;
} rotary_encoder_config_t;

/*
 * 默认配置宏：快速填充配置结构
 *  - dev_hdl: 设备句柄（例如 (rotary_encoder_dev_t)PCNT_UNIT_0）
 *  - gpio_a/gpio_b: A/B 两相的 GPIO 编号
 */
#define ROTARY_ENCODER_DEFAULT_CONFIG(dev_hdl, gpio_a, gpio_b) \
    {                                                          \
        .dev = dev_hdl,                                        \
        .phase_a_gpio_num = gpio_a,                            \
        .phase_b_gpio_num = gpio_b,                            \
        .flags = 0,                                            \
    }

// Encoder handle forward declaration
typedef struct rotary_encoder_t rotary_encoder_t;

/*
 * 编码器接口（函数指针表）：
 *  - set_glitch_filter：设置 PCNT 毛刺滤波窗口（单位：微秒），用于抑制输入抖动/噪声。
 *  - start            ：启动计数（恢复 PCNT 计数器工作）。
 *  - stop             ：停止计数（暂停 PCNT 计数器工作）。
 *  - del              ：删除编码器对象，释放资源（实现可能同时禁用相关事件）。
 *  - get_counter_value：获取当前累计计数值（含溢出补偿）。
 *  - config_wheel     ：配置车轮几何参数（直径，单位 mm）与每圈脉冲数，用于速度换算。
 *  - get_speed_mps    ：以“上次调用到本次调用”的时间窗计算瞬时轮速（单位 m/s）。
 */
struct rotary_encoder_t {
    esp_err_t (*set_glitch_filter)(rotary_encoder_t *encoder, uint32_t max_glitch_us);
    esp_err_t (*start)(rotary_encoder_t *encoder);
    esp_err_t (*stop)(rotary_encoder_t *encoder);
    esp_err_t (*del)(rotary_encoder_t *encoder);
    int       (*get_counter_value)(rotary_encoder_t *encoder);
    // 新增：配置车轮直径与每圈脉冲数，用于将脉冲换算为轮速
    esp_err_t (*config_wheel)(rotary_encoder_t *encoder, float diameter_mm, int counts_per_rev);
    // 新增：获取轮速（m/s），基于 config_wheel 设置和最近一次采样窗口
    float     (*get_speed_mps)(rotary_encoder_t *encoder);
};

/*
 * 创建 EC11 风格的 A/B 双相编码器（基于 PCNT）
 * 参数：
 *  - config: 编码器配置（包含设备句柄与 A/B 引脚）
 *  - ret_encoder: 输出的编码器接口指针（成功返回后可通过其方法进行操作）
 * 返回：
 *  - ESP_OK            成功
 *  - 其他错误码        失败（例如参数错误、内存不足、PCNT 配置失败等）
 */
esp_err_t rotary_encoder_new_ec11(const rotary_encoder_config_t *config, rotary_encoder_t **ret_encoder);

/*
 * 便捷创建函数：传入 PCNT 单元与 A/B 引脚，内部：
 *  - 调用 rotary_encoder_new_ec11 创建对象
 *  - 设置默认毛刺滤波（1us）
 *  - 启动计数
 * 返回：编码器对象指针；若失败返回 NULL
 */
rotary_encoder_t * create_rotary_encoder(uint32_t pcnt_unit, uint8_t channela_num, uint8_t channelb_num);

#ifdef __cplusplus
}
#endif