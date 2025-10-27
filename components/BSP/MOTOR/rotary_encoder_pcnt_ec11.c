// Rotary encoder PCNT implementation (EC11-style, A/B phase) for FishBot wheels
// Follows V4Refer implementation, integrated into BSP/MOTOR
/*
 * 模块简介：
 *  - 使用 ESP32 PCNT 外设对 A/B 双相编码器进行硬件计数，自动识别正/反方向。
 *  - 通道0：A 为脉冲输入，B 为控制信号；通道1：B 为脉冲输入，A 为控制信号。
 *  - 通过 PCNT 的高/低计数限触发事件实现溢出/下溢补偿，确保计数可无限累积。
 *  - 提供毛刺滤波接口以抑制输入抖动，适合机械/霍尔传感器。
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pcnt.h"
#include "rotary_encoder.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 日志 TAG */
static const char *TAG = "BSP_ENCODER";

/*
 * PCNT 计数上下限：触发事件用于累加（溢出/下溢）
 *  - counter_h_lim = +100，counter_l_lim = -100
 *  - 当达到高/低限时，进入中断回调，将固定值累加到软件计数上。
 */
#define EC11_PCNT_DEFAULT_HIGH_LIMIT (100)
#define EC11_PCNT_DEFAULT_LOW_LIMIT  (-100)

/*
 * 统一的检查宏：如果条件不满足，打印错误、设置返回码并跳转到 err 标签。
 */
#define ROTARY_CHECK(a, msg, ret, ...)                                           \
    do {                                                                          \
        if (!(a)) {                                                               \
            ESP_LOGE(TAG, "%s(%d): " msg, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret_code = ret;                                                       \
            goto err;                                                             \
        }                                                                         \
    } while (0)

/*
 * 编码器实现对象（EC11 风格）：
 *  - parent       ：接口方法表（必须是第一个字段，保证从接口指针强转后能正确取到实现对象）
 *  - accumu_count ：软件累计计数，用于 PCNT 溢出/下溢补偿
 *  - pcnt_unit    ：使用的 PCNT 单元编号
 *  - wheel_diameter_mm ：车轮直径（mm），默认 65mm
 *  - counts_per_rev    ：一圈的“计数数”，需根据实际测量设置（A/B 四倍边沿计数后得到的总计数）
 *  - last_count        ：上次获取计数值（用于速度计算）
 *  - last_time_us      ：上次速度计算的时间戳（us）
 *  - last_speed_mps    ：上次计算得到的速度（m/s），用于 dt 很小时返回稳定值
 */
typedef struct {
    rotary_encoder_t parent;    // interface methods MUST be the first field
    int accumu_count;           // accumulated counter on overflow/underflow
    pcnt_unit_t pcnt_unit;      // PCNT unit used
    float wheel_diameter_mm;    // wheel diameter
    int   counts_per_rev;       // counts per revolution (must be measured)
    int   last_count;           // last raw count for speed calc
    int64_t last_time_us;       // last timestamp in us
    float last_speed_mps;       // last computed speed
} ec11_t;

/*
 * 设置 PCNT 毛刺滤波窗口（单位：微秒）
 *  - PCNT 硬件滤波单位为 APB 周期（约 80MHz），此处将 us 转换为 cycles（us*80）。
 *  - 当 max_glitch_us > 0 时启用滤波；为 0 则禁用滤波。
 */
static esp_err_t ec11_set_glitch_filter(rotary_encoder_t *encoder, uint32_t max_glitch_us)
{
    esp_err_t ret_code = ESP_OK;
    ec11_t *ec11 = (ec11_t *)encoder; // safe because parent is the first field

    uint32_t cycles = max_glitch_us * 80;
    ROTARY_CHECK(pcnt_set_filter_value(ec11->pcnt_unit, cycles) == ESP_OK, "set glitch filter failed", ESP_FAIL);
    if (max_glitch_us) {
        pcnt_filter_enable(ec11->pcnt_unit);
    } else {
        pcnt_filter_disable(ec11->pcnt_unit);
    }
    return ESP_OK;
err:
    return ret_code;
}

/*
 * 启动计数：恢复 PCNT 计数器工作。
 */
static esp_err_t ec11_start(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = (ec11_t *)encoder;
    pcnt_counter_resume(ec11->pcnt_unit);
    // 启动时同步一次计数与时间，避免初次速度计算出现巨大 dt
    pcnt_get_counter_value(ec11->pcnt_unit, (int16_t*)&ec11->last_count);
    ec11->last_time_us = esp_timer_get_time();
    return ESP_OK;
}

/*
 * 停止计数：暂停 PCNT 计数器工作。
 */
static esp_err_t ec11_stop(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = (ec11_t *)encoder;
    pcnt_counter_pause(ec11->pcnt_unit);
    return ESP_OK;
}

/*
 * 获取累计计数值：PCNT 当前值 + 软件累计值（溢出/下溢补偿）。
 */
static int ec11_get_counter_value(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = (ec11_t *)encoder;
    int16_t val = 0;
    pcnt_get_counter_value(ec11->pcnt_unit, &val);
    return val + ec11->accumu_count;
}

/*
 * 配置车轮几何参数与每圈计数：
 *  - diameter_mm    ：车轮直径（mm）
 *  - counts_per_rev ：一圈的计数数量（根据实际测量设置）
 */
static esp_err_t ec11_config_wheel(rotary_encoder_t *encoder, float diameter_mm, int counts_per_rev)
{
    ec11_t *ec11 = (ec11_t *)encoder;
    if (diameter_mm <= 0.0f || counts_per_rev <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    ec11->wheel_diameter_mm = diameter_mm;
    ec11->counts_per_rev = counts_per_rev;
    return ESP_OK;
}

/*
 * 计算瞬时轮速（m/s）：以“上次调用到本次调用”的时间窗为基准
 * 公式：speed = Δcount / counts_per_rev * (π*D/1000) / Δt
 */
static float ec11_get_speed_mps(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = (ec11_t *)encoder;
    int64_t now_us = esp_timer_get_time();
    int cur = ec11_get_counter_value(encoder);
    int delta = cur - ec11->last_count;
    int64_t dt_us = now_us - ec11->last_time_us;
    // 更新采样窗口
    ec11->last_count = cur;
    ec11->last_time_us = now_us;

    if (ec11->counts_per_rev <= 0 || dt_us <= 0) {
        return 0.0f;
    }
    float dt = (float)dt_us / 1000000.0f; // s
    float circumference_m = (float)M_PI * (ec11->wheel_diameter_mm / 1000.0f);
    float meters_per_count = circumference_m / (float)ec11->counts_per_rev;
    float speed_mps = (float)delta * meters_per_count / dt; // signed by delta
    ec11->last_speed_mps = speed_mps;
    return speed_mps;
}

/*
 * 删除对象：禁用事件并释放内存。
 *  - 此处仅禁用高/低限事件；ISR 移除需要相同回调指针，简单起见不做移除（IDF 允许多对象共享同一服务）。
 */
static esp_err_t ec11_del(rotary_encoder_t *encoder)
{
    ec11_t *ec11 = (ec11_t *)encoder;
    pcnt_event_disable(ec11->pcnt_unit, PCNT_EVT_H_LIM);
    pcnt_event_disable(ec11->pcnt_unit, PCNT_EVT_L_LIM);
    free(ec11);
    return ESP_OK;
}

/*
 * PCNT 事件回调：处理高/低限触发，进行软件累计值补偿。
 */
static void ec11_pcnt_overflow_handler(void *arg)
{
    ec11_t *ec11 = (ec11_t *)arg;
    uint32_t status = 0;
    pcnt_get_event_status(ec11->pcnt_unit, &status);

    if (status & PCNT_EVT_H_LIM) {
        ec11->accumu_count += EC11_PCNT_DEFAULT_HIGH_LIMIT;
    } else if (status & PCNT_EVT_L_LIM) {
        ec11->accumu_count += EC11_PCNT_DEFAULT_LOW_LIMIT;
    }
}

/*
 * 创建 EC11 风格编码器：
 *  - 配置 PCNT 两个通道以实现 A/B 相相位判断和方向计数。
 *    通道0（A->pulse, B->ctrl）：pos=DEC, neg=INC, lctrl=REVERSE, hctrl=KEEP
 *    通道1（B->pulse, A->ctrl）：pos=INC, neg=DEC
 *  - 暂停并清零计数器，安装 ISR 服务（仅安装一次），注册事件回调并使能高/低限事件。
 *  - 设置默认车轮直径 65mm；counts_per_rev 需由用户测量并通过 config_wheel 设置。
 *  - 填充接口方法表，返回编码器对象。
 */
esp_err_t rotary_encoder_new_ec11(const rotary_encoder_config_t *config, rotary_encoder_t **ret_encoder)
{
    esp_err_t ret_code = ESP_OK;
    ec11_t *ec11 = NULL;

    ROTARY_CHECK(config != NULL, "configuration can't be null", ESP_ERR_INVALID_ARG);
    ROTARY_CHECK(ret_encoder != NULL, "can't assign context to null", ESP_ERR_INVALID_ARG);

    ec11 = (ec11_t *)calloc(1, sizeof(ec11_t));
    ROTARY_CHECK(ec11 != NULL, "allocate context memory failed", ESP_ERR_NO_MEM);

    ec11->pcnt_unit = (pcnt_unit_t)(config->dev);

    ESP_LOGI(TAG, "pcnt_unit=%d, A=%d, B=%d", ec11->pcnt_unit, config->phase_a_gpio_num, config->phase_b_gpio_num);

    // Configure PCNT channel 0: A pulses, B control
    pcnt_config_t dev_config = {
        .pulse_gpio_num = config->phase_a_gpio_num,
        .ctrl_gpio_num = config->phase_b_gpio_num,
        .channel = PCNT_CHANNEL_0,
        .unit = ec11->pcnt_unit,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = EC11_PCNT_DEFAULT_HIGH_LIMIT,
        .counter_l_lim = EC11_PCNT_DEFAULT_LOW_LIMIT,
    };
    ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "config pcnt channel 0 failed", ESP_FAIL);

    // Configure PCNT channel 1: B pulses, A control
    dev_config.pulse_gpio_num = config->phase_b_gpio_num;
    dev_config.ctrl_gpio_num = config->phase_a_gpio_num;
    dev_config.channel = PCNT_CHANNEL_1;
    dev_config.pos_mode = PCNT_COUNT_INC;
    dev_config.neg_mode = PCNT_COUNT_DEC;
    ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "config pcnt channel 1 failed", ESP_FAIL);

    // Pause and clear counter before starting
    pcnt_counter_pause(ec11->pcnt_unit);
    pcnt_counter_clear(ec11->pcnt_unit);

    // Register ISR service (install once globally)
    static bool s_isr_installed = false;
    if (!s_isr_installed) {
        ROTARY_CHECK(pcnt_isr_service_install(0) == ESP_OK, "install isr service failed", ESP_FAIL);
        s_isr_installed = true;
    }

    // Add overflow/underflow handler
    pcnt_isr_handler_add(ec11->pcnt_unit, ec11_pcnt_overflow_handler, ec11);
    pcnt_event_enable(ec11->pcnt_unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(ec11->pcnt_unit, PCNT_EVT_L_LIM);

    // 初始化轮几何与采样状态
    ec11->wheel_diameter_mm = 65.0f; // 默认直径 65mm，根据项目说明
    ec11->counts_per_rev = 0;        // 需要实际测量后设置正确值，默认 0 表示未配置
    ec11->last_count = 0;
    ec11->last_time_us = esp_timer_get_time();
    ec11->last_speed_mps = 0.0f;

    // Fill vtable
    ec11->parent.del = ec11_del;
    ec11->parent.start = ec11_start;
    ec11->parent.stop = ec11_stop;
    ec11->parent.set_glitch_filter = ec11_set_glitch_filter;
    ec11->parent.get_counter_value = ec11_get_counter_value;
    ec11->parent.config_wheel = ec11_config_wheel;
    ec11->parent.get_speed_mps = ec11_get_speed_mps;

    *ret_encoder = &(ec11->parent);
    return ESP_OK;
err:
    if (ec11) {
        free(ec11);
    }
    return ret_code;
}

/*
 * 便捷创建函数：
 *  - 传入 PCNT 单元和 A/B 引脚，内部创建编码器对象。
 *  - 设置默认毛刺滤波（1us），并启动计数器。
 *  - 若任一步失败，释放对象并返回 NULL。
 */
rotary_encoder_t * create_rotary_encoder(uint32_t pcnt_unit, uint8_t channela_num, uint8_t channelb_num)
{
    rotary_encoder_t *encoder = NULL;
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, channela_num, channelb_num);
    if (rotary_encoder_new_ec11(&config, &encoder) != ESP_OK) {
        return NULL;
    }
    if (encoder->set_glitch_filter(encoder, 1) != ESP_OK) { // 1us glitch filter as default
        encoder->del(encoder);
        return NULL;
    }
    if (encoder->start(encoder) != ESP_OK) {
        encoder->del(encoder);
        return NULL;
    }
    return encoder;
}