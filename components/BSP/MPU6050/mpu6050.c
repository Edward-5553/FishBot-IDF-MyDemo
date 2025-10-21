#include "mpu6050.h"
#include "myi2c.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LSM6DS3";

/* 设备句柄（添加到 I2C 总线后获得） */
static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_addr = LSM6DS3_I2C_ADDR_DEFAULT; /* 默认 0x6A */

/* I2C 传输超时（毫秒） */
#define LSM6DS3_I2C_TIMEOUT_MS 1000

/* 写单个寄存器 */
static esp_err_t lsm6ds3_write_reg(uint8_t reg, uint8_t data)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = { reg, data };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), LSM6DS3_I2C_TIMEOUT_MS);
}

/* 读多个寄存器（从首地址开始连续读取） */
static esp_err_t lsm6ds3_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, LSM6DS3_I2C_TIMEOUT_MS);
}

/* 在指定总线上进行初始化（简化：不做多个地址/速率重试） */
esp_err_t lsm6ds3_init_on_bus(i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t speed_hz)
{
    if (bus == NULL) {
        ESP_LOGE(TAG, "I2C 总线未初始化或句柄为空，请先调用 myiic_init() 或传入有效 bus 句柄");
        return ESP_ERR_INVALID_STATE;
    }

    /* 若之前已添加过设备，先移除 */
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }

    s_addr = i2c_addr;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_addr,
        .scl_speed_hz = speed_hz,
    };
    ESP_LOGI(TAG, "添加 LSM6DS3 设备: addr=0x%02X, speed=%lu Hz", s_addr, (unsigned long)speed_hz);
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加设备失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 读取 WHO_AM_I，应为 0x69 */
    uint8_t who = 0;
    err = lsm6ds3_read_regs(LSM6DS3_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "读取 WHO_AM_I 失败: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);
    if (who == 0x69) {
        ESP_LOGI(TAG, "识别为 LSM6DS3/DS3TR-C");
    } else if (who == 0x6A) {
        ESP_LOGI(TAG, "识别为 LSM6DSL（与 DS3 寄存器相近）");
    } else {
        ESP_LOGW(TAG, "WHO_AM_I 异常(非 0x69/0x6A)，请确认器件型号/地址/连线/模式(CS=高为 I2C)");
    }

    /* 基本配置（LSM6DS3）：
     * - CTRL3_C: IF_INC=1 (自动地址递增), BDU=1
     * - CTRL1_XL: ODR=104Hz, FS=±2g
     * - CTRL2_G : ODR=104Hz, FS=±245dps
     */
    err = lsm6ds3_write_reg(LSM6DS3_REG_CTRL3_C, 0x44);
    if (err != ESP_OK) { ESP_LOGE(TAG, "写 CTRL3_C 失败: %s", esp_err_to_name(err)); return err; }

    err = lsm6ds3_write_reg(LSM6DS3_REG_CTRL1_XL, 0x40);
    if (err != ESP_OK) { ESP_LOGE(TAG, "写 CTRL1_XL 失败: %s", esp_err_to_name(err)); return err; }

    err = lsm6ds3_write_reg(LSM6DS3_REG_CTRL2_G, 0x40);
    if (err != ESP_OK) { ESP_LOGE(TAG, "写 CTRL2_G 失败: %s", esp_err_to_name(err)); return err; }

    ESP_LOGI(TAG, "LSM6DS3 初始化完成，地址 0x%02X", s_addr);
    return ESP_OK;
}

esp_err_t lsm6ds3_read_raw(lsm6ds3_raw_data_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t buf[14];
    /* LSM6DS3 从 OUT_TEMP_L 连续读取 14 字节: 温度(2) + G(6) + XL(6) */
    esp_err_t err = lsm6ds3_read_regs(LSM6DS3_REG_OUT_TEMP_L, buf, sizeof(buf));
    if (err != ESP_OK) return err;

    out->temp_raw= (int16_t)((buf[1]  << 8) | buf[0]);
    out->gyro_x  = (int16_t)((buf[3]  << 8) | buf[2]);
    out->gyro_y  = (int16_t)((buf[5]  << 8) | buf[4]);
    out->gyro_z  = (int16_t)((buf[7]  << 8) | buf[6]);
    out->acc_x   = (int16_t)((buf[9]  << 8) | buf[8]);
    out->acc_y   = (int16_t)((buf[11] << 8) | buf[10]);
    out->acc_z   = (int16_t)((buf[13] << 8) | buf[12]);

    return ESP_OK;
}

esp_err_t lsm6ds3_read_temp_raw(int16_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    uint8_t b[2];
    esp_err_t err = lsm6ds3_read_regs(LSM6DS3_REG_OUT_TEMP_L, b, 2);
    if (err != ESP_OK) return err;
    *out = (int16_t)((b[1] << 8) | b[0]);
    return ESP_OK;
}

esp_err_t lsm6ds3_read_temp_c(float *out_c)
{
    if (!out_c) return ESP_ERR_INVALID_ARG;
    int16_t raw;
    esp_err_t err = lsm6ds3_read_temp_raw(&raw);
    if (err != ESP_OK) return err;
    *out_c = lsm6ds3_temp_c(raw);
    return ESP_OK;
}

float lsm6ds3_temp_c(int16_t temp_raw)
{
    /* LSM6DS3/LSM6DSL: Temp(°C) = 25 + TEMP_OUT/256
     * 参考 ST 官方驱动与社区答复示例：lsm6dsm_from_lsb_to_celsius(lsb) = (lsb/256) + 25
     */
    return 25.0f + ((float)temp_raw / 256.0f);
}

float lsm6ds3_acc_g_default(int16_t raw)
{
    /* LSM6DS3 ±2g 模式比例因子约 0.061 mg/LSB -> 0.000061 g/LSB */
    return (float)raw * 0.000061f;
}

float lsm6ds3_gyro_dps_default(int16_t raw)
{
    /* LSM6DS3 ±245 dps 模式比例因子约 8.75 mdps/LSB -> 0.00875 dps/LSB */
    return (float)raw * 0.00875f;
}