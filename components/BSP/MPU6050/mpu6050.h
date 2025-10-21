#ifndef __MPU6050_H
#define __MPU6050_H

#include "esp_err.h"
#include <stdint.h>
#include "driver/i2c_master.h"  /* 适配 IDF v5.x，使用新的 I2C 总线/设备模型 */

#ifdef __cplusplus
extern "C" {
#endif

/* 注意：你的原理图为 LSM6DS3TR-C（非 MPU6050）。LSM6DS3 的 I2C 地址由 SA0 决定：
 * SA0=0 -> 0x6A；SA0=1 -> 0x6B
 */
#define LSM6DS3_I2C_ADDR_DEFAULT   0x6A    /* 默认按 SA0=0 处理，若拉高则传 0x6B */

/* LSM6DS3TR-C 寄存器地址（用于替换原 MPU6050 寄存器） */
#define LSM6DS3_REG_WHO_AM_I       0x0F    /* 期望值 0x69 */
#define LSM6DS3_REG_CTRL1_XL       0x10    /* 加速度控制/ODR/量程 */
#define LSM6DS3_REG_CTRL2_G        0x11    /* 陀螺仪控制/ODR/量程 */
#define LSM6DS3_REG_CTRL3_C        0x12    /* 通用控制：IF_INC/BDU/SW_RESET 等 */

#define LSM6DS3_REG_OUT_TEMP_L     0x20
#define LSM6DS3_REG_OUT_TEMP_H     0x21
#define LSM6DS3_REG_OUTX_L_G       0x22
#define LSM6DS3_REG_OUTX_H_G       0x23
#define LSM6DS3_REG_OUTY_L_G       0x24
#define LSM6DS3_REG_OUTY_H_G       0x25
#define LSM6DS3_REG_OUTZ_L_G       0x26
#define LSM6DS3_REG_OUTZ_H_G       0x27
#define LSM6DS3_REG_OUTX_L_XL      0x28
#define LSM6DS3_REG_OUTX_H_XL      0x29
#define LSM6DS3_REG_OUTY_L_XL      0x2A
#define LSM6DS3_REG_OUTY_H_XL      0x2B
#define LSM6DS3_REG_OUTZ_L_XL      0x2C
#define LSM6DS3_REG_OUTZ_H_XL      0x2D

/* 原始数据结构体：保持不变 */
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_raw; /* 原始温度值 */
} lsm6ds3_raw_data_t;

/* 旧 MPU6050 接口已移除，统一使用 lsm6ds3_* 接口 */

/* 在指定的 I2C 总线上初始化（仅按传入 addr/speed 配置，不做多重重试）*/
esp_err_t lsm6ds3_init_on_bus(i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t speed_hz);

/* 读取加速度、温度、角速度原始值（一次性读 14 字节） */
esp_err_t lsm6ds3_read_raw(lsm6ds3_raw_data_t *out);

/* 常用换算函数（按 LSM6DS3 默认量程设置） */
float lsm6ds3_temp_c(int16_t temp_raw);           /* 摄氏度 */
float lsm6ds3_acc_g_default(int16_t raw);         /* -> g */
float lsm6ds3_gyro_dps_default(int16_t raw);      /* -> deg/s */

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */