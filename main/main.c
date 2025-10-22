#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

////// Component Part >>>>>>
#include "led.h"
#include "myi2c.h"
#include "mpu6050.h"
#include "oled.h"
////// Component Part <<<<<<

static const char *TAG = "MAIN";

/* IMU 位姿滤波状态（Roll/Pitch/Yaw，单位：度） */
static float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;
/* FreeRTOS Tick 计时，用于积分 */
static TickType_t s_last_tick = 0;
/* 是否完成位姿初始对准（用加速度静态解算进行初始化） */
static bool s_pose_initialized = false;
static bool s_oled_ready = false;
static float s_gbias_x = 0.0f, s_gbias_y = 0.0f, s_gbias_z = 0.0f; /* 陀螺零速偏移（dps） */

/* 静止校准陀螺零速偏移（请在调用前保持设备静止）*/
static void calibrate_gyro_bias_static(int samples, int delay_ms)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int got = 0;
    ESP_LOGI(TAG, "开始陀螺零偏静止校准，请保持设备静止，采样次数=%d", samples);
    for (int i = 0; i < samples; ++i) {
        lsm6ds3_raw_data_t imu;
        esp_err_t r = lsm6ds3_read_raw(&imu);
        if (r == ESP_OK) {
            float gx = lsm6ds3_gyro_dps_default(imu.gyro_x);
            float gy = lsm6ds3_gyro_dps_default(imu.gyro_y);
            float gz = lsm6ds3_gyro_dps_default(imu.gyro_z);
            sum_x += gx; sum_y += gy; sum_z += gz; ++got;
        } else {
            ESP_LOGW(TAG, "校准采样失败: %s", esp_err_to_name(r));
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    if (got > 0) {
        s_gbias_x = sum_x / (float)got;
        s_gbias_y = sum_y / (float)got;
        s_gbias_z = sum_z / (float)got;
        ESP_LOGI(TAG, "零偏校准完成: bias[dps] X=%.3f Y=%.3f Z=%.3f (有效样本=%d)", s_gbias_x, s_gbias_y, s_gbias_z, got);
    } else {
        ESP_LOGW(TAG, "零偏校准失败：无有效样本");
    }
}

/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void) {
  esp_err_t ret;

  ESP_LOGI(TAG, "应用启动，IDF版本: %s", esp_get_idf_version());

  ret = nvs_flash_init(); /* 初始化NVS */
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "NVS 初始化成功");
  } else if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
             ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS 存储页不足或版本更新，擦除并重新初始化 NVS");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "NVS 重新初始化完成");
  } else {
    ESP_LOGE(TAG, "NVS 初始化失败: 0x%x", ret);
  }

  ESP_LOGI(TAG, "LED 初始化...");
  led_init();                        /* LED初始化 */

  ESP_LOGI(TAG, "IIC 初始化...");
  esp_log_level_set("IIC", ESP_LOG_DEBUG);
  ret = myiic_init();               /* 初始化IIC0 */  
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "IIC 总线初始化成功");
  } else {
    ESP_LOGE(TAG, "IIC 总线初始化失败: 0x%x", ret);
  }

  /* LSM6DS3 初始化（显式传入 bus/addr/speed）*/
  esp_log_level_set("LSM6DS3", ESP_LOG_DEBUG);
  ESP_LOGI(TAG, "LSM6DS3 初始化...");
  ret = lsm6ds3_init_on_bus(bus_handle, LSM6DS3_I2C_ADDR_DEFAULT, IIC_SPEED_CLK);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "LSM6DS3 初始化成功");
    calibrate_gyro_bias_static(200, 5); /* 启动静止零偏校准：200次，每次5ms，总约1秒 */
  } else {
    ESP_LOGE(TAG, "LSM6DS3 初始化失败: %s", esp_err_to_name(ret));
  }

  // OLED 初始化与演示
  ESP_LOGI(TAG, "OLED 初始化...");
  if (oled_init()) {
    s_oled_ready = true;
    ESP_LOGI(TAG, "OLED 初始化成功");
    oled_clear();
  } else {
    ESP_LOGE(TAG, "OLED 初始化失败");
  }

  ESP_LOGI(TAG, "进入主循环，周期 500ms");

  /* 初始化积分计时 */
  s_last_tick = xTaskGetTickCount();

  while (1) {
    LED_TOGGLE();

    /* 读取并打印温度 */
    float temp_c;
    esp_err_t temp_ret = lsm6ds3_read_temp_c(&temp_c);
    if (temp_ret == ESP_OK) {
      ESP_LOGI(TAG, "IMU Temp: %.2f C", temp_c);
    } else {
      ESP_LOGW(TAG, "读取温度失败: %s", esp_err_to_name(temp_ret));
    }

    /* 读取原始数据，计算三轴位姿（RPY） */
    lsm6ds3_raw_data_t imu;
    ret = lsm6ds3_read_raw(&imu);
    if (ret == ESP_OK) {
      /* 计算 dt（秒） */
      TickType_t now = xTaskGetTickCount();
      float dt = ((float)(now - s_last_tick)) * (portTICK_PERIOD_MS / 1000.0f);
      s_last_tick = now;

      /* 单位换算：加速度->g，角速度->dps */
      float ax = lsm6ds3_acc_g_default(imu.acc_x);
      float ay = lsm6ds3_acc_g_default(imu.acc_y);
      float az = lsm6ds3_acc_g_default(imu.acc_z);
      float gx = lsm6ds3_gyro_dps_default(imu.gyro_x) - s_gbias_x;
      float gy = lsm6ds3_gyro_dps_default(imu.gyro_y) - s_gbias_y;
      float gz = lsm6ds3_gyro_dps_default(imu.gyro_z) - s_gbias_z;

      /* 基于加速度的倾角（度）：roll = atan2(ay, az)，pitch = atan2(-ax, sqrt(ay^2+az^2)) */
      float roll_acc  = atan2f(ay, az) * 57.2957795f; /* 180/pi */
      float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;

      /* 互补滤波融合加速度与陀螺仪（建议 alpha≈0.98 在高采样率下使用；当前 500ms 周期可将 alpha 调低） */
      const float alpha = 0.95f; /* 周期较慢时适当降低，增强加速度稳定性 */

      if (!s_pose_initialized) {
        /* 用静态加速度初始对准 */
        s_roll = roll_acc;
        s_pitch = pitch_acc;
        s_yaw = 0.0f;
        s_pose_initialized = true;
      }

      /* 陀螺仪积分 + 互补滤波 */
      s_roll  = alpha * (s_roll  + gx * dt) + (1.0f - alpha) * roll_acc;
      s_pitch = alpha * (s_pitch + gy * dt) + (1.0f - alpha) * pitch_acc;
      s_yaw  += gz * dt; /* 没有磁力计，yaw 仅由陀螺仪积分，随时间可能漂移 */

      ESP_LOGI(TAG, "Pose[deg] Roll=%.2f Pitch=%.2f Yaw=%.2f | Acc[g] X=%.2f Y=%.2f Z=%.2f | Gyro[dps] X=%.1f Y=%.1f Z=%.1f",
               s_roll, s_pitch, s_yaw, ax, ay, az, gx, gy, gz);
    } else {
      ESP_LOGW(TAG, "读取 LSM6DS3 原始数据失败: %s", esp_err_to_name(ret));
    }

    // OLED 动态刷新测试：显示温度与姿态
    if (s_oled_ready) {
      char line2[22];
      if (temp_ret == ESP_OK) {
        snprintf(line2, sizeof(line2), "Temp:%8.2f C    ", temp_c);
      } else {
        snprintf(line2, sizeof(line2), "Temp:   N/A      ");
      }
      oled_ascii8(0, 2, line2);

      char line3[22];
      snprintf(line3, sizeof(line3), "Roll:%8.2f deg   ", s_roll);
      oled_ascii8(0, 3, line3);

      char line4[22];
      snprintf(line4, sizeof(line4), "Pitch:%8.2f deg  ", s_pitch);
      oled_ascii8(0, 4, line4);

      char line5[22];
      snprintf(line5, sizeof(line5), "Yaw:%8.2f deg    ", s_yaw);
      oled_ascii8(0, 5, line5);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
