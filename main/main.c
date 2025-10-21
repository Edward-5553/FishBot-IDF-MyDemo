#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

////// Component Part >>>>>>
#include "led.h"
#include "myi2c.h"
#include "mpu6050.h"
////// Component Part <<<<<<

static const char *TAG = "MAIN";

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
  } else {
    ESP_LOGE(TAG, "LSM6DS3 初始化失败: %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG, "进入主循环，周期 500ms");

  while (1) {
    LED_TOGGLE();
    /*
    // 读取 LSM6DS3 原始数据示例
    lsm6ds3_raw_data_t imu;
    ret = lsm6ds3_read_raw(&imu);
    if (ret == ESP_OK) {
      float ax = lsm6ds3_acc_g_default(imu.acc_x);
      float ay = lsm6ds3_acc_g_default(imu.acc_y);
      float az = lsm6ds3_acc_g_default(imu.acc_z);
      float gx = lsm6ds3_gyro_dps_default(imu.gyro_x);
      float gy = lsm6ds3_gyro_dps_default(imu.gyro_y);
      float gz = lsm6ds3_gyro_dps_default(imu.gyro_z);
      float tc = lsm6ds3_temp_c(imu.temp_raw);
      ESP_LOGI(TAG, "Accel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.1f Y=%.1f Z=%.1f | Temp=%.2fC",
               ax, ay, az, gx, gy, gz, tc);
    } else {
      ESP_LOGW(TAG, "读取 LSM6DS3 失败: %s", esp_err_to_name(ret));
    }
    */

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
