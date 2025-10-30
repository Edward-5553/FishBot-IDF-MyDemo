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
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <stdbool.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

////// Component Part >>>>>>
#include "led.h"
#include "myi2c.h"
#include "mpu6050.h"
#include "oled.h"
#include "motor.h"
// 新增：编码器（A/B 双相，PCNT 计数）
#include "rotary_encoder.h"
#include "pid_controller.h"  // 新增：速度 PID 控制器
#include "driver/pcnt.h"
////// Component Part <<<<<<

////// MicroROS Part >>>>>>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/publisher.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include "lwip/ip4_addr.h"
////// MicroROS Part <<<<<<

#include <global_config.h>

static const char *TAG = "MAIN";

// Static Variables
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_publisher_t publisher;
static geometry_msgs__msg__Twist velocity_msg;



// 新增：参考 V4 驱动方式，默认使用 COAST（快衰减）以先确保能转动
#define MOTOR_DEMO_DRIVE_MODE_BRAKE_DEFAULT  0
// 新增：是否自动执行一次前进/后退方向校验（运行 2s 前进 + 2s 后退）
#define ROBOT_DIR_TEST_ENABLE                 0
// 用户测试宏：置1启用MOTOR2占空70%并打印轮速，置0编译移除该测试代码
#define MOTOR2_SPEED_TEST_ENABLE              0
// 用户测试宏：置1启用车轮速度 PID 闭环控制（目标速度 m/s）。
// 为兼容历史，此宏仍沿用原名（开启后将为所有轮子创建 PID 控制，若某些轮子尚未配置编码器，则以右前轮的测量作为临时反馈）。
#define MOTOR2_PID_TEST_ENABLE                0
// PID 目标速度（m/s），请根据场景安全设置，初值偏保守
#define PID_TARGET_SPEED_MPS                  0.20f
// PID 初始增益（单位：输出permille/速度m/s），请按需要调参
#define PID_KP                                600.0f
#define PID_KI                                200.0f
#define PID_KD                                0.0f

// —— 四轮编码器配置（统一使用宏，便于后期修改）——
// 说明：
// 1) 右前轮（FR）原先为固定写法，现改为宏定义形式，默认 FR 使用 PCNT_UNIT_0，A=GPIO17，B=GPIO18。
// 2) 左前（FL）、左后（RL）、右后（RR）同样通过宏启用与配置；如暂未连接，可将 ENABLE 置0。
// 3) 若某轮 ENABLE=0 则不会创建该轮编码器；PID 控制时该轮将暂用 FR 的速度作为反馈，待启用后再切换为独立闭环。

// 右前轮（Motor2）编码器：默认 A=GPIO17, B=GPIO18，默认使用 PCNT_UNIT_0
#define ENCODER_FR_ENABLE 1
#if ENCODER_FR_ENABLE
  #define ENC_FR_PCNT_UNIT  PCNT_UNIT_0
  #define ENC_FR_GPIO_A     GPIO_NUM_17
  #define ENC_FR_GPIO_B     GPIO_NUM_18
#endif

// 左前轮（Motor1）编码器：A=GPIO7, B=GPIO6，建议使用 PCNT_UNIT_1
#define ENCODER_FL_ENABLE 1
#if ENCODER_FL_ENABLE
  #define ENC_FL_PCNT_UNIT  PCNT_UNIT_1
  #define ENC_FL_GPIO_A     GPIO_NUM_7
  #define ENC_FL_GPIO_B     GPIO_NUM_6
#endif

// 左后轮（Motor3）编码器：A=GPIO19, B=GPIO20，建议使用 PCNT_UNIT_2
#define ENCODER_RL_ENABLE 1
#if ENCODER_RL_ENABLE
  #define ENC_RL_PCNT_UNIT  PCNT_UNIT_2
  #define ENC_RL_GPIO_A     GPIO_NUM_19
  #define ENC_RL_GPIO_B     GPIO_NUM_20
#endif

// 右后轮（Motor4）编码器：A=GPIO11, B=GPIO10，建议使用 PCNT_UNIT_3
#define ENCODER_RR_ENABLE 1
#if ENCODER_RR_ENABLE
  #define ENC_RR_PCNT_UNIT  PCNT_UNIT_3
  #define ENC_RR_GPIO_A     GPIO_NUM_11
  #define ENC_RR_GPIO_B     GPIO_NUM_10
#endif

/* IMU 位姿滤波状态（Roll/Pitch/Yaw，单位：度） */
static float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;
/* FreeRTOS Tick 计时，用于积分 */
static TickType_t s_last_tick = 0;
/* 是否完成位姿初始对准（用加速度静态解算进行初始化） */
static bool s_pose_initialized = false;
static bool s_oled_ready = false;
static float s_gbias_x = 0.0f, s_gbias_y = 0.0f, s_gbias_z = 0.0f; /* 陀螺零速偏移（dps） */
static motor_t s_motor1, s_motor2, s_motor3, s_motor4;            /* 四路电机对象 */
// 轮位映射：motor1=左前, motor2=右前, motor3=左后, motor4=右后（保留以支持后续方向控制）
#define MOTOR_FL s_motor1 // 左前 Front-Left
#define MOTOR_FR s_motor2 // 右前 Front-Right
#define MOTOR_RL s_motor3 // 左后 Rear-Left
#define MOTOR_RR s_motor4 // 右后 Rear-Right
// 每个轮位的“逻辑正转”极性（+1：permille>0 为正转；-1：permille<0 为正转）（保留以支持后续方向控制）
#define MOTOR_POL_FL (-1)
#define MOTOR_POL_FR (+1)
#define MOTOR_POL_RL (-1)
#define MOTOR_POL_RR (+1)
// 新增：统一车辆驱动对象 + 编码器对象（示例：右前轮）
static robot_drive_t s_rb;                    // 统一驱动对象
static rotary_encoder_t *s_enc_fr = NULL;     // 右前轮编码器（宏定义：ENC_FR_GPIO_A/ENC_FR_GPIO_B/ENC_FR_PCNT_UNIT）
static rotary_encoder_t *s_enc_fl = NULL;     // 左前轮编码器（需填写 A/B 引脚）
static rotary_encoder_t *s_enc_rl = NULL;     // 左后轮编码器（需填写 A/B 引脚）
static rotary_encoder_t *s_enc_rr = NULL;     // 右后轮编码器（需填写 A/B 引脚）
static int s_enc_last_cnt = 0;                // 上次计数
static TickType_t s_enc_last_tick = 0;        // 上次计数时间戳
static int s_counts_per_rev_cfg = 0;          // 右前轮每圈脉冲数（测得）
// 新增：方向校验状态机
static int s_dir_test_state = 0;              // 0:待开始 1:前进中 2:前进结束 3:后退中 4:后退结束
static TickType_t s_dir_test_ts = 0;          // 当前阶段起始 tick
static int s_dir_test_start_cnt = 0;          // 阶段起始计数
static bool s_dir_test_enable = ROBOT_DIR_TEST_ENABLE;
#if MOTOR2_SPEED_TEST_ENABLE
static bool s_motor2_test_started = false;    // MOTOR2 70%占空测试是否已启动
#endif
// 新增：右前轮速度 PID 控制器状态
#if MOTOR2_PID_TEST_ENABLE
static pid_controller_t s_pid_fl, s_pid_fr, s_pid_rl, s_pid_rr;  // 四轮 PID 控制器
static bool s_pid_all_inited = false;                             // 是否已完成四轮 PID 初始化
static TickType_t s_pid_last_tick = 0;                            // 统一 PID 采样周期计时
#endif

// WiFi连接状态和重试计数
static int s_wifi_retry_num = 0;
static const int WIFI_MAXIMUM_RETRY = 5;
static bool s_wifi_connected = false;

// WIFI Related Function
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi 已启动，尝试连接到 SSID: %s", WIFI_SSID);
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "已连接到 SSID: %s", WIFI_SSID);
        s_wifi_retry_num = 0;  // 重置重试计数
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "WiFi断开连接，原因: %d", disconnected->reason);
        
        s_wifi_connected = false;
        if (s_wifi_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_wifi_retry_num++;
            ESP_LOGI(TAG, "重试连接到AP，第 %d/%d 次", s_wifi_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            ESP_LOGE(TAG, "连接到AP失败，已达到最大重试次数");
        }
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "已获取 IP 地址: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        s_wifi_retry_num = 0;
    }
}

static esp_err_t wifi_init_sta(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "初始化 WiFi 作为 Station...");
    ESP_LOGI(TAG, "目标 SSID: %s", WIFI_SSID);
    
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init 失败: 0x%x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "网络接口初始化成功");

    // 本段代码属于 Wi-Fi Station 初始化流程的一部分：
    // 1. 先调用 esp_netif_init() 完成 TCP/IP 协议栈与网络接口的初始化；
    // 2. 随后创建默认事件循环 esp_event_loop_create_default()，为后续 Wi-Fi 事件（连接、断开、获取 IP 等）提供统一分发机制；
    // 3. 最后通过 esp_wifi_register_event_handler() 把自定义的 wifi_event_handler 注册到系统，
    //    确保所有 Wi-Fi 相关事件（如 STA_START、STA_CONNECTED、STA_DISCONNECTED、GOT_IP）都能被捕获并自动处理重连/打印信息。
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_loop_create_default 失败: 0x%x", ret);
        return ret;
    } 
    ESP_LOGI(TAG, "事件循环创建成功");

    // 创建默认WiFi STA网络接口
    esp_netif_create_default_wifi_sta();
    ESP_LOGI(TAG, "WiFi STA网络接口创建成功");

    // 初始化WiFi
    wifi_init_config_t s_wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&s_wifi_init_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: 0x%x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "WiFi驱动初始化成功");

    // 注册 WiFi 事件回调函数，所有 WiFi 事件都会触发 wifi_event_handler
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_register WIFI_EVENT 失败: 0x%x", ret);
        return ret;
    }
    
    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_handler_register IP_EVENT 失败: 0x%x", ret);
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode 失败: 0x%x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "WiFi模式设置为STA成功");

    wifi_config_t s_wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,  // 支持更多认证模式
            .pmf_cfg = {
                .capable = true,
                .required = false,  // 不强制要求PMF，提高兼容性
            },
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .failure_retry_cnt = 3,  // 连接失败重试次数
        },
    };

    ESP_LOGI(TAG, "配置WiFi参数 - SSID: %s, 认证模式: WPA/WPA2-PSK", WIFI_SSID);
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &s_wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config 失败: 0x%x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "WiFi配置设置成功");

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: 0x%x", ret);
        return ret;
    }
    ESP_LOGI(TAG, "WiFi启动成功，等待连接事件...");

    ESP_LOGI(TAG, "WiFi 初始化完成，SSID: %s", WIFI_SSID);
    return ret;
}

// 检查WiFi连接状态
static bool is_wifi_connected(void)
{
    return s_wifi_connected;
}

// 重新启动WiFi连接
static void restart_wifi_connection(void)
{
    ESP_LOGI(TAG, "重新启动WiFi连接...");
    s_wifi_retry_num = 0;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(1000));  // 等待1秒
    esp_wifi_connect();
}

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

  ESP_LOGI(TAG, "WiFi 初始化...");
  ret = wifi_init_sta();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi 初始化失败: 0x%x", ret);
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

  // MOTOR 初始化（LEDC 低速模式，5kHz，13-bit），GPIO映射来自用户提供
  ESP_LOGI(TAG, "MOTOR 初始化...");
  ret = motor_init(&s_motor1, GPIO_NUM_4,  GPIO_NUM_5,  LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
  if (ret != ESP_OK) ESP_LOGE(TAG, "Motor1 初始化失败: %s", esp_err_to_name(ret));
  ret = motor_init(&s_motor2, GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
  if (ret != ESP_OK) ESP_LOGE(TAG, "Motor2 初始化失败: %s", esp_err_to_name(ret));
  ret = motor_init(&s_motor3, GPIO_NUM_8,  GPIO_NUM_3,  LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
  if (ret != ESP_OK) ESP_LOGE(TAG, "Motor3 初始化失败: %s", esp_err_to_name(ret));
  ret = motor_init(&s_motor4, GPIO_NUM_46, GPIO_NUM_9,  LEDC_CHANNEL_6, LEDC_CHANNEL_7, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
  if (ret != ESP_OK) ESP_LOGE(TAG, "Motor4 初始化失败: %s", esp_err_to_name(ret));

#if MOTOR_DEMO_DRIVE_MODE_BRAKE_DEFAULT
  motor_set_drive_mode(&s_motor1, MOTOR_DRIVE_BRAKE);
  motor_set_drive_mode(&s_motor2, MOTOR_DRIVE_BRAKE);
  motor_set_drive_mode(&s_motor3, MOTOR_DRIVE_BRAKE);
  motor_set_drive_mode(&s_motor4, MOTOR_DRIVE_BRAKE);
#else
  motor_set_drive_mode(&s_motor1, MOTOR_DRIVE_COAST);
  motor_set_drive_mode(&s_motor2, MOTOR_DRIVE_COAST);
  motor_set_drive_mode(&s_motor3, MOTOR_DRIVE_COAST);
  motor_set_drive_mode(&s_motor4, MOTOR_DRIVE_COAST);
#endif

  // 默认全部空转停止
  motor_stop_freewheel(&s_motor1);
  motor_stop_freewheel(&s_motor2);
  motor_stop_freewheel(&s_motor3);
  motor_stop_freewheel(&s_motor4);

  // 初始化统一驱动对象并保持停止
  robot_init_drive(&s_rb, &MOTOR_FL, &MOTOR_FR, &MOTOR_RL, &MOTOR_RR, MOTOR_POL_FL, MOTOR_POL_FR, MOTOR_POL_RL, MOTOR_POL_RR);
  robot_stop(&s_rb);

  // 创建右前轮编码器（使用宏定义，便于后期修改）
#if ENCODER_FR_ENABLE
  s_enc_fr = create_rotary_encoder(ENC_FR_PCNT_UNIT, ENC_FR_GPIO_A, ENC_FR_GPIO_B);
#else
  s_enc_fr = NULL;
#endif
  if (s_enc_fr) {
    ESP_LOGI(TAG, "右前轮编码器创建成功: PCNT_UNIT_%d, A=GPIO%u, B=GPIO%u", (int)ENC_FR_PCNT_UNIT, (unsigned)ENC_FR_GPIO_A, (unsigned)ENC_FR_GPIO_B);
    // 根据用户测量：10圈共 19741 脉冲 -> 每圈 ≈ 1974.1，四舍五入为 1974
    int counts_per_rev = (int)lroundf(19741.0f / 10.0f);
    s_counts_per_rev_cfg = counts_per_rev;
    esp_err_t cwret = s_enc_fr->config_wheel(s_enc_fr, 65.0f, counts_per_rev);
    if (cwret == ESP_OK) {
      ESP_LOGI(TAG, "轮速换算配置: 直径=65mm, counts_per_rev=%d (来源: 10圈测得19741脉冲)", counts_per_rev);
    } else {
      ESP_LOGE(TAG, "轮速换算配置失败: %s", esp_err_to_name(cwret));
    }
  } else {
    ESP_LOGE(TAG, "右前轮编码器创建失败，后续速度/方向打印不可用");
  }

  // 可选：创建左前/左后/右后编码器（需在顶部填写 GPIO 与 PCNT 单元并将 ENABLE 置1）
#if ENCODER_FL_ENABLE
  s_enc_fl = create_rotary_encoder(ENC_FL_PCNT_UNIT, ENC_FL_GPIO_A, ENC_FL_GPIO_B);
  if (s_enc_fl) {
    esp_err_t cwret = s_enc_fl->config_wheel(s_enc_fl, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(19741.0f / 10.0f));
    ESP_LOGI(TAG, "左前轮编码器创建成功: unit=%d, A=%d, B=%d", (int)ENC_FL_PCNT_UNIT, (int)ENC_FL_GPIO_A, (int)ENC_FL_GPIO_B);
    if (cwret != ESP_OK) {
      ESP_LOGW(TAG, "左前轮速度换算配置失败: %s", esp_err_to_name(cwret));
    }
  } else {
    ESP_LOGE(TAG, "左前轮编码器创建失败（请检查 PCNT 单元与 GPIO 配置）");
  }
#endif

#if ENCODER_RL_ENABLE
  s_enc_rl = create_rotary_encoder(ENC_RL_PCNT_UNIT, ENC_RL_GPIO_A, ENC_RL_GPIO_B);
  if (s_enc_rl) {
    esp_err_t cwret = s_enc_rl->config_wheel(s_enc_rl, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(19741.0f / 10.0f));
    ESP_LOGI(TAG, "左后轮编码器创建成功: unit=%d, A=%d, B=%d", (int)ENC_RL_PCNT_UNIT, (int)ENC_RL_GPIO_A, (int)ENC_RL_GPIO_B);
    if (cwret != ESP_OK) {
      ESP_LOGW(TAG, "左后轮速度换算配置失败: %s", esp_err_to_name(cwret));
    }
  } else {
    ESP_LOGE(TAG, "左后轮编码器创建失败（请检查 PCNT 单元与 GPIO 配置）");
  }
#endif

#if ENCODER_RR_ENABLE
  s_enc_rr = create_rotary_encoder(ENC_RR_PCNT_UNIT, ENC_RR_GPIO_A, ENC_RR_GPIO_B);
  if (s_enc_rr) {
    esp_err_t cwret = s_enc_rr->config_wheel(s_enc_rr, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(19741.0f / 10.0f));
    ESP_LOGI(TAG, "右后轮编码器创建成功: unit=%d, A=%d, B=%d", (int)ENC_RR_PCNT_UNIT, (int)ENC_RR_GPIO_A, (int)ENC_RR_GPIO_B);
    if (cwret != ESP_OK) {
      ESP_LOGW(TAG, "右后轮速度换算配置失败: %s", esp_err_to_name(cwret));
    }
  } else {
    ESP_LOGE(TAG, "右后轮编码器创建失败（请检查 PCNT 单元与 GPIO 配置）");
  }
#endif
  s_enc_last_tick = xTaskGetTickCount();

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

    // 新增：编码器速度/方向打印（右前轮）
    if (s_enc_fr) {
      TickType_t now_enc = xTaskGetTickCount();
      float dt_enc = ((float)(now_enc - s_enc_last_tick)) * (portTICK_PERIOD_MS / 1000.0f);
      s_enc_last_tick = now_enc;
      int cnt = s_enc_fr->get_counter_value(s_enc_fr);
      int delta = cnt - s_enc_last_cnt;
      s_enc_last_cnt = cnt;
      float pps = (dt_enc > 0.0f) ? (delta / dt_enc) : 0.0f;  // pulses per second
      float v_mps = s_enc_fr->get_speed_mps(s_enc_fr);        // 轮速（m/s），带符号
      const char *dir = (delta > 0) ? "DIR+" : (delta < 0) ? "DIR-" : "STOP";
      ESP_LOGI(TAG, "FR Enc: cnt=%d delta=%d speed=%.1fpps v=%.3fm/s %s", cnt, delta, pps, v_mps, dir);
    }

    // 额外打印：各轮速度（若已配置编码器）
    {
      float v_fl = (s_enc_fl) ? s_enc_fl->get_speed_mps(s_enc_fl) : 0.0f;
      float v_fr = (s_enc_fr) ? s_enc_fr->get_speed_mps(s_enc_fr) : 0.0f;
      float v_rl = (s_enc_rl) ? s_enc_rl->get_speed_mps(s_enc_rl) : 0.0f;
      float v_rr = (s_enc_rr) ? s_enc_rr->get_speed_mps(s_enc_rr) : 0.0f;
      // 只有在至少一个编码器存在时打印该汇总行
      if (s_enc_fl || s_enc_fr || s_enc_rl || s_enc_rr) {
        ESP_LOGI(TAG, "Wheels v[m/s]: FL=%s%.3f FR=%s%.3f RL=%s%.3f RR=%s%.3f",
                 (s_enc_fl ? "" : "N/A:"), v_fl,
                 (s_enc_fr ? "" : "N/A:"), v_fr,
                 (s_enc_rl ? "" : "N/A:"), v_rl,
                 (s_enc_rr ? "" : "N/A:"), v_rr);
        ESP_LOGI(TAG, "Encoders present: FL=%d FR=%d RL=%d RR=%d",
                 s_enc_fl ? 1 : 0, s_enc_fr ? 1 : 0, s_enc_rl ? 1 : 0, s_enc_rr ? 1 : 0);
      }
    }

#if MOTOR2_SPEED_TEST_ENABLE
    // 一键测试：MOTOR2(右前轮)占空比70%，并打印轮速
    if (!s_motor2_test_started) {
      motor_set_permille(&s_motor2, 700);
      s_motor2_test_started = true;
      ESP_LOGI(TAG, "MOTOR2 70%%占空测试启动: permille=700 (右前轮)");
    }
    if (s_enc_fr) {
      float v_test = s_enc_fr->get_speed_mps(s_enc_fr);
      ESP_LOGI(TAG, "MOTOR2 TEST: wheel speed v=%.3f m/s", v_test);
    } else {
      ESP_LOGW(TAG, "MOTOR2 TEST: 未检测到右前轮编码器，无法打印轮速");
    }
#endif

#if MOTOR2_PID_TEST_ENABLE
    // 四轮速度 PID 闭环控制（目标速度 PID_TARGET_SPEED_MPS）
    if (!s_pid_all_inited) {
      // 输出限幅：电机 permille [-1000, 1000]；积分限幅缩小一些避免长时间饱和
      pid_init(&s_pid_fl, PID_KP, PID_KI, PID_KD, -1000.0f, +1000.0f, -800.0f, +800.0f);
      pid_init(&s_pid_fr, PID_KP, PID_KI, PID_KD, -1000.0f, +1000.0f, -800.0f, +800.0f);
      pid_init(&s_pid_rl, PID_KP, PID_KI, PID_KD, -1000.0f, +1000.0f, -800.0f, +800.0f);
      pid_init(&s_pid_rr, PID_KP, PID_KI, PID_KD, -1000.0f, +1000.0f, -800.0f, +800.0f);
      // 为提升鲁棒性，导数项使用测量值并加一阶低通滤波
      pid_set_d_on_measurement(&s_pid_fl, true); pid_set_d_filter_alpha(&s_pid_fl, 0.9f);
      pid_set_d_on_measurement(&s_pid_fr, true); pid_set_d_filter_alpha(&s_pid_fr, 0.9f);
      pid_set_d_on_measurement(&s_pid_rl, true); pid_set_d_filter_alpha(&s_pid_rl, 0.9f);
      pid_set_d_on_measurement(&s_pid_rr, true); pid_set_d_filter_alpha(&s_pid_rr, 0.9f);
      s_pid_last_tick = xTaskGetTickCount();
      s_pid_all_inited = true;
      ESP_LOGI(TAG, "四轮 PID 启动: target=%.3f m/s, Kp=%.1f Ki=%.1f Kd=%.1f", PID_TARGET_SPEED_MPS, PID_KP, PID_KI, PID_KD);
    }
    // 计算 dt
    TickType_t now_pid = xTaskGetTickCount();
    float dt_pid = ((float)(now_pid - s_pid_last_tick)) * (portTICK_PERIOD_MS / 1000.0f);
    s_pid_last_tick = now_pid;
    // 测量各轮速度（若未配置编码器，则临时采用右前轮速度作为反馈以保持一致）
    float v_fr = s_enc_fr ? s_enc_fr->get_speed_mps(s_enc_fr) : 0.0f;
    float v_fl = s_enc_fl ? s_enc_fl->get_speed_mps(s_enc_fl) : v_fr;
    float v_rl = s_enc_rl ? s_enc_rl->get_speed_mps(s_enc_rl) : v_fr;
    float v_rr = s_enc_rr ? s_enc_rr->get_speed_mps(s_enc_rr) : v_fr;
    // 计算四轮控制输出
    int out_fl = (int)lroundf(pid_compute(&s_pid_fl, PID_TARGET_SPEED_MPS, v_fl, dt_pid));
    int out_fr = (int)lroundf(pid_compute(&s_pid_fr, PID_TARGET_SPEED_MPS, v_fr, dt_pid));
    int out_rl = (int)lroundf(pid_compute(&s_pid_rl, PID_TARGET_SPEED_MPS, v_rl, dt_pid));
    int out_rr = (int)lroundf(pid_compute(&s_pid_rr, PID_TARGET_SPEED_MPS, v_rr, dt_pid));
    // 输出限幅到 [-1000, 1000]，分行书写避免 -Werror=misleading-indentation
    if (out_fl > 1000) out_fl = 1000;
    if (out_fl < -1000) out_fl = -1000;
    if (out_fr > 1000) out_fr = 1000;
    if (out_fr < -1000) out_fr = -1000;
    if (out_rl > 1000) out_rl = 1000;
    if (out_rl < -1000) out_rl = -1000;
    if (out_rr > 1000) out_rr = 1000;
    if (out_rr < -1000) out_rr = -1000;
    // 分别驱动四个轮子（统一驱动对象的极性映射由此体现）
    motor_set_permille(&MOTOR_FL, MOTOR_POL_FL * out_fl);
    motor_set_permille(&MOTOR_FR, MOTOR_POL_FR * out_fr);
    motor_set_permille(&MOTOR_RL, MOTOR_POL_RL * out_rl);
    motor_set_permille(&MOTOR_RR, MOTOR_POL_RR * out_rr);
    ESP_LOGI(TAG, "PID 4W: target=%.3f m/s | v_FL=%.3f v_FR=%.3f v_RL=%.3f v_RR=%.3f | out_FL/FR/RL/RR=%d/%d/%d/%d‰",
             PID_TARGET_SPEED_MPS, v_fl, v_fr, v_rl, v_rr, out_fl, out_fr, out_rl, out_rr);
#endif
    // 新增：自动方向校验（一次性）
    if (s_dir_test_enable && s_enc_fr) {
      switch (s_dir_test_state) {
        case 0: {
          // 启动前进 2s
          s_dir_test_start_cnt = s_enc_fr->get_counter_value(s_enc_fr);
          robot_drive_forward(&s_rb, 300);
          s_dir_test_ts = xTaskGetTickCount();
          ESP_LOGI(TAG, "方向校验: 前进测试开始 (300/1000)");
          s_dir_test_state = 1;
          break;
        }
        case 1: {
          TickType_t now = xTaskGetTickCount();
          if ((now - s_dir_test_ts) >= pdMS_TO_TICKS(2000)) {
            int end_cnt = s_enc_fr->get_counter_value(s_enc_fr);
            int delta = end_cnt - s_dir_test_start_cnt;
            float dt = ((float)(now - s_dir_test_ts)) * (portTICK_PERIOD_MS / 1000.0f);
            float pps = (dt > 0.0f) ? (delta / dt) : 0.0f;
            float avg_mps = 0.0f;
            if (s_counts_per_rev_cfg > 0 && dt > 0.0f) {
              float circumference_m = (float)M_PI * 0.065f; // 直径 65mm -> 0.065m
              avg_mps = ((float)delta / (float)s_counts_per_rev_cfg) * circumference_m / dt;
            }
            ESP_LOGI(TAG, "方向校验: 前进2s 结果 delta=%d speed=%.1fpps avg_v=%.3fm/s (正负号即方向)", delta, pps, avg_mps);
            robot_stop(&s_rb);
            s_dir_test_ts = xTaskGetTickCount();
            s_dir_test_state = 2;
          }
          break;
        }
        case 2: {
          // 启动后退 2s
          s_dir_test_start_cnt = s_enc_fr->get_counter_value(s_enc_fr);
          robot_drive_backward(&s_rb, 300);
          s_dir_test_ts = xTaskGetTickCount();
          ESP_LOGI(TAG, "方向校验: 后退测试开始 (300/1000)");
          s_dir_test_state = 3;
          break;
        }
        case 3: {
          TickType_t now = xTaskGetTickCount();
          if ((now - s_dir_test_ts) >= pdMS_TO_TICKS(2000)) {
            int end_cnt = s_enc_fr->get_counter_value(s_enc_fr);
            int delta = end_cnt - s_dir_test_start_cnt;
            float dt = ((float)(now - s_dir_test_ts)) * (portTICK_PERIOD_MS / 1000.0f);
            float pps = (dt > 0.0f) ? (delta / dt) : 0.0f;
            float avg_mps = 0.0f;
            if (s_counts_per_rev_cfg > 0 && dt > 0.0f) {
              float circumference_m = (float)M_PI * 0.065f; // 直径 65mm -> 0.065m
              avg_mps = ((float)delta / (float)s_counts_per_rev_cfg) * circumference_m / dt;
            }
            ESP_LOGI(TAG, "方向校验: 后退2s 结果 delta=%d speed=%.1fpps avg_v=%.3fm/s (正负号即方向)", delta, pps, avg_mps);
            robot_stop(&s_rb);
            s_dir_test_state = 4;
            s_dir_test_enable = false;  // 完成一次校验
            ESP_LOGI(TAG, "方向校验完成");
          }
          break;
        }
        default:
          break;
      }
    }

    // WiFi连接状态监控和重连机制
    static TickType_t last_wifi_check = 0;
    TickType_t current_tick = xTaskGetTickCount();
    
    // 每10秒检查一次WiFi连接状态
    if ((current_tick - last_wifi_check) >= pdMS_TO_TICKS(10000)) {
        last_wifi_check = current_tick;
        
        if (!is_wifi_connected()) {
            ESP_LOGW(TAG, "WiFi未连接，尝试重新连接...");
            restart_wifi_connection();
        } else {
            ESP_LOGI(TAG, "WiFi连接正常");
        }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
