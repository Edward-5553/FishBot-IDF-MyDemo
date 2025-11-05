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
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

////// Component Part >>>>>>
#include "led.h"
#include "myi2c.h"
#include "mpu6050.h"
#include "oled.h"
#include "motor.h"
#include "rotary_encoder.h"
#include "pid_controller.h"
#include "driver/pcnt.h"
#include "kinematics_diff4.h"
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
static rcl_subscription_t cmd_vel_subscriber;

static geometry_msgs__msg__Twist velocity_msg;
static geometry_msgs__msg__Twist cmd_vel_twist;

static float robot_linear_x = 0.0f; // 机器人线速度（x 轴）  // 沿 x 轴
static float robot_angular_z = 0.0f; // 机器人角速度（z 轴） // 绕 z 轴

/* IMU 位姿滤波状态（Roll/Pitch/Yaw，单位：度） */
static float s_roll = 0.0f, s_pitch = 0.0f, s_yaw = 0.0f;
/* FreeRTOS Tick 计时，用于积分 */
static TickType_t s_last_tick = 0;
/* 是否完成位姿初始对准（用加速度静态解算进行初始化） */
static bool s_pose_initialized = false;
// static bool s_oled_ready = false;  // 暂时未使用，注释掉避免警告
static float s_gbias_x = 0.0f, s_gbias_y = 0.0f, s_gbias_z = 0.0f; /* 陀螺零速偏移（dps） */
static motor_t s_motor1, s_motor2, s_motor3, s_motor4;            /* 四路电机对象 */
// 轮位映射：motor1=左前, motor2=右前, motor3=左后, motor4=右后（保留以支持后续方向控制）
#define MOTOR_FL s_motor1 // 左前 Front-Left
#define MOTOR_FR s_motor2 // 右前 Front-Right
#define MOTOR_RL s_motor3 // 左后 Rear-Left
#define MOTOR_RR s_motor4 // 右后 Rear-Right

// 新增：统一车辆驱动对象 + 编码器对象（示例：右前轮）
static robot_drive_t s_rb;                    // 统一驱动对象
static rotary_encoder_t *s_enc_fr = NULL;     // 右前轮编码器（宏定义：ENC_FR_GPIO_A/ENC_FR_GPIO_B/ENC_FR_PCNT_UNIT）
static rotary_encoder_t *s_enc_fl = NULL;     // 左前轮编码器（需填写 A/B 引脚）
static rotary_encoder_t *s_enc_rl = NULL;     // 左后轮编码器（需填写 A/B 引脚）
static rotary_encoder_t *s_enc_rr = NULL;     // 右后轮编码器（需填写 A/B 引脚）
static TickType_t s_enc_last_tick = 0;        // 上次计数时间戳
static int s_counts_per_rev_cfg = 0;          // 右前轮每圈脉冲数（测得）

// WiFi连接状态和重试计数
static int s_wifi_retry_num = 0;
static const int WIFI_MAXIMUM_RETRY = 5;
static bool s_wifi_connected = false;

#if MOTOR2_PID_TEST_ENABLE
static pid_controller_t s_pid_fl, s_pid_fr, s_pid_rl, s_pid_rr;  // 四轮 PID 控制器
static bool s_pid_all_inited = false;                             // 是否已完成四轮 PID 初始化
static TickType_t s_pid_last_tick = 0;                            // 统一 PID 采样周期计时
#endif

diff4_kinematics_cfg_t diff4_kinematics_cfg = {
    .track_width_m = TRACK_WIDTH_M,
    .wheel_radius_m = WHEEL_RADIUS_M,
};

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

// Subscribe the /cmd_vel topic
static void cmd_vel_callback(const void * msgin)
{
    float linear_x = 0.0f;
    float angular_z = 0.0f;
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;
    ESP_LOGI(TAG, "收到速度指令: vx=%.3f, omega=%.3f", linear_x, angular_z);
    // 如需联动驱动层，可在此触发目标速度更新（当前保持只赋值，不改变驱动逻辑）
}

// Publish velocity message to micro-ROS
static void publish_velocity(float vx, float omega)
{
    //velocity_msg
    velocity_msg.linear.x = vx;
    velocity_msg.linear.y = 0.0;
    velocity_msg.linear.z = 0.0;
    velocity_msg.angular.x = 0.0;
    velocity_msg.angular.y = 0.0;
    velocity_msg.angular.z = omega;

    rcl_ret_t ret = rcl_publish(&publisher, &velocity_msg, NULL);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "micro-ROS 发布速度消息失败: 0x%x", ret);
    }
}

// MicroROS Related Function
static bool microros_init(void)
{
    ESP_LOGI(TAG, "初始化 micro-ROS...");
    
    // 等待WiFi连接完成
    int retry_count = 0;
    while (!is_wifi_connected() && retry_count < 30) {
        ESP_LOGI(TAG, "等待WiFi连接... (%d/30)", retry_count + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
    }
    
    if (!is_wifi_connected()) {
        ESP_LOGE(TAG, "WiFi未连接，无法初始化micro-ROS");
        return false;
    }
    
    // 初始化分配器
    allocator = rcl_get_default_allocator();
    
    // 初始化选项
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
        ESP_LOGE(TAG, "rcl_init_options_init 失败");
        return false;
    }
    
    // 设置UDP地址
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_uros_options_set_udp_address(MICROROS_AGENT_IP, MICROROS_AGENT_PORT, rmw_options) != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_options_set_udp_address 失败");
        return false;
    }
    
    // 初始化支持结构
    if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init_with_options 失败");
        return false;
    }
    
    // 创建节点
    if (rclc_node_init_default(&node, "fishbot_velocity_publisher", "", &support) != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_node_init_default 失败");
        return false;
    }
    
    // 创建发布者（设备反馈话题）
    if (rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/robot/velocity") != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_publisher_init_default 失败");
        return false;
    }
    
    // 创建订阅者（命令话题）
    if (rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_subscription_init_default 失败");
        return false;
    }
    
    // 初始化执行器
    if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_executor_init 失败");
        return false;
    }

    // 执行器添加订阅
    if (rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_twist,
        &cmd_vel_callback,
        ON_NEW_DATA) != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_executor_add_subscription 失败");
        return false;
    }
    
    ESP_LOGI(TAG, "micro-ROS 初始化成功");
    return true;
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

  ESP_LOGI(TAG, "应用启动, 开发IDF版本: v5.5.1, 当前IDF版本: %s", esp_get_idf_version());

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
  } else {
    ESP_LOGI(TAG, "WiFi 连接成功，初始化 micro-ROS...");
    if (!microros_init()) {
      ESP_LOGE(TAG, "micro-ROS 初始化失败");
    }
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
  /*
  ESP_LOGI(TAG, "OLED 初始化...");
  if (oled_init()) {
    s_oled_ready = true;
    ESP_LOGI(TAG, "OLED 初始化成功");
    oled_clear();
  } else {
    ESP_LOGE(TAG, "OLED 初始化失败");
  }
  */

  // MOTOR 初始化（LEDC 低速模式，5kHz，13-bit），GPIO映射来自用户提供
  ESP_LOGI(TAG, "MOTOR 初始化...");
  ret = motor_init(&s_motor1, GPIO_NUM_5,  GPIO_NUM_4,  LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
  if (ret != ESP_OK) ESP_LOGE(TAG, "Motor1 初始化失败: %s", esp_err_to_name(ret));
  ret = motor_init(&s_motor2, GPIO_NUM_15, GPIO_NUM_16, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
  if (ret != ESP_OK) ESP_LOGE(TAG, "Motor2 初始化失败: %s", esp_err_to_name(ret));
  ret = motor_init(&s_motor3, GPIO_NUM_3,  GPIO_NUM_8,  LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, PWM_DEFAULT_FREQ_HZ, PWM_DEFAULT_RESOLUTION);
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

  // 初始化统一驱动对象并保持停止（双相PWM已保证方向一致性，无需极性参数）
  robot_init_drive(&s_rb, &MOTOR_FL, &MOTOR_FR, &MOTOR_RL, &MOTOR_RR);
  robot_stop(&s_rb);

#if ENCODER_FL_ENABLE
  s_enc_fl = create_rotary_encoder(ENC_FL_PCNT_UNIT, ENC_FL_GPIO_A, ENC_FL_GPIO_B);
  if (s_enc_fl) {
    esp_err_t cwret = s_enc_fl->config_wheel(s_enc_fl, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(FL_PCNT_TEN_CIRCLE_COUNT / 10.0f));
    ESP_LOGI(TAG, "左前轮编码器创建成功: unit=%d, A=%d, B=%d", (int)ENC_FL_PCNT_UNIT, (int)ENC_FL_GPIO_A, (int)ENC_FL_GPIO_B);
    if (cwret != ESP_OK) {
      ESP_LOGW(TAG, "左前轮速度换算配置失败: %s", esp_err_to_name(cwret));
    }
  } else {
    ESP_LOGE(TAG, "左前轮编码器创建失败（请检查 PCNT 单元与 GPIO 配置）");
  }
#endif

#if ENCODER_FR_ENABLE
  s_enc_fr = create_rotary_encoder(ENC_FR_PCNT_UNIT, ENC_FR_GPIO_A, ENC_FR_GPIO_B);
  if (s_enc_fr) {
    esp_err_t cwret = s_enc_fr->config_wheel(s_enc_fr, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(FR_PCNT_TEN_CIRCLE_COUNT / 10.0f));
    ESP_LOGI(TAG, "右前轮编码器创建成功: unit=%d, A=%d, B=%d", (int)ENC_FR_PCNT_UNIT, (int)ENC_FR_GPIO_A, (int)ENC_FR_GPIO_B);
    if (cwret != ESP_OK) {
      ESP_LOGW(TAG, "右前轮速度换算配置失败: %s", esp_err_to_name(cwret));
    }
  } else {
    ESP_LOGE(TAG, "右前轮编码器创建失败（请检查 PCNT 单元与 GPIO 配置）");
  }
#endif

#if ENCODER_RL_ENABLE
  s_enc_rl = create_rotary_encoder(ENC_RL_PCNT_UNIT, ENC_RL_GPIO_A, ENC_RL_GPIO_B);
  if (s_enc_rl) {
    esp_err_t cwret = s_enc_rl->config_wheel(s_enc_rl, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(RL_PCNT_TEN_CIRCLE_COUNT / 10.0f));
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
    esp_err_t cwret = s_enc_rr->config_wheel(s_enc_rr, 65.0f, s_counts_per_rev_cfg > 0 ? s_counts_per_rev_cfg : (int)lroundf(RR_PCNT_TEN_CIRCLE_COUNT / 10.0f));
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

    // 根据电机编码器获取各电机转速
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
      }

      diff4_forward_mps(&diff4_kinematics_cfg, v_fl, v_fr, v_rl, v_rr, &robot_linear_x, &robot_angular_z);
      //diff4_forward_rads(&diff4_kinematics_cfg, robot_linear_x, robot_angular_z, &v_fl, &v_fr, &v_rl, &v_rr);
      publish_velocity(robot_linear_x, robot_angular_z);
      rclc_executor_spin_some(&executor, pdMS_TO_TICKS(10));
      ESP_LOGI(TAG, "Robot velocity: linear_x=%.3f m/s", robot_linear_x);
    }

#if MOTOR2_PID_TEST_ENABLE
    // 四轮速度 PID 闭环控制（目标速度 PID_TARGET_SPEED_MPS）
    if (!s_pid_all_inited) {
        // 输出限幅：电机 permille [-1000, 1000]；积分限幅缩小一些避免长时间饱和
        pid_init(&s_pid_fl, PID_KP, PID_KI, PID_KD, -3000.0f, +3000.0f, -800.0f, +800.0f);
        pid_init(&s_pid_fr, PID_KP, PID_KI, PID_KD, -3000.0f, +3000.0f, -800.0f, +800.0f);
        pid_init(&s_pid_rl, PID_KP, PID_KI, PID_KD, -3000.0f, +3000.0f, -800.0f, +800.0f);
        pid_init(&s_pid_rr, PID_KP, PID_KI, PID_KD, -3000.0f, +3000.0f, -800.0f, +800.0f);
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
    float v_fl = s_enc_fl ? s_enc_fl->get_speed_mps(s_enc_fl) : 0.0f;
    float v_rl = s_enc_rl ? s_enc_rl->get_speed_mps(s_enc_rl) : 0.0f;
    float v_rr = s_enc_rr ? s_enc_rr->get_speed_mps(s_enc_rr) : 0.0f;
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
    // 分别驱动四个轮子（双相PWM已保证方向一致性，无需极性修正）
    motor_set_permille(&MOTOR_FL, out_fl);
    motor_set_permille(&MOTOR_FR, out_fr);
    motor_set_permille(&MOTOR_RL, out_rl);
    motor_set_permille(&MOTOR_RR, out_rr);
    ESP_LOGI(TAG, "PID 4W: target=%.3f m/s | v_FL=%.3f v_FR=%.3f v_RL=%.3f v_RR=%.3f | out_FL/FR/RL/RR=%d/%d/%d/%d‰",
             PID_TARGET_SPEED_MPS, v_fl, v_fr, v_rl, v_rr, out_fl, out_fr, out_rl, out_rr);
#endif

#if 0
    motor_set_permille(&MOTOR_FL, MOTOR_POL_FL * 750);
    motor_set_permille(&MOTOR_FR, MOTOR_POL_FR * 750);
    motor_set_permille(&MOTOR_RL, MOTOR_POL_RL * 750);
    motor_set_permille(&MOTOR_RR, MOTOR_POL_RR * 750);
 

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
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
