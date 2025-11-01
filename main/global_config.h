#ifndef __GLOBAL_CONFIG_H__
#define __GLOBAL_CONFIG_H__

// WIFI Configuration
// 选择要使用的WiFi网络：取消注释对应的宏定义
#define USE_HOME_WIFI    // 使用家里WiFi
//#define USE_COMPANY_WIFI   // 使用公司WiFi

#ifdef USE_COMPANY_WIFI
#define WIFI_SSID "CEG-EDWARD-NB-1124"
#define WIFI_PASS "yx341563"
#elif defined(USE_HOME_WIFI)
// 请确认以下信息是否正确：
#define WIFI_SSID "Xiaomi_75D7"
#define WIFI_PASS "yx341563"
#else
// 默认WiFi配置（如果上面两个都未定义）
#define WIFI_SSID "Xiaomi_75D7"
#define WIFI_PASS "yx341563"
#endif
// MicroROS Configuration 
// Agent Configuration
#define MICROROS_AGENT_IP "192.168.137.85"
#define MICROROS_AGENT_PORT "8888"

// 新增：定义圆周率（若未定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 新增：参考 V4 驱动方式，默认使用 COAST（快衰减）以先确保能转动
#define MOTOR_DEMO_DRIVE_MODE_BRAKE_DEFAULT  0
// 用户测试宏：置1启用MOTOR2占空70%并打印轮速，置0编译移除该测试代码
#define MOTOR2_SPEED_TEST_ENABLE              0

// 注意：双相PWM GPIO已重新定义，正负占空比时电机转动方向一致，无需软件极性修正

// PID Configuration
// 用户测试宏：置1启用车轮速度 PID 闭环控制（目标速度 m/s）。
#define MOTOR2_PID_TEST_ENABLE                1
// PID 目标速度（m/s），请根据场景安全设置，初值偏保守
#define PID_TARGET_SPEED_MPS                  0.30f
// PID 初始增益（单位：输出permille/速度m/s），请按需要调参
#define PID_KP                                1500.0f
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
  #define ENC_FR_GPIO_A     GPIO_NUM_18
  #define ENC_FR_GPIO_B     GPIO_NUM_17
  #define FR_PCNT_TEN_CIRCLE_COUNT 19834
#endif

// 左前轮（Motor1）编码器：A=GPIO7, B=GPIO6，建议使用 PCNT_UNIT_1
#define ENCODER_FL_ENABLE 1
#if ENCODER_FL_ENABLE
  #define ENC_FL_PCNT_UNIT  PCNT_UNIT_1
  #define ENC_FL_GPIO_A     GPIO_NUM_6
  #define ENC_FL_GPIO_B     GPIO_NUM_7
  #define FL_PCNT_TEN_CIRCLE_COUNT 19618
#endif

// 左后轮（Motor3）编码器：A=GPIO19, B=GPIO20，建议使用 PCNT_UNIT_2
#define ENCODER_RL_ENABLE 1
#if ENCODER_RL_ENABLE
  #define ENC_RL_PCNT_UNIT  PCNT_UNIT_2
  #define ENC_RL_GPIO_A     GPIO_NUM_20
  #define ENC_RL_GPIO_B     GPIO_NUM_19
  #define RL_PCNT_TEN_CIRCLE_COUNT 20005
#endif

// 右后轮（Motor4）编码器：A=GPIO11, B=GPIO10，建议使用 PCNT_UNIT_3
#define ENCODER_RR_ENABLE 1
#if ENCODER_RR_ENABLE
  #define ENC_RR_PCNT_UNIT  PCNT_UNIT_3
  #define ENC_RR_GPIO_A     GPIO_NUM_11
  #define ENC_RR_GPIO_B     GPIO_NUM_10
  #define RR_PCNT_TEN_CIRCLE_COUNT 19689
#endif

// KINECT 配置
#define TRACK_WIDTH_M 0.25          // 25 cm
#define WHEEL_RADIUS_M 0.0325       // 32.5 mm

#endif