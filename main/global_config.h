#ifndef __GLOBAL_CONFIG_H__
#define __GLOBAL_CONFIG_H__

// WIFI Configuration
#define WIFI_SSID "CEG-EDWARD-NB-1124"
#define WIFI_PASS "yx341563"

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

// 每个轮位的“逻辑正转”极性（+1：permille>0 为正转；-1：permille<0 为正转）（保留以支持后续方向控制）
#define MOTOR_POL_FL (-1)
#define MOTOR_POL_FR (+1)
#define MOTOR_POL_RL (-1)
#define MOTOR_POL_RR (+1)

// PID Configuration
// 用户测试宏：置1启用车轮速度 PID 闭环控制（目标速度 m/s）。
#define MOTOR2_PID_TEST_ENABLE                0
// PID 目标速度（m/s），请根据场景安全设置，初值偏保守
#define PID_TARGET_SPEED_MPS                  0.5f
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
  #define ENC_FR_GPIO_A     GPIO_NUM_18
  #define ENC_FR_GPIO_B     GPIO_NUM_17
  #define FR_PCNT_TEN_CIRCLE_COUNT 19834
#endif

// 左前轮（Motor1）编码器：A=GPIO7, B=GPIO6，建议使用 PCNT_UNIT_1
#define ENCODER_FL_ENABLE 1
#if ENCODER_FL_ENABLE
  #define ENC_FL_PCNT_UNIT  PCNT_UNIT_1
  #define ENC_FL_GPIO_A     GPIO_NUM_7
  #define ENC_FL_GPIO_B     GPIO_NUM_6
  #define FL_PCNT_TEN_CIRCLE_COUNT 19618
#endif

// 左后轮（Motor3）编码器：A=GPIO19, B=GPIO20，建议使用 PCNT_UNIT_2
#define ENCODER_RL_ENABLE 1
#if ENCODER_RL_ENABLE
  #define ENC_RL_PCNT_UNIT  PCNT_UNIT_2
  #define ENC_RL_GPIO_A     GPIO_NUM_19
  #define ENC_RL_GPIO_B     GPIO_NUM_20
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