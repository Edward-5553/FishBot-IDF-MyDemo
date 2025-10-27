// 四轮差速（skid-steer）运动学正/逆解
// 提供以“轮线速度 m/s”和“轮角速度 rad/s”为输入/输出的两套接口。
// 约定：
// - FL: 左前，FR: 右前，RL: 左后，RR: 右后。
// - 四轮差速认为左右两侧各两轮速度应一致（地面无侧向滑移时），但接口允许不一致并做平均。
// - 车体用机器人坐标系表示：vx 为前进速度（m/s），vy=0（理想差速车无横向自由度），omega 为绕竖直轴的角速度（rad/s）。

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float track_width_m;   // 左右车轮中心线间距（m），又称“轮距/轴距”中的横向轮距
    float wheel_radius_m;  // 车轮半径（m），用于角速度接口；若仅用线速度接口可置为 0
} diff4_kinematics_cfg_t;

// —— 正解（Forward Kinematics）——
// 输入四个车轮线速度（m/s），输出机器人前进速度 vx 与角速度 omega。
// vy 在理想差速模型中为 0，可忽略。
void diff4_forward_mps(const diff4_kinematics_cfg_t *cfg,
                       float v_fl, float v_fr, float v_rl, float v_rr,
                       float *vx_out, float *omega_out);

// 输入四个车轮角速度（rad/s），输出机器人前进速度 vx 与角速度 omega。
void diff4_forward_rads(const diff4_kinematics_cfg_t *cfg,
                        float w_fl, float w_fr, float w_rl, float w_rr,
                        float *vx_out, float *omega_out);

// —— 逆解（Inverse Kinematics）——
// 输入机器人期望的 vx（m/s）和 omega（rad/s），输出四个车轮线速度（m/s）。
// 典型差速情况下：左右两侧速度分别相等（返回同侧两轮相同值）。
void diff4_inverse_mps(const diff4_kinematics_cfg_t *cfg,
                       float vx, float omega,
                       float *v_fl_out, float *v_fr_out, float *v_rl_out, float *v_rr_out);

// 输入机器人期望的 vx（m/s）和 omega（rad/s），输出四个车轮角速度（rad/s）。
void diff4_inverse_rads(const diff4_kinematics_cfg_t *cfg,
                        float vx, float omega,
                        float *w_fl_out, float *w_fr_out, float *w_rl_out, float *w_rr_out);

// —— 常用辅助 ——
// 将角速度（rad/s）转换为线速度（m/s）
static inline float wheel_radps_to_mps(float radps, float radius_m) {
    return radps * radius_m;
}
// 将线速度（m/s）转换为角速度（rad/s）
static inline float wheel_mps_to_radps(float mps, float radius_m) {
    return (radius_m > 0.0f) ? (mps / radius_m) : 0.0f;
}

#ifdef __cplusplus
}
#endif