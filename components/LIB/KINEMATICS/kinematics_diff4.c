// 四轮差速（skid-steer）运动学实现
#include "kinematics_diff4.h"

// 正解：使用线速度（m/s）
void diff4_forward_mps(const diff4_kinematics_cfg_t *cfg,
                       float v_fl, float v_fr, float v_rl, float v_rr,
                       float *vx_out, float *omega_out)
{
    if (!cfg) {
        if (vx_out) *vx_out = 0.0f;
        if (omega_out) *omega_out = 0.0f;
        return;
    }
    // 左右侧取平均（允许前后不完全一致）
    float v_left  = 0.5f * (v_fl + v_rl);
    float v_right = 0.5f * (v_fr + v_rr);
    // 纵向速度：左右平均
    float vx = 0.5f * (v_left + v_right);
    // 角速度：左右差除以轮距
    float omega = (cfg->track_width_m > 0.0f) ? ((v_right - v_left) / cfg->track_width_m) : 0.0f;
    if (vx_out) *vx_out = vx;
    if (omega_out) *omega_out = omega;
}

// 正解：使用角速度（rad/s）
void diff4_forward_rads(const diff4_kinematics_cfg_t *cfg,
                        float w_fl, float w_fr, float w_rl, float w_rr,
                        float *vx_out, float *omega_out)
{
    if (!cfg || cfg->wheel_radius_m <= 0.0f) {
        if (vx_out) *vx_out = 0.0f;
        if (omega_out) *omega_out = 0.0f;
        return;
    }
    float v_fl = wheel_radps_to_mps(w_fl, cfg->wheel_radius_m);
    float v_fr = wheel_radps_to_mps(w_fr, cfg->wheel_radius_m);
    float v_rl = wheel_radps_to_mps(w_rl, cfg->wheel_radius_m);
    float v_rr = wheel_radps_to_mps(w_rr, cfg->wheel_radius_m);
    diff4_forward_mps(cfg, v_fl, v_fr, v_rl, v_rr, vx_out, omega_out);
}

// 逆解：期望机器人线/角速度 -> 四轮线速度（m/s）
void diff4_inverse_mps(const diff4_kinematics_cfg_t *cfg,
                       float vx, float omega,
                       float *v_fl_out, float *v_fr_out, float *v_rl_out, float *v_rr_out)
{
    if (!cfg || cfg->track_width_m <= 0.0f) {
        if (v_fl_out) *v_fl_out = 0.0f;
        if (v_fr_out) *v_fr_out = 0.0f;
        if (v_rl_out) *v_rl_out = 0.0f;
        if (v_rr_out) *v_rr_out = 0.0f;
        return;
    }
    // 差速模型：左右两侧线速度
    float v_left  = vx - 0.5f * omega * cfg->track_width_m;
    float v_right = vx + 0.5f * omega * cfg->track_width_m;
    if (v_fl_out) *v_fl_out = v_left;
    if (v_rl_out) *v_rl_out = v_left;
    if (v_fr_out) *v_fr_out = v_right;
    if (v_rr_out) *v_rr_out = v_right;
}

// 逆解：期望机器人线/角速度 -> 四轮角速度（rad/s）
void diff4_inverse_rads(const diff4_kinematics_cfg_t *cfg,
                        float vx, float omega,
                        float *w_fl_out, float *w_fr_out, float *w_rl_out, float *w_rr_out)
{
    if (!cfg || cfg->track_width_m <= 0.0f || cfg->wheel_radius_m <= 0.0f) {
        if (w_fl_out) *w_fl_out = 0.0f;
        if (w_fr_out) *w_fr_out = 0.0f;
        if (w_rl_out) *w_rl_out = 0.0f;
        if (w_rr_out) *w_rr_out = 0.0f;
        return;
    }
    float v_fl, v_fr, v_rl, v_rr;
    diff4_inverse_mps(cfg, vx, omega, &v_fl, &v_fr, &v_rl, &v_rr);
    if (w_fl_out) *w_fl_out = wheel_mps_to_radps(v_fl, cfg->wheel_radius_m);
    if (w_fr_out) *w_fr_out = wheel_mps_to_radps(v_fr, cfg->wheel_radius_m);
    if (w_rl_out) *w_rl_out = wheel_mps_to_radps(v_rl, cfg->wheel_radius_m);
    if (w_rr_out) *w_rr_out = wheel_mps_to_radps(v_rr, cfg->wheel_radius_m);
}