#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Adaptive.h"
#include <AP_Logger/AP_Logger.h>

// 静态成员定义
Matrix3f AC_CustomControl_Adaptive::_Upsilon_hat;
float AC_CustomControl_Adaptive::_theta_hat[6] = {0};
Vector3f AC_CustomControl_Adaptive::_d_hat;
bool AC_CustomControl_Adaptive::_initialized = false;

// 参数表
const AP_Param::GroupInfo AC_CustomControl_Adaptive::var_info[] = {
    AP_SUBGROUPINFO(_p_angle_roll,  "ANG_RLL_", 1, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 2, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_angle_yaw,   "ANG_YAW_", 3, AC_CustomControl_Adaptive, AC_P),
    AP_GROUPINFO("GAMMA",    4, AC_CustomControl_Adaptive, _gamma,    2.0f),
    AP_GROUPINFO("DEADZONE", 5, AC_CustomControl_Adaptive, _deadzone, 0.02f),
    AP_GROUPINFO("SIGMA",    6, AC_CustomControl_Adaptive, _sigma,    0.0f),
    AP_GROUPEND
};

AC_CustomControl_Adaptive::AC_CustomControl_Adaptive(AC_CustomControl& frontend,
                                                     AP_AHRS_View*& ahrs,
                                                     AC_AttitudeControl*& att_control,
                                                     AP_MotorsMulticopter*& motors,
                                                     float dt)
    : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
      _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P * 0.9f),
      _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P * 0.9f),
      _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P * 0.9f)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);

    if (!_initialized) {
        _Upsilon_hat.identity();
        memset(_theta_hat, 0, sizeof(_theta_hat));
        _d_hat.zero();
        _initialized = true;
    }
}

void AC_CustomControl_Adaptive::reset()
{
    // 估计参数保留，不清零（保证平滑切换）
}

void AC_CustomControl_Adaptive::set_notch_sample_rate(float)
{
    // 本控制器未使用陷波器
}

// 构建回归矩阵 Phi (3x6)
void AC_CustomControl_Adaptive::build_regressor(const Vector3f &omega, float fc,
                                                float Phi[3][6]) const
{
    for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 6; j++)
            Phi[i][j] = 0.0f;

    Phi[0][0] = omega.y * omega.z;
    Phi[1][1] = omega.x * omega.z;
    Phi[2][2] = omega.x * omega.y;

    Phi[0][3] = fc;
    Phi[1][4] = fc;
    Phi[2][5] = fc;
}

// 带死区和投影的自适应更新律
void AC_CustomControl_Adaptive::adapt_update(const float Phi[3][6], float dt,
                                             const Vector3f &e, const Vector3f &tau_des)
{
    float gamma = _gamma;
    float dead = _deadzone;
    float sigma = _sigma;

    if (e.length() < dead) {
        return;
    }

    // 计算 Phi^T * e (6x1)
    float PhiT_e[6] = {0};
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            PhiT_e[i] += Phi[j][i] * e[j];
        }
    }

    // 计算 e * tau_des' 矩阵
    Matrix3f e_tau_matrix;
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            e_tau_matrix[i][j] = e[i] * tau_des[j];
        }
    }

    // 更新 Upsilon
    Matrix3f dUpsilon = -(e_tau_matrix + _Upsilon_hat * sigma) * gamma;
    _Upsilon_hat += dUpsilon * dt;

    // 更新 theta
    for (uint8_t i = 0; i < 6; i++) {
        float dtheta = -gamma * (PhiT_e[i] + sigma * _theta_hat[i]);
        _theta_hat[i] += dtheta * dt;
    }

    // 更新 d_hat
    Vector3f dd = - (e + _d_hat * sigma) * gamma;
    _d_hat += dd * dt;

    // ----- 投影 -----
    for (uint8_t i = 0; i < 3; i++) {
        _Upsilon_hat[i][i] = constrain_float(_Upsilon_hat[i][i], 0.5f, 2.0f);
        for (uint8_t j = 0; j < 3; j++) {
            if (i != j) {
                _Upsilon_hat[i][j] = constrain_float(_Upsilon_hat[i][j], -1.0f, 1.0f);
            }
        }
    }
    for (uint8_t i = 0; i < 3; i++) {
        _theta_hat[i] = constrain_float(_theta_hat[i], -0.99f, 0.99f);
    }
    for (uint8_t i = 3; i < 6; i++) {
        _theta_hat[i] = constrain_float(_theta_hat[i], -10.0f, 10.0f);
    }
    for (uint8_t i = 0; i < 3; i++) {
        _d_hat[i] = constrain_float(_d_hat[i], -0.5f, 0.5f);
    }
}

// 主控制更新函数
Vector3f AC_CustomControl_Adaptive::update()
{
    // 获取当前姿态四元数
    Quaternion attitude_body;
    _ahrs->get_quat_body_to_ned(attitude_body);

    Vector3f gyro = _ahrs->get_gyro_latest();

    // 获取期望姿态和期望角速度前馈
    Quaternion attitude_target = _att_control->get_attitude_target_quat();
    Vector3f ang_vel_target = _att_control->get_attitude_target_ang_vel();
    Quaternion rot_target_to_body = attitude_body.inverse() * attitude_target;
    Vector3f ang_vel_ff = rot_target_to_body * ang_vel_target;

    // 姿态误差
    Vector3f attitude_error;
    float thrust_angle_rad, thrust_error_angle_rad;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body,
                                                  attitude_error,
                                                  thrust_angle_rad,
                                                  thrust_error_angle_rad);

    // 期望角速度
    Vector3f target_rate;
    target_rate.x = _p_angle_roll.kP() * attitude_error.x + ang_vel_ff.x;
    target_rate.y = _p_angle_pitch.kP() * attitude_error.y + ang_vel_ff.y;
    target_rate.z = _p_angle_yaw.kP() * attitude_error.z + ang_vel_ff.z;

    // 角速度跟踪误差
    Vector3f rate_error = target_rate - gyro;

    // 设定的油门值
    float fc = _motors->get_throttle();

    // 构建回归矩阵
    float Phi[3][6];
    build_regressor(gyro, fc, Phi);

    // 自适应更新（使用上一时刻力矩）
    static Vector3f tau_prev;
    adapt_update(Phi, _dt, rate_error, tau_prev);

    // 计算标称力矩
    Vector3f tau_nom;
    for (uint8_t i = 0; i < 3; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < 6; j++) {
            sum += Phi[i][j] * _theta_hat[j];
        }
        tau_nom[i] = -sum - _d_hat[i] - 0.5f * rate_error[i];
    }

    // 求逆解算实际力矩
    Vector3f tau;
    Matrix3f inv_Upsilon;
    if (_Upsilon_hat.inverse(inv_Upsilon)) {
        tau = inv_Upsilon * tau_nom;
    } else {
        tau = tau_nom;  // 降级处理
    }

    tau_prev = tau;

    return tau;
}

#endif // AP_CUSTOMCONTROL_ADAPTIVE_ENABLED
