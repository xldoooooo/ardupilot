#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Adaptive.h"
#include "AC_AttitudeControl/AC_AttitudeControl_Multi.h"
#include <AP_Logger/AP_Logger.h>

// 静态成员定义
Matrix3f AC_CustomControl_Adaptive::_Upsilon_hat;
float AC_CustomControl_Adaptive::_theta_hat[6] = {0};
Vector3f AC_CustomControl_Adaptive::_d_hat;

// 参数表
const AP_Param::GroupInfo AC_CustomControl_Adaptive::var_info[] = {
    AP_SUBGROUPINFO(_p_angle_roll,  "ANG_RLL_", 1, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 2, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_angle_yaw,   "ANG_YAW_", 3, AC_CustomControl_Adaptive, AC_P),
    AP_GROUPINFO("GAMMA",    4, AC_CustomControl_Adaptive, _gamma,    0.5f),
    AP_GROUPINFO("GAMMA",    4, AC_CustomControl_Adaptive, _gamma,    2.0f),
    AP_GROUPINFO("DEADZONE", 5, AC_CustomControl_Adaptive, _deadzone, 0.02f),
    AP_GROUPINFO("SIGMA",    6, AC_CustomControl_Adaptive, _sigma,    0.0f),
    AP_SUBGROUPINFO(_p_rate_roll,   "RAT_RLL_", 4, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_rate_pitch,   "RAT_PIT_", 5, AC_CustomControl_Adaptive, AC_P),
    AP_SUBGROUPINFO(_p_rate_yaw,   "RAT_YAW_", 6, AC_CustomControl_Adaptive, AC_P),
    AP_GROUPINFO("TAU",    7, AC_CustomControl_Adaptive, _tau,    0.05f),
    AP_GROUPINFO("GAMMA",    8, AC_CustomControl_Adaptive, _gamma,    0.1f),
    AP_GROUPINFO("DEADZONE", 9, AC_CustomControl_Adaptive, _deadzone, 0.05f),
    AP_GROUPINFO("SIGMA",   10, AC_CustomControl_Adaptive, _sigma,   0.01f),
    AP_GROUPINFO("L", 11, AC_CustomControl_Adaptive, _L, 0.0f),
    AP_GROUPEND
};

AC_CustomControl_Adaptive::AC_CustomControl_Adaptive(AC_CustomControl& frontend,
                                                     AP_AHRS_View*& ahrs,
                                                     AC_AttitudeControl*& att_control,
                                                     AP_MotorsMulticopter*& motors,
                                                     float dt)
    : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P),
    _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P),
    _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P),
    _p_rate_roll(AC_ATC_MULTI_RATE_RP_P),
    _p_rate_pitch(AC_ATC_MULTI_RATE_RP_P),
    _p_rate_yaw(AC_ATC_MULTI_RATE_YAW_P),
    _omega_ref(Vector3f(0,0,0)),   // 新增
    _tau_prev(Vector3f(0,0,0)),     // 新增
    _gyro_buf_idx(0)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);

    _Upsilon_hat = Matrix3f(1.0f, 0,     0,
                            0,    1.0f,  0,
                            0,    0,     1.0f);
    memset(_theta_hat, 0, sizeof(_theta_hat));
    _d_hat.zero();

    // 将缓冲区清零
    for (uint8_t i = 0; i < 5; i++) {
        _gyro_buffer[i].zero();
    }
}

void AC_CustomControl_Adaptive::reset()
{
    // 设置更合理的 Upsilon_hat 初值
    _Upsilon_hat = Matrix3f(1.0f, 0,    0,
                            0,    1.0f, 0,
                            0,    0,    1.0f);
    
    // 设置 theta_hat 初值
    _theta_hat[0] = 0.0f;   // 科里奥利项初始为0
    _theta_hat[1] = 0.0f;
    _theta_hat[2] = 0.0f;
    _theta_hat[3] = 0.01f;  // 滚转油门项
    _theta_hat[4] = 0.01f;  // 俯仰油门项
    _theta_hat[5] = 0.005f; // 偏航油门项
    
    _d_hat.zero();
    _omega_ref = _ahrs->get_gyro_latest();
    _tau_prev.zero();
}

// 对长度为5的Vector3f数组求中值（简单插入排序取中间值）
static Vector3f median_filter(Vector3f buf[5])
{
    // 分别对 x, y, z 分量求中值
    auto median_of_5 = [](float a, float b, float c, float d, float e) -> float {
        // 使用简单的排序网络（5元素最优排序网络）
        // 这里为了简洁，用数组排序，性能足够
        float arr[5] = {a, b, c, d, e};
        for (uint8_t i = 0; i < 4; i++) {
            for (uint8_t j = i+1; j < 5; j++) {
                if (arr[i] > arr[j]) {
                    float tmp = arr[i];
                    arr[i] = arr[j];
                    arr[j] = tmp;
                }
            }
        }
        return arr[2]; // 中值
    };

    Vector3f result;
    result.x = median_of_5(buf[0].x, buf[1].x, buf[2].x, buf[3].x, buf[4].x);
    result.y = median_of_5(buf[0].y, buf[1].y, buf[2].y, buf[3].y, buf[4].y);
    result.z = median_of_5(buf[0].z, buf[1].z, buf[2].z, buf[3].z, buf[4].z);
    return result;
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
        // 即使误差进入死区，仍然施加 sigma 修正以防止长时间静默时参数漂移
        // 但可以减小修正强度（可选，此处直接使用同样 sigma）
        // 更新 Upsilon（仅 sigma 项）
        Matrix3f dUpsilon_sigma = - _Upsilon_hat * gamma * sigma;
        _Upsilon_hat += dUpsilon_sigma * dt;

        for (uint8_t i = 0; i < 6; i++) {
            _theta_hat[i] += -gamma * sigma * _theta_hat[i] * dt;
        }
        _d_hat += - _d_hat * gamma * sigma * dt;
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

    // 带 sigma 修正的更新律
    // Upsilon: dUpsilon = -gamma*(e*tau_des' + sigma*Upsilon)
    Matrix3f dUpsilon = (-e_tau_matrix - _Upsilon_hat * sigma) * gamma;
    _Upsilon_hat += dUpsilon * dt;

    // theta: dtheta = -gamma*(PhiT_e + sigma*theta)
    for (uint8_t i = 0; i < 6; i++) {
        float dtheta = -gamma * (PhiT_e[i] + sigma * _theta_hat[i]);
        _theta_hat[i] += dtheta * dt;
    }

    // d_hat: dd = -gamma*(e + sigma*d_hat)
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
    // 根据电机状态重置控制器，模拟飞行器起飞前和飞行中切换控制器的情况
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // 我们仍然在地面上，重置自定义控制器以避免积分器等的累积
            reset();
            break;
        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // 我们已经离开地面了，继续使用当前控制器状态
            break;
    }

    // 获取当前姿态四元数
    Quaternion attitude_body;
    _ahrs->get_quat_body_to_ned(attitude_body);

    // 获取期望姿态和机体系下的期望角速度前馈
    Vector3f gyro = _ahrs->get_gyro_latest();

    // 获取期望姿态和期望角速度前馈
    Quaternion attitude_target = _att_control->get_attitude_target_quat();
    Vector3f ang_vel_target = _att_control->get_attitude_target_ang_vel();
    Quaternion rot_target_to_body = attitude_body.inverse() * attitude_target;
    Vector3f ang_vel_ff = rot_target_to_body * ang_vel_target;

    // 获取姿态误差、推力倾角、期望推力与实际推力的夹角
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
    Vector3f gyro = _ahrs->get_gyro_latest();

    // 在 update() 中：
    Vector3f gyro_raw = _ahrs->get_gyro_latest();

    // 更新环形缓冲区
    _gyro_buffer[_gyro_buf_idx] = gyro_raw;
    _gyro_buf_idx = (_gyro_buf_idx + 1) % 5;

    // 获取中值滤波后的角速度（需至少填满一次缓冲区，初始几帧可直接用原始值或缓冲区当前值）
    Vector3f gyro;
    if (_gyro_buf_idx == 0) {
        // 缓冲区刚满一轮，可以安全使用中值
        gyro = median_filter(_gyro_buffer);
    } else {
        // 缓冲区未满时，直接用原始值或已有的中值近似（这里简化为原始值）
        gyro = gyro_raw;
    }

    // ========== 参考模型（一阶低通滤波） ==========
    // 参考模型状态：_omega_ref，跟踪 target_rate
    float alpha = 1 / (_tau + _dt + _L*_dt);
    _omega_ref = _omega_ref * _tau * alpha + target_rate * _dt * alpha + gyro * _L * _dt * alpha;
    Vector3f rate_error = _omega_ref - gyro;
    Vector3f rate_error = target_rate - gyro;


    // 设定的油门值
    float fc = _motors->get_throttle();

    // 构建回归矩阵
    float Phi[3][6];
    build_regressor(gyro, fc, Phi);

    // 自适应更新（使用上一时刻力矩）
    adapt_update(Phi, _dt, rate_error, _tau_prev);

    float rate_p[3];
    rate_p[0] = _p_rate_roll.kP();
    rate_p[1] = _p_rate_pitch.kP();
    rate_p[2] = _p_rate_yaw.kP();

    // 计算标称力矩
    Vector3f tau_nom;
    for (uint8_t i = 0; i < 3; i++) {
        float sum = 0.0f;
        for (uint8_t j = 0; j < 6; j++) {
            sum += Phi[i][j] * _theta_hat[j];
        }

        tau_nom[i] = -sum - _d_hat[i] + rate_p[i] * rate_error[i];
    }

    // 求逆解算实际力矩
    Vector3f tau;
    Matrix3f inv_Upsilon;
    if (!_Upsilon_hat.inverse(inv_Upsilon)) {
        // 使用预设的保守逆矩阵（例如对角矩阵的逆）
        inv_Upsilon = Matrix3f(1.0f, 0, 0, 0, 1.0f, 0, 0, 0, 1.0f);
    }
    tau = inv_Upsilon * tau_nom;

    _tau_prev = tau;

    // 每 10 个循环记录一次（假设 dt=0.002~0.01s，10次约 0.02~0.1s）
    if (++_log_counter >= 10) {
        _log_counter = 0;
        AP::logger().Write("ADUP",
            "TimeUS,Up11,Up12,Up13,Up21,Up22,Up23,Up31,Up32,Up33",
            "Qfffffffff",                   // 格式：1个uint64 + 9个float
            AP_HAL::micros64(),                         // 时间戳
            _Upsilon_hat[0][0], _Upsilon_hat[0][1], _Upsilon_hat[0][2],  
            _Upsilon_hat[1][0], _Upsilon_hat[1][1],
            _Upsilon_hat[1][2], _Upsilon_hat[2][0],
            _Upsilon_hat[2][1], _Upsilon_hat[2][2]);             
        
        AP::logger().Write("ADTH",
            "TimeUS,Th1,Th2,Th3,Th4,Th5,Th6",
            "Qffffff",                   // 格式：1个uint64 + 6个float
            AP_HAL::micros64(),                         // 时间戳     
            _theta_hat[0], _theta_hat[1], _theta_hat[2],
            _theta_hat[3], _theta_hat[4], _theta_hat[5]); 
        
        AP::logger().Write("ADDD",
            "TimeUS,D1,D2,D3",
            "Qfff",                   // 格式：1个uint64 + 3个float
            AP_HAL::micros64(),                         // 时间戳     
            _d_hat.x, _d_hat.y, _d_hat.z); 
        
    }

    return tau;
}

#endif // AP_CUSTOMCONTROL_ADAPTIVE_ENABLED
