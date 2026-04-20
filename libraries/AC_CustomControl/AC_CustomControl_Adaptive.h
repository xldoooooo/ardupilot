#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_ADAPTIVE_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AC_CustomControl_Adaptive : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Adaptive(AC_CustomControl& frontend, AP_AHRS_View*& ahrs,
                              AC_AttitudeControl*& att_control,
                              AP_MotorsMulticopter*& motors, float dt);

    Vector3f update() override;
    void reset() override;
    void set_notch_sample_rate(float sample_rate) override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    float _dt;
    AC_P _p_angle_roll;
    AC_P _p_angle_pitch;
    AC_P _p_angle_yaw;

    AP_Float _gamma;
    AP_Float _deadzone;
    AP_Float _sigma;

    static Matrix3f _Upsilon_hat;
    static float _theta_hat[6];
    static Vector3f _d_hat;
    static bool _initialized;

    void build_regressor(const Vector3f &omega, float fc, float Phi[3][6]) const;
    void adapt_update(const float Phi[3][6], float dt,
                      const Vector3f &e, const Vector3f &tau_des);
};

#endif // AP_CUSTOMCONTROL_ADAPTIVE_ENABLED
