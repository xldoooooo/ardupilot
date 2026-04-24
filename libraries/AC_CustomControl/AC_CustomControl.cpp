#include <AP_HAL/AP_HAL.h>

#include "AC_CustomControl.h"

#if AP_CUSTOMCONTROL_ENABLED

#include "AC_CustomControl_Backend.h"
#include "AC_CustomControl_Empty.h"
#include "AC_CustomControl_PID.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>

// 用户自定义参数
const AP_Param::GroupInfo AC_CustomControl::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Custom control type
    // @Description: Custom control type to be used
    // @Values: 0:None, 1:Empty, 2:PID, 3:INDI, 4:XLD
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 1, AC_CustomControl, _controller_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _AXIS_MASK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("_AXIS_MASK", 2, AC_CustomControl, _custom_controller_mask, 0),

    // parameters for empty controller. only used as a template, no need for param table 
    AP_SUBGROUPVARPTR(_backend, "1_", 6, AC_CustomControl, _backend_var_info[0]),

    // @Group: 2_
    // @Path: AC_CustomControl_PID.cpp
    AP_SUBGROUPVARPTR(_backend, "2_", 7, AC_CustomControl, _backend_var_info[1]),

    AP_GROUPEND
};

const struct AP_Param::GroupInfo *AC_CustomControl::_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

AC_CustomControl::AC_CustomControl(AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors) :
    _ahrs(ahrs),
    _att_control(att_control),
    _motors(motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 初始化自定义控制器后端，并从EEPROM加载参数
void AC_CustomControl::init(void)
{
    _dt = AP::scheduler().get_loop_period_s();

    switch (CustomControlType(_controller_type))
    {
        case CustomControlType::CONT_NONE:
            break;
        case CustomControlType::CONT_EMPTY: // This is template backend. Don't initialize it.
            // This is template backend. Don't initialize it.
            // _backend = NEW_NOTHROW AC_CustomControl_Empty(*this, _ahrs, _att_control, _motors, _dt);
            // _backend_var_info[get_type()] = AC_CustomControl_Empty::var_info;
            break;
        case CustomControlType::CONT_PID:
            _backend = NEW_NOTHROW AC_CustomControl_PID(*this, _ahrs, _att_control, _motors, _dt);
            _backend_var_info[get_type()] = AC_CustomControl_PID::var_info;
            break;
        default:
            return;
    }

    if (_backend && _backend_var_info[get_type()]) {
        AP_Param::load_object_from_eeprom(_backend, _backend_var_info[get_type()]);
    }
}

// 运行自定义控制器并将输出发送到分配矩阵/电机
void AC_CustomControl::update(void)
{
    if (is_safe_to_run()) {
        Vector3f motor_out_rpy = _backend->update();
        motor_set(motor_out_rpy);  // 设置_roll_in、_pitch_in和_yaw_in，并重置主控制器的积分器以避免积分饱和
    }
}

// 根据掩码将自定义姿态控制器的输出应用到电机上，并重置主控制器的积分器以避免积分饱和
void AC_CustomControl::motor_set(Vector3f rpy) {
    if (_custom_controller_mask & (uint8_t)CustomControlOption::ROLL) {
        _motors->set_roll(rpy.x);  // 设置_roll_in
        _motors->set_roll_ff(0.0f); // 自定义控制启用时清零主控制器roll前馈
        _att_control->get_rate_roll_pid().set_integrator(0.0);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::PITCH) {
        _motors->set_pitch(rpy.y);  // 设置_pitch_in
        _motors->set_pitch_ff(0.0f); // 自定义控制启用时清零主控制器pitch前馈
        _att_control->get_rate_pitch_pid().set_integrator(0.0);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::YAW) {
        _motors->set_yaw(rpy.z);  // 设置_yaw_in
        _motors->set_yaw_ff(0.0f); // 自定义控制启用时清零主控制器yaw前馈
        _att_control->get_rate_yaw_pid().set_integrator(0.0);
    }
}

// 将主控制器的目标置为当前状态，重置滤波器，并将积分器置零，以允许平滑过渡到主控制器
void AC_CustomControl::reset_main_att_controller(void)
{
    // 如果主控制器启用了前馈，则将姿态和速率目标重置为当前状态，以实现平滑过渡回主控制器
    if (_att_control->get_bf_feedforward()) {
        _att_control->relax_attitude_controllers();
    }

    _att_control->get_rate_roll_pid().set_integrator(0.0);
    _att_control->get_rate_pitch_pid().set_integrator(0.0);
    _att_control->get_rate_yaw_pid().set_integrator(0.0);
}

// 遥控器通道置为高电平，就启用该函数
void AC_CustomControl::set_custom_controller(bool enabled)
{
    // 记录切换日志
    log_switch();

    // 初始化状态为不安全，直到所有检查通过
    _custom_controller_active = false;

    // 未定义控制器的情况下，不允许切换，以避免意外重置主控制器
    if (_controller_type == CustomControlType::CONT_NONE) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is not enabled");
        return;
    }

    // 控制器类型超出范围，不允许切换，以避免访问空指针
    if (_controller_type > CUSTOMCONTROL_MAX_TYPES) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller type is out of range");
        return;
    }

    // 后端控制器未初始化，不允许切换，以避免访问空指针
    if (_backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reboot to enable selected custom controller");
        return;
    }

    // 如果没有选择任何轴，则不允许切换，以避免无法控制的状态
    if (_custom_controller_mask == 0 && enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Axis mask is not set");
        return;
    }

    // 如果切换回主控制器，则将主控制器目标重置为当前状态，以实现平滑过渡回主控制器
    if (!enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is OFF");
        // don't reset if the empty backend is selected
        if (_controller_type > CustomControlType::CONT_EMPTY) {
            reset_main_att_controller();
        }
    }

    // 如果启用自定义控制器，则重置滤波器和积分器，以避免积分饱和，并记录切换日志
    if (enabled && _controller_type > CustomControlType::CONT_NONE) {
        _backend->reset();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is ON");
    }

    _custom_controller_active = enabled;

    // 记录切换日志
    log_switch();
}

// 确认遥控器通道置为高电平，并且选择了控制器类型
bool AC_CustomControl::is_safe_to_run(void) {
    if (_custom_controller_active && (_controller_type > CustomControlType::CONT_NONE)
        && (_controller_type <= CUSTOMCONTROL_MAX_TYPES) && _backend != nullptr)
    {
        return true;
    }

    return false;
}

// 当自定义控制器被切换时记录日志
void AC_CustomControl::log_switch(void) {
    // @LoggerMessage: CC
    // @Description: Custom Controller data
    // @Field: TimeUS: Time since system startup
    // @Field: Type: controller type
    // @FieldValueEnum: Type: AC_CustomControl::CustomControlType
    // @Field: Act: true if controller is active
    AP::logger().Write("CC", "TimeUS,Type,Act","QBB",
                            AP_HAL::micros64(),
                            _controller_type,
                            _custom_controller_active);
}

// 设置自定义控制器的陷波采样率
void AC_CustomControl::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_backend != nullptr) {
        _backend->set_notch_sample_rate(sample_rate);
    }
#endif
}

#endif  // AP_CUSTOMCONTROL_ENABLED
