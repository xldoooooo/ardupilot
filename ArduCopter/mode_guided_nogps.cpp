#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED

// 初始化GUIDED_NOGPS模式
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // 初始化角度控制器
    ModeGuided::angle_control_start();
    return true;
}

// 运行GUIDED角度控制器（以100Hz以上）
void ModeGuidedNoGPS::run()
{
    ModeGuided::angle_control_run();
}

#endif
