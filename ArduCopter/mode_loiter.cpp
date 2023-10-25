#include "Copter.h"

#if MODE_LOITER_ENABLED == ENABLED

// “loiter”飞行模式的初始化和运行相关的函数调用。

// ModeLoiter::init - 初始化loiter飞行模式
bool ModeLoiter::init(bool ignore_checks)
{
    // 如果无线电通信故障，将执行radio_failsafe代码块
    if (!copter.failsafe.radio) {
        // 定义目标横滚和俯仰角度变量
        float target_roll, target_pitch;

        // 如果处于简单模式，将简单模式的变换应用到飞行员的输入
        update_simple_mode();

        // 获取飞行员期望的倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // 处理飞行员的横滚和俯仰输入，将它们转换为期望加速度
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // 如果出现无线电通信故障，清除飞行员期望的加速度
        // 防止无法切换到RTL或其他模式
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // 初始化目标状态
    loiter_nav->init_target();

    // 如果垂直位置控制器尚未激活，初始化垂直位置控制器
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 设置垂直速度和加速度的限制值
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 如果启用了特定的预着陆功能，设置相关变量
# if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
# endif

    // 返回初始化状态
    return true;
}

# if AC_PRECLAND_ENABLED
// 检查精准定点功能是否启用，以决定是否执行精准定点操作
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false; // 精准定点未启用
    }

    if (copter.ap.land_complete_maybe) {
        return false; // 飞行器在地面上不进行移动
    }

    // 允许飞行员主动移动飞行器，即使精准定点已激活
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }

    // 检查是否获得有效的精准定点矢量
    if (!copter.precland.target_acquired()) {
        return false; // 未获取有效的定点目标矢量
    }

    // 执行精准定点操作
    return true;
}

// 在精准定点模式下执行横向和纵向定点控制
void ModeLoiter::precision_loiter_xy()
{
    // 清除飞行员的期望加速度，以防止干扰精准定点控制
    loiter_nav->clear_pilot_desired_acceleration();

    Vector2f target_pos, target_vel;

    // 获取精准定点的目标位置，如果未获得目标位置，则使用当前位置
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }

    // 获取精准定点的目标速度
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();

    // 输入目标位置、目标速度和零加速度，执行横向和纵向精准定点控制
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);

    // 更新横向和纵向定点控制器
    pos_control->update_xy_controller();
}
# endif

// loiter_run - 运行Loiter模式的控制器
// 应该以100Hz或更高的频率调用
void ModeLoiter::run()
{
    float target_roll, target_pitch; // 飞行器的目标滚转和俯仰角
    float target_yaw_rate   = 0.0f;  // 飞行器的目标偏航速率
    float target_climb_rate = 0.0f;  // 飞行器的目标爬升速率

    // 设置垂直速度和加速度的限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 处理飞行员的输入，除非出现无线电失控
    if (!copter.failsafe.radio) {
        // 将简单模式下的飞行员输入进行转换，以应用简单模式的飞行转换
        update_simple_mode();

        // 获取飞行员期望的倾斜角度，以用于飞行器的姿态控制
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // 处理飞行员的俯仰和滚转输入，以用于飞行器的姿态控制
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // 获取飞行员期望的偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // 获取飞行员期望的爬升速率，确保速率在安全范围内
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // 在出现无线电失控事件并且飞行器没有切换到RTL模式时，清除飞行员期望的加速度
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // 如果飞行器可能已经降落，减轻Loiter模式下的目标
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // 确定AltHold模式状态机的状态
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // AltHold状态机
    switch (loiter_state) {
        case AltHold_MotorStopped:
            // 当飞行器的电机停止时执行以下操作
            // 重置姿态控制器的速率I项（积分项）
            attitude_control->reset_rate_controller_I_terms();
            // 重置姿态控制器的偏航目标和速率
            attitude_control->reset_yaw_target_and_rate();
            // 强制位置控制器输出（油门）减小到零
            pos_control->relax_z_controller(0.0f);
            // 初始化Loiter目标
            loiter_nav->init_target();
            // 设置姿态控制器的推力矢量、偏航速率，并不执行偏航调整
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            break;

        case AltHold_Landed_Ground_Idle:
            // 如果飞行器在地面上且空闲，则执行以下操作
            // 重置姿态控制器的偏航目标和速率
            attitude_control->reset_yaw_target_and_rate();
            FALLTHROUGH;

        case AltHold_Landed_Pre_Takeoff:
            // 如果飞行器在地面上，但即将起飞，则执行以下操作
            // 平滑地重置姿态控制器的速率I项（积分项）
            attitude_control->reset_rate_controller_I_terms_smoothly();
            // 初始化Loiter目标
            loiter_nav->init_target();
            // 设置姿态控制器的推力矢量、偏航速率，并不执行偏航调整
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            // 强制位置控制器输出（油门）减小到零
            pos_control->relax_z_controller(0.0f);
            break;

        case AltHold_Takeoff:
            // 当飞行器正在起飞时执行以下操作
            // 如果起飞尚未开始，则启动起飞程序
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            }
            // 获取经过避障调整后的爬升速率
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
            // 根据飞行员的输入调整位置控制器的目标
            takeoff.do_pilot_takeoff(target_climb_rate);
            // 更新Loiter目标
            loiter_nav->update();
            // 设置姿态控制器的推力矢量、偏航速率，并不执行偏航调整
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            break;

        case AltHold_Flying:
            // 如果飞行器正在飞行，则执行以下操作
            // 设置电机状态为全速状态
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            // 如果精密悬停控制已经启用，执行以下操作
# if AC_PRECLAND_ENABLED
            bool precision_loiter_old_state = _precision_loiter_active;
            if (do_precision_loiter()) {
                // 执行精密悬停的XY轴控制
                precision_loiter_xy();
                _precision_loiter_active = true;
            } else {
                _precision_loiter_active = false;
            }
            // 如果精密悬停之前是激活状态，但现在不再激活，那么重新初始化Loiter目标
            if (precision_loiter_old_state && !_precision_loiter_active) {
                loiter_nav->init_target();
            }
            // 如果我们不在执行精密悬停，继续运行Loiter控制器
            if (!_precision_loiter_active) {
                loiter_nav->update();
            }
# else
            // 如果精密悬停未启用，继续运行Loiter控制器
            loiter_nav->update();
# endif
            // 设置姿态控制器的推力矢量、偏航速率，并不执行偏航调整
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            // 获取经过避障调整后的爬升速率
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
            // 根据地面测量更新垂直偏移
            copter.surface_tracking.update_surface_offset();
            // 将指令的爬升速率发送给位置控制器
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            break;
    }

    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}

// wp_distance - 获取飞行器当前位置到Loiter模式目标的直线距离
uint32_t ModeLoiter::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

// wp_bearing - 获取飞行器当前位置到Loiter模式目标的方位角
int32_t ModeLoiter::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
