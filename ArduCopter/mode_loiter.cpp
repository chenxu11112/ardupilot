#include "Copter.h"

#if MODE_LOITER_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeLoiter::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // 从简单模式转换成飞行员输入 -- apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // 获取飞行员期望的倾斜角度
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // 处理飞行员的俯仰滚转输入 -- process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // 初始化垂直位置控制器 -- initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 设置垂直速度和加速度限幅 -- set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    #if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
    #endif

    return true;
}

    #if AC_PRECLAND_ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false; // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
    #endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiter::run()
{
    float target_roll, target_pitch; // 目标滚转和俯仰角
    float target_yaw_rate   = 0.0f;  // 目标偏航速率
    float target_climb_rate = 0.0f;  // 目标爬升速率

    // 设置垂直速度和加速度限幅 -- set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 处理飞行员输入直到无线电失控 -- process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // 从简单模式转换成飞行员输入 -- apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // 获取飞行员期望的倾斜角度 -- convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // 处理飞行员的俯仰滚转输入 -- process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // 获取飞行员希望得到的航向速度 -- get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // 获取飞行员希望得到的爬升速度 -- get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // 放弃loiter目标如果可能降落 -- relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // loiter状态机的状态 -- Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // loiter状态机 -- Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f); // 强制油门输出衰减到零
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f); // 强制油门输出衰减到零
        break;

    case AltHold_Takeoff:
        // 启动起飞
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
        }

        // 获取避障调整后的爬升速率
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // 设置位置控制器目标，根据飞行员输入进行调整
        takeoff.do_pilot_takeoff(target_climb_rate);

        // 运行loiter控制器
        loiter_nav->update();

        // 调用姿态控制器
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Flying:
        // 设置电机为全速状态
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    #if AC_PRECLAND_ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // 如果我们不在做精密悬停，运行loiter控制器
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
    #else
        loiter_nav->update();
    #endif

        // 调用姿态控制器
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // 获取避障调整的爬升速率
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

uint32_t ModeLoiter::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLoiter::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
