#include "Copter.h"

// 这段代码的目的是根据飞行器上的测距传感器（通常是超声波或激光测距传感器）测量的高度来调整位置控制器的偏移，以实现垂直跟踪地面或天花板。
// update_surface_offset - 管理位置控制器的垂直偏移，以跟随测量的地面或天花板高度
void Copter::SurfaceTracking::update_surface_offset()
{
#if RANGEFINDER_ENABLED == ENABLED
    // 检查是否超时
    const uint32_t now_ms  = millis();
    const bool     timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // 检查跟踪状态和测距传感器是否正常
    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) || ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        // 计算表面相对于EKF原点的高度
        // 例如，如果飞行器距离EKF原点10米，而测距传感器报告高度为3米，curr_surface_alt_above_origin_cm将是7米（或700厘米）
        RangeFinderState& rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;

        // 更新位置控制器的目标高度偏移，以使其与表面的高度相匹配
        copter.pos_control->set_pos_offset_target_z_cm(rf_state.terrain_offset_cm);
        last_update_ms    = now_ms;
        valid_for_logging = true;

        // 如果此控制器刚刚启用，或者目标在向上和向下之间发生了更改，或者故障已经消除，将重置目标高度
        if (timeout || reset_target || (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->set_pos_offset_z_cm(rf_state.terrain_offset_cm);
            reset_target           = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        // 如果表面跟踪不活跃，将重置位置控制器的偏移值，并在下次活跃时重置目标偏移
        if (timeout) {
            copter.pos_control->set_pos_offset_z_cm(0);
            copter.pos_control->set_pos_offset_target_z_cm(0);
            reset_target = true;
        }
    }
#else
    // 如果未启用测距传感器功能，将重置位置控制器的偏移值
    copter.pos_control->set_pos_offset_z_cm(0);
    copter.pos_control->set_pos_offset_target_z_cm(0);
#endif
}

// get target altitude (in cm) above ground
// returns true if there is a valid target
bool Copter::SurfaceTracking::get_target_alt_cm(float &target_alt_cm) const
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return false;
    }
    // check target has been updated recently
    if (AP_HAL::millis() - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        return false;
    }
    target_alt_cm = (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm());
    return true;
}

// set target altitude (in cm) above ground
void Copter::SurfaceTracking::set_target_alt_cm(float _target_alt_cm)
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return;
    }
    copter.pos_control->set_pos_offset_z_cm(copter.inertial_nav.get_position_z_up_cm() - _target_alt_cm);
    last_update_ms = AP_HAL::millis();
}

bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist) const
{
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    target_dist = dir * (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm()) * 0.01f;
    return true;
}

float Copter::SurfaceTracking::get_dist_for_logging() const
{
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_cm : copter.rangefinder_state.alt_cm) * 0.01f;
}

// set direction
void Copter::SurfaceTracking::set_surface(Surface new_surface)
{
    if (surface == new_surface) {
        return;
    }
    // check we have a range finder in the correct direction
    if ((new_surface == Surface::GROUND) && !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no downward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    if ((new_surface == Surface::CEILING) && !copter.rangefinder.has_orientation(ROTATION_PITCH_90)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no upward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    surface = new_surface;
    reset_target = true;
}
