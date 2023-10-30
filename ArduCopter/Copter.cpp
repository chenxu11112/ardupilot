/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Andy Piper          :Harmonic notch, In-flight FFT, Bi-directional DShot, various drivers
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel         :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland :PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func)                                            FAST_TASK_CLASS(Copter, &copter, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller),
#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    FAST_TASK(run_custom_controller),
#endif
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(heli_update_autorotation),
#endif // HELI_FRAME
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
    // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(update_heli_control_dynamics),
#endif // HELI_FRAME
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've landed or crashed
    FAST_TASK(update_land_and_crash_detectors),
    // surface tracking update
    FAST_TASK(update_rangefinder_terrain_offset),
#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &copter.camera_mount, update_fast),
#endif
    FAST_TASK(Log_Video_Stabilisation),

    SCHED_TASK(rc_loop, 250, 130, 3),
    SCHED_TASK(throttle_loop, 50, 75, 6),
    SCHED_TASK_CLASS(AP_GPS, &copter.gps, update, 50, 200, 9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow, &copter.optflow, update, 200, 160, 12),
#endif
    SCHED_TASK(update_batt_compass, 10, 120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all, 10, 50, 18),
    SCHED_TASK(arm_motors_check, 10, 50, 21),
#if TOY_MODE_ENABLED == ENABLED
    SCHED_TASK_CLASS(ToyMode, &copter.g2.toy_mode, update, 10, 50, 24),
#endif
    SCHED_TASK(auto_disarm_check, 10, 50, 27),
    SCHED_TASK(auto_trim, 10, 75, 30),
#if RANGEFINDER_ENABLED == ENABLED
    SCHED_TASK(read_rangefinder, 20, 100, 33),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity, &copter.g2.proximity, update, 200, 50, 36),
#endif
#if AP_BEACON_ENABLED
    SCHED_TASK_CLASS(AP_Beacon, &copter.g2.beacon, update, 400, 50, 39),
#endif
    SCHED_TASK(update_altitude, 10, 100, 42),
    SCHED_TASK(run_nav_updates, 50, 100, 45),
    SCHED_TASK(update_throttle_hover, 100, 90, 48),
#if MODE_SMARTRTL_ENABLED == ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL, &copter.mode_smartrtl, save_position, 3, 100, 51),
#endif
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer, &copter.sprayer, update, 3, 90, 54),
#endif
    SCHED_TASK(three_hz_loop, 3, 75, 57),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &copter.ServoRelayEvents, update_events, 50, 75, 60),
#endif
    SCHED_TASK_CLASS(AP_Baro, &copter.barometer, accumulate, 50, 90, 63),
#if AC_PRECLAND_ENABLED
    SCHED_TASK(update_precland, 400, 50, 69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight, 50, 75, 72),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(loop_rate_logging, LOOP_RATE, 50, 75),
#endif
    SCHED_TASK(one_hz_loop, 1, 100, 81),
    SCHED_TASK(ekf_check, 10, 75, 84),
    SCHED_TASK(check_vibration, 10, 50, 87),
    SCHED_TASK(gpsglitch_check, 10, 50, 90),
    SCHED_TASK(takeoff_check, 50, 50, 91),
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landinggear_update, 10, 75, 93),
#endif
    SCHED_TASK(standby_update, 100, 75, 96),
    SCHED_TASK(lost_vehicle_check, 10, 50, 99),
    SCHED_TASK_CLASS(GCS, (GCS*)&copter._gcs, update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS, (GCS*)&copter._gcs, update_send, 400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &copter.camera_mount, update, 50, 75, 108),
#endif
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera, &copter.camera, update, 50, 75, 111),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop, 10, 350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25, 110, 117),
    SCHED_TASK_CLASS(AP_Logger, &copter.logger, periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &copter.ins, periodic, 400, 50, 123),

    SCHED_TASK_CLASS(AP_Scheduler, &copter.scheduler, update_logging, 0.1, 75, 126),
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM, &copter.rpm_sensor, update, 40, 200, 129),
#endif
    SCHED_TASK_CLASS(AP_TempCalibration, &copter.g2.temp_calibration, update, 10, 100, 135),
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update, 10, 100, 138),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check, 10, 100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK(terrain_update, 10, 100, 144),
#endif
#if AP_GRIPPER_ENABLED
    SCHED_TASK_CLASS(AP_Gripper, &copter.g2.gripper, update, 10, 75, 147),
#endif
#if AP_WINCH_ENABLED
    SCHED_TASK_CLASS(AP_Winch, &copter.g2.winch, update, 50, 50, 150),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop, 100, 75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz, 50, 75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop, 10, 75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop, 3.3, 75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1, 75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button, &copter.button, update, 5, 100, 168),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats, &copter.g2.stats, update, 1, 100, 171),
#endif
};

void Copter::get_scheduler_tasks(const AP_Scheduler::Task*& tasks,
                                 uint8_t&                   task_count,
                                 uint32_t&                  log_bit)
{
    tasks      = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit    = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];

#if AP_SCRIPTING_ENABLED
# if MODE_GUIDED_ENABLED == ENABLED
// start takeoff to given altitude (for use by scripting)
bool Copter::start_takeoff(float alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start(alt * 100.0f)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}

// set target location (for use by scripting)
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

// set target position (for use by scripting)
bool Copter::set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);

    return mode_guided.set_destination(pos_neu_cm, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative, terrain_alt);
}

// set target position and velocity (for use by scripting)
bool Copter::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

// set target position, velocity and acceleration (for use by scripting)
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative);
}

bool Copter::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, -vel_ned.z * 100.0f);
    mode_guided.set_velocity(vel_neu_cms);
    return true;
}

// set target velocity and acceleration (for use by scripting)
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    mode_guided.set_velaccel(vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, relative_yaw);
    return true;
}

bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg), radians(pitch_deg), radians(yaw_deg));

    mode_guided.set_angle(q, Vector3f {}, climb_rate_ms * 100, false);
    return true;
}
# endif

# if MODE_CIRCLE_ENABLED == ENABLED
// circle mode controls
bool Copter::get_circle_radius(float& radius_m)
{
    radius_m = circle_nav->get_radius() * 0.01f;
    return true;
}

bool Copter::set_circle_rate(float rate_dps)
{
    circle_nav->set_rate(rate_dps);
    return true;
}
# endif

// set desired speed (m/s). Used for scripting.
bool Copter::set_desired_speed(float speed)
{
    // exit if vehicle is not in auto mode
    if (!flightmode->is_autopilot()) {
        return false;
    }

    wp_nav->set_speed_xy(speed * 100.0f);
    return true;
}

# if MODE_AUTO_ENABLED == ENABLED
// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Copter::nav_script_time(uint16_t& id, uint8_t& cmd, float& arg1, float& arg2, int16_t& arg3, int16_t& arg4)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

// lua scripts use this to indicate when they have complete the command
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
# endif

// returns true if the EKF failsafe has triggered.  Only used by Lua scripts
bool Copter::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing. Only used by Lua scripts
bool Copter::is_landing() const
{
    return flightmode->is_landing();
}

// returns true if vehicle is taking off. Only used by Lua scripts
bool Copter::is_taking_off() const
{
    return flightmode->is_taking_off();
}

bool Copter::current_mode_requires_mission() const
{
#if MODE_AUTO_ENABLED == ENABLED
    return flightmode == &mode_auto;
#else
    return false;
#endif
}

/**
 * @brief 从遥控器/接收机读取用户输入。
 *
 * 此函数以100Hz的频率被调用，用于从遥控器或接收机读取用户输入。
 * 它读取遥控器输入和遥控器上的3档开关。
 */
void Copter::rc_loop()
{
    // 读取遥控器输入
    read_radio();

    // 读取遥控器上的3档开关
    rc().read_mode_switch();
}

/**
 * @brief 油门环路 - 应该以50赫兹的频率运行。
 */
void Copter::throttle_loop()
{
    // 更新油门低补偿值（控制油门与姿态控制的优先级）
    update_throttle_mix();

    // 检查自动解锁状态
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // 更新旋翼速度
    heli_update_rotor_speed_targets();

    // 更新传统直升机平衡杆运动
    heli_update_landing_swash();
#endif

    // 补偿地效（如果启用）
    update_ground_effect_detector();

    // 更新EKF地形高度稳定性
    update_ekf_terrain_height_stable();
}

/**
 * @brief 更新电池和罗盘 - 应以10赫兹的频率调用。
 */
void Copter::update_batt_compass(void)
{
    // 首先读取电池状态，因为它可能用于电机干扰补偿
    battery.read();

    if (AP::compass().available()) {
        // 使用电机油门值更新罗盘 - 用于罗盘电机干扰
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

/**
 * @brief 全速率记录姿态、速率和PID环路。
 * 应在循环速率下运行。
 */
void Copter::loop_rate_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        // 记录姿态信息
        Log_Write_Attitude();

        // 记录PID参数（仅在PIDS位掩码已设置时记录）
        Log_Write_PIDS();
    }
    if (should_log(MASK_LOG_FTN_FAST)) {
        // 写入陷波滤波器的日志消息
        AP::ins().write_notch_log_messages();
    }
    if (should_log(MASK_LOG_IMU_FAST)) {
        // 写入IMU数据
        AP::ins().Write_IMU();
    }
}

/**
 * @brief 十赫兹日志记录循环。
 * @details 此函数应在10赫兹的频率下运行。
 */
void Copter::ten_hz_logging_loop()
{
    // 如果我们尚未以更高速率记录，记录姿态数据
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
    if (!should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        // 如果选择了PIDS位掩码，即使没有选择ATT位掩码，也会以10赫兹记录；如果ATT_FAST和PIDS位掩码已设置，则以循环速率记录
        Log_Write_PIDS();
    }
    // 始终以10赫兹记录EKF姿态数据，除非ATTITUDE_FAST已设置，然后在25赫兹循环中进行记录
    if (should_log(MASK_LOG_MOTBATT)) {
        motors->Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
#if HAL_PROXIMITY_ENABLED
        g2.proximity.log(); // 写入接近传感器距离
#endif
#if AP_BEACON_ENABLED
        g2.beacon.log();
#endif
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
#if AP_WINCH_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/**
 * @brief 25Hz日志记录循环。
 * @details 该函数应以25赫兹的频率运行。
 */
void Copter::twentyfive_hz_logging()
{
    // 如果需要记录姿态快速数据
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS(); // 记录EKF位置信息
    }

    // 如果需要记录IMU数据且不需要记录IMU快速数据
    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU(); // 记录IMU数据
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    // 如果需要记录姿态中等数据或姿态快速数据
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        // 更新自转日志
        g2.arot.Log_Write_Autorotation();
    }
#endif

#if HAL_GYROFFT_ENABLED
    // 如果需要记录FTN快速数据
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages(); // 记录陀螺仪FFT消息
    }
#endif
}

/**
 * @brief 3赫兹循环。
 * @details 该函数应以3赫兹的频率运行。
 */
void Copter::three_hz_loop()
{
    // 检查是否失去了与地面站的联系
    failsafe_gcs_check();

    // 检查是否丢失了地形数据
    failsafe_terrain_check();

    // 检查是否出现了航迹估计失效情况
    failsafe_deadreckon_check();

#if AP_FENCE_ENABLED
    // 检查是否已触发防护栅栏
    fence_check();
#endif // AP_FENCE_ENABLED

    // 在飞行调参中更新通道6
    tuning();

    // 根据高度检查是否应启用回避
    low_alt_avoidance();
}

/**
 * @brief 1赫兹循环。
 * @details 该函数应以1赫兹的频率运行。
 */
void Copter::one_hz_loop()
{
    // 如果需要记录任何数据
    if (should_log(MASK_LOG_ANY)) {
        // 记录AP状态数据
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }

    // 如果电机未解锁
    if (!motors->armed()) {
        // 更新使用电机互锁状态
        update_using_interlock();

        // 检查用户是否已更新了框架类别或类型
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // 设置所有油门通道范围
        motors->update_throttle_range();
#endif
    }

    // 更新分配的功能并启用辅助舵机
    SRV_Channels::enable_aux_servos();

    // 记录地形数据
    terrain_logging();

#if HAL_ADSB_ENABLED
    // 设置ADS-B飞行状态为非着陆状态
    adsb.set_is_flying(!ap.land_complete);
#endif

    // 设置AP通知标志中的飞行状态为非着陆状态
    AP_Notify::flags.flying = !ap.land_complete;
}

/**
 * @brief 初始化简单方位。
 * @details 该函数用于初始化简单方位，捕获当前的 cos_yaw 和 sin_yaw 值，并初始化超级简单方位
 * 使其朝向家的方向（即与简单模式方位相差180度）。
 */
void Copter::init_simple_bearing()
{
    // 捕获当前的 cos_yaw 和 sin_yaw 值
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // 初始化超级简单方位（朝向家的方向，与简单模式方位相差180度）
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor + 18000);
    super_simple_cos_yaw      = simple_cos_yaw;
    super_simple_sin_yaw      = simple_sin_yaw;

    // 记录简单方位数据
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

/**
 * @brief 更新简单模式。
 * @details 如果飞机处于简单模式下，将调整飞行员的输入。简单模式会旋转控制通道的输入以适应不同的飞行方向。
 */
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // 如果没有新的遥控器帧或不在简单模式下，立即退出
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // 标记遥控器帧已被消耗
    ap.new_radio_frame = false;

    if (simple_mode == SimpleMode::SIMPLE) {
        // 将滚转和俯仰输入旋转 -初始的简单方位（即朝向北）
        rollx  = channel_roll->get_control_in() * simple_cos_yaw - channel_pitch->get_control_in() * simple_sin_yaw;
        pitchx = channel_roll->get_control_in() * simple_sin_yaw + channel_pitch->get_control_in() * simple_cos_yaw;
    } else {
        // 将滚转和俯仰输入旋转 -超级简单方位（即朝向家的反方向）
        rollx  = channel_roll->get_control_in() * super_simple_cos_yaw - channel_pitch->get_control_in() * super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in() * super_simple_sin_yaw + channel_pitch->get_control_in() * super_simple_cos_yaw;
    }

    // 将滚转和俯仰输入从朝向北旋转到飞行器的视角
    channel_roll->set_control_in(rollx * ahrs.cos_yaw() + pitchx * ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx * ahrs.sin_yaw() + pitchx * ahrs.cos_yaw());
}

/**
 * @brief 更新超级简单方位。
 * @details 基于位置信息调整简单方位，应在更新 home_bearing 后调用。
 *
 * @param force_update 强制更新标志，如果为 true，则强制更新超级简单方位。
 */
void Copter::update_super_simple_bearing(bool force_update)
{
    // 如果不需要强制更新
    if (!force_update) {
        // 如果不在超级简单模式下或距离家的距离小于超级简单半径
        if (simple_mode != SimpleMode::SUPERSIMPLE || home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    // 获取到家的方位
    const int32_t bearing = home_bearing();

    // 检查到家的方位是否改变了至少5度
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    // 更新超级简单方位的上次方位
    super_simple_last_bearing = bearing;

    // 计算方位对应的弧度角
    const float angle_rad = radians((super_simple_last_bearing + 18000) / 100);

    // 更新超级简单方位的 cos_yaw 和 sin_yaw
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

/**
 * @brief 读取姿态融合传感器（AHRS）数据。
 * @details 告诉 AHRS 跳过惯性导航系统（INS）更新，因为我们已经在 FAST_TASK 中完成了更新。
 */
void Copter::read_AHRS(void)
{
    // 告诉 AHRS 跳过 INS 更新，因为在 FAST_TASK 中已经执行过。
    ahrs.update(true);
}

/**
 * @brief 更新高度信息。
 * @details 读取气压计高度信息，并记录控制调整数据。
 */
void Copter::update_altitude()
{
    // 读取气压计高度信息
    read_barometer();

    // 如果需要记录控制调整数据
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning(); // 记录控制调整数据
        // 如果不需要记录快速FTN数据
        if (!should_log(MASK_LOG_FTN_FAST)) {
            AP::ins().write_notch_log_messages(); // 记录INS的陷波滤波数据
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages(); // 记录陀螺仪FFT消息
#endif
        }
    }
}

/**
 * @brief 获取当前航点距离（单位：米）。
 * @details 该函数用于获取当前航点与飞行器的距离信息。
 * @param[out] distance 存储航点距离的变量。
 * @return 获取成功时返回 true。
 */
bool Copter::get_wp_distance_m(float& distance) const
{
    // 请参考 GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance() * 0.01;
    return true;
}

/**
 * @brief 获取当前航点方位角（单位：度）。
 * @details 该函数用于获取当前航点的方位角，即飞行器指向航点的方向角度。
 * @param[out] bearing 存储航点方位角的变量。
 * @return 获取成功时返回 true。
 */
bool Copter::get_wp_bearing_deg(float& bearing) const
{
    // 请参考 GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing() * 0.01;
    return true;
}

/**
 * @brief 获取当前航点横跨误差（单位：米）。
 * @details 该函数用于获取当前航点横跨误差，表示飞行器离开航线的距离。
 * @param[out] xtrack_error 存储横跨误差的变量。
 * @return 获取成功时返回 true。
 */
bool Copter::get_wp_crosstrack_error_m(float& xtrack_error) const
{
    // 请参考 GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

/**
 * @brief 获取目标地球坐标系角速度（单位：弧度/秒）。
 * @details 该函数用于获取目标地球坐标系角速度，通常用于某些云台的Z轴组件。
 * @param[out] rate_ef_targets 存储目标地球坐标系角速度的变量。
 * @return 获取成功时返回 true。
 */
bool Copter::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{
    // 如果已着陆或未解锁，则始终返回零向量
    if (copter.ap.land_complete) {
        rate_ef_targets.zero();
    } else {
        rate_ef_targets = attitude_control->get_rate_ef_targets();
    }
    return true;
}

/*
  主要Copter类的构造函数
 */
Copter::Copter(void)
    : logger(g.log_bitmask)                                // 日志记录器初始化
    , flight_modes(&g.flight_mode1)                        // 飞行模式初始化
    , simple_cos_yaw(1.0f)                                 // 简单方位的cos_yaw初始化
    , super_simple_cos_yaw(1.0)                            // 超级简单方位的cos_yaw初始化
    , land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF) // 着陆加速度的滤波器初始化
    , rc_throttle_control_in_filter(1.0f)                  // 遥控节流控制输入滤波器初始化
    , inertial_nav(ahrs)                                   // 惯性导航系统初始化
    , param_loader(var_info)                               // 参数加载器初始化
    , flightmode(&mode_stabilize)                          // 飞行模式初始化
{
}

Copter      copter;           // 主要Copter对象
AP_Vehicle& vehicle = copter; // AP_Vehicle对象引用

AP_HAL_MAIN_CALLBACKS(&copter); // AP_HAL主回调
