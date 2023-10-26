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

#include "AP_Arming.h"
#include <AC_Fence/AC_Fence.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Button/AP_Button.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_FETtecOneWire/AP_FETtecOneWire.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Generator/AP_Generator.h>
#include <AP_GyroFFT/AP_GyroFFT.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OSD/AP_OSD.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_Proximity/AP_Proximity.h>
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
# include <AP_CANManager/AP_CANManager.h>
# include <AP_Common/AP_Common.h>
# include <AP_Vehicle/AP_Vehicle_Type.h>

# include <AP_DroneCAN/AP_DroneCAN.h>
# include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#endif

#include <AP_Logger/AP_Logger.h>

#define AP_ARMING_COMPASS_MAGFIELD_EXPECTED 530
#define AP_ARMING_COMPASS_MAGFIELD_MIN      185 // 0.35 * 530 milligauss
#define AP_ARMING_COMPASS_MAGFIELD_MAX      875 // 1.65 * 530 milligauss
#define AP_ARMING_BOARD_VOLTAGE_MAX         5.8f
#define AP_ARMING_ACCEL_ERROR_THRESHOLD     0.75f
#define AP_ARMING_MAGFIELD_ERROR_THRESHOLD  100
#define AP_ARMING_AHRS_GPS_ERROR_MAX        10 // accept up to 10m difference between AHRS and GPS

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
# define ARMING_RUDDER_DEFAULT (uint8_t) RudderArming::ARMONLY
#else
# define ARMING_RUDDER_DEFAULT (uint8_t) RudderArming::ARMDISARM
#endif

#ifndef PREARM_DISPLAY_PERIOD
# define PREARM_DISPLAY_PERIOD 30
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Arming::var_info[] = {

    // @Param{Plane, Rover}: REQUIRE
    // @DisplayName: 启动电机要求
    // @Description: 在满足一些条件之前禁用解锁。若为0，没有要求（可以立即解锁）。若为1，在未解锁时发送最小油门PWM值到油门通道。若为2，在未解锁时发送0 PWM（无信号）到油门通道。对于启用了ICE并在ICE_OPTIONS中设置了解锁时的油门选项的飞机，电机将在未解锁时始终获得THR_MIN。当所有强制要求和ARMING_CHECK条款都满足时，可以使用方向舵杆或GCS命令进行解锁。请注意，将此参数设置为0时，需要重新启动才能立即解锁飞机。
    // @Values: 0:禁用, 1:未解锁时发送最小PWM, 2:未解锁时发送0 PWM
    // @User: 高级
    AP_GROUPINFO_FLAGS_FRAME("REQUIRE", 0, AP_Arming, require, float(Required::YES_MIN_PWM),
                             AP_PARAM_FLAG_NO_SHIFT,
                             AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_ROVER),

    // 2是CHECK参数，存储在AP_Int16中

    // @Param: ACCTHRESH
    // @DisplayName: 加速度计误差阈值
    // @Description: 用于检测不一致的加速度计的加速度计误差阈值。将此误差范围与其他加速度计进行比较，以检测硬件或校准错误。较小的值表示更严格的检查和更难通过解锁检查。并非所有加速度计都相同。
    // @Units: m/s/s
    // @Range: 0.25 3.0
    // @User: 高级
    AP_GROUPINFO("ACCTHRESH", 3, AP_Arming, accel_error_threshold, AP_ARMING_ACCEL_ERROR_THRESHOLD),

    // 索引4是VOLT_MIN，已移动到AP_BattMonitor
    // 索引5是VOLT2_MIN，已移动到AP_BattMonitor

    // @Param: RUDDER
    // @DisplayName: 使用方向舵解锁/锁定
    // @Description: 允许使用方向舵输入进行解锁/锁定。启用后，可以使用右方向舵解锁，左方向舵锁定。方向舵解锁仅在油门为零加减死区（RCx_DZ）内有效。根据不同的飞行器类型，某些模式下禁止解锁。请查看每种飞行器的文档。如果在自动油门模式下允许解锁，解锁时请谨慎操作！
    // @Values: 0:禁用, 1:仅解锁, 2:解锁/锁定
    // @User: 高级
    AP_GROUPINFO_FRAME("RUDDER", 6, AP_Arming, _rudder_arming, ARMING_RUDDER_DEFAULT, AP_PARAM_FRAME_PLANE | AP_PARAM_FRAME_ROVER | AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_BLIMP),

    // @Param: MIS_ITEMS
    // @DisplayName: 需要的任务项目
    // @Description: 位掩码，标识需要计划以解锁飞机的任务项目。
    // @Bitmask: 0:着陆, 1:VTOL着陆, 2:DO_LAND_START, 3:起飞, 4:VTOL起飞, 5:集结点, 6:RTL
    // @User: 高级
    AP_GROUPINFO("MIS_ITEMS", 7, AP_Arming, _required_mission_items, 0),

    // @Param: CHECK
    // @DisplayName: 启动电机前执行的检查（位掩码）
    // @Description: 在允许启动电机之前执行的检查的位掩码。对于大多数用户，建议将此参数保持默认值1（启用所有检查）。您可以通过将每个检查类型的值相加来设置此参数，以选择所需的检查。例如，若只想在具有GPS锁定并且没有遥控器失控情况下才能解锁，则可以将ARMING_CHECK设置为72。
    // @Bitmask: 0:所有, 1:气压计, 2:罗盘, 3:GPS锁定, 4:INS, 5:参数, 6:遥控通道, 7:电池电压, 8:电池电量, 10:可用记录, 11:硬件安全开关, 12:GPS配置, 13:系统, 14:任务, 15:测距仪, 16:相机, 17:辅助授权, 18:视觉定位, 19:FFT
    // @Bitmask{Plane}: 0:所有, 1:气压计, 2:罗盘, 3:GPS锁定, 4:INS, 5:参数, 6:遥控通道, 7:电池电压, 8:电池电量, 9:空速, 10:可用记录, 11:硬件安全开关, 12:GPS配置, 13:系统, 14:任务, 15:测距仪, 16:相机, 17:辅助授权, 19:FFT
    // @User: 标准
    AP_GROUPINFO("CHECK", 8, AP_Arming, checks_to_perform, ARMING_CHECK_ALL),

    // @Param: OPTIONS
    // @DisplayName: 解锁选项
    // @Description: 用于更改解锁行为的选项。
    // @Values: 0:无, 1:禁用预解锁显示
    // @User: 高级
    AP_GROUPINFO("OPTIONS", 9, AP_Arming, _arming_options, 0),

    // @Param: MAGTHRESH
    // @DisplayName: 罗盘磁场强度误差阈值 vs 地球磁场模型
    // @Description: 与地球磁场模型相比，罗盘磁场强度误差阈值。X和Y轴使用此阈值进行比较，Z轴使用2倍此阈值。0表示禁用检查。
    // @Units: mGauss
    // @Range: 0 500
    // @User: 高级
    AP_GROUPINFO("MAGTHRESH", 10, AP_Arming, magfield_error_threshold, AP_ARMING_MAGFIELD_ERROR_THRESHOLD),

    AP_GROUPEND
};

#if HAL_WITH_IO_MCU
# include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#pragma GCC diagnostic push
#if defined(__clang__)
# pragma GCC diagnostic ignored "-Wbitwise-instead-of-logical"
#endif

AP_Arming::AP_Arming()
{
    if (_singleton) {
        AP_HAL::panic("Too many AP_Arming instances");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// update - 执行预解锁检查。预计每秒调用一次。
void AP_Arming::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    // 执行预解锁检查，并每30秒显示失败信息
    bool display_fail = false;
    if (now_ms - last_prearm_display_ms > PREARM_DISPLAY_PERIOD * 1000) {
        display_fail           = true;
        last_prearm_display_ms = now_ms;
    }
    // 另一方面，用户可能永远不希望显示这些失败信息：
    if (option_enabled(Option::DISABLE_PREARM_DISPLAY)) {
        display_fail = false;
    }

    pre_arm_checks(display_fail); // 执行预解锁检查
}

// 返回期望的磁场强度值（用于罗盘校准）
uint16_t AP_Arming::compass_magfield_expected() const
{
    return AP_ARMING_COMPASS_MAGFIELD_EXPECTED;
}

// 检查飞行器是否已解锁
bool AP_Arming::is_armed() const
{
    return armed || arming_required() == Required::NO;
}

// 如果已解锁并且安全开关处于关闭状态，则返回 true
bool AP_Arming::is_armed_and_safety_off() const
{
    return is_armed() && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}

// 返回已启用的解锁前检查的位掩码
uint32_t AP_Arming::get_enabled_checks() const
{
    return checks_to_perform;
}

// 检查特定的解锁前检查是否已启用
bool AP_Arming::check_enabled(const enum AP_Arming::ArmingChecks check) const
{
    if (checks_to_perform & ARMING_CHECK_ALL) {
        return true;
    }
    return (checks_to_perform & check);
}

// check_failed - 处理解锁前检查失败情况并向用户报告失败信息
void AP_Arming::check_failed(const enum AP_Arming::ArmingChecks check, bool report, const char* fmt, ...) const
{
    if (!report) {
        return;
    }
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];

    // metafmt包装了传入的格式字符串，以根据当前正在进行的检查类型（PreArm或Arm）添加前缀。
    const char* metafmt = "PreArm: %s"; // 它一直是格式化的
    if (running_arming_checks) {
        metafmt = "Arm: %s";
    }
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), metafmt, fmt);

    MAV_SEVERITY severity = MAV_SEVERITY_CRITICAL;
    if (!check_enabled(check)) {
        // 技术上应该是NOTICE，但在这个级别会让用户感到不安：
        severity = MAV_SEVERITY_DEBUG;
    }
    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(severity, taggedfmt, arg_list); // 发送警告消息
    va_end(arg_list);
}

// check_failed - 处理通用的失败情况并向用户报告失败信息
void AP_Arming::check_failed(bool report, const char* fmt, ...) const
{
    if (!report) {
        return;
    }
    char taggedfmt[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];

    // metafmt包装了传入的格式字符串，以根据当前正在进行的检查类型（PreArm或Arm）添加前缀。
    const char* metafmt = "PreArm: %s"; // 它一直是格式化的
    if (running_arming_checks) {
        metafmt = "Arm: %s";
    }
    hal.util->snprintf(taggedfmt, sizeof(taggedfmt), metafmt, fmt);

    va_list arg_list;
    va_start(arg_list, fmt);
    gcs().send_textv(MAV_SEVERITY_CRITICAL, taggedfmt, arg_list); // 发送警告消息
    va_end(arg_list);
}

// barometer_checks - 执行与气压计相关的解锁前检查
bool AP_Arming::barometer_checks(bool report)
{
#ifdef HAL_BARO_ALLOW_INIT_NO_BARO
    return true;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (AP::sitl()->baro_count == 0) {
        // 模拟没有气压计的板卡
        return true;
    }
#endif
    if (check_enabled(ARMING_CHECK_BARO)) {
        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] {};
        if (!AP::baro().arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_BARO, report, "Baro: %s", buffer);
            return false;
        }
    }

    return true;
}

// airspeed_checks - 执行与空速计相关的解锁前检查
bool AP_Arming::airspeed_checks(bool report)
{
#if AP_AIRSPEED_ENABLED
    if (check_enabled(ARMING_CHECK_AIRSPEED)) {
        const AP_Airspeed* airspeed = AP_Airspeed::get_singleton();
        if (airspeed == nullptr) {
            // 不是具备空速计功能的飞行器
            return true;
        }
        for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
            if (airspeed->enabled(i) && airspeed->use(i) && !airspeed->healthy(i)) {
                check_failed(ARMING_CHECK_AIRSPEED, report, "Airspeed %d not healthy", i + 1);
                return false;
            }
        }
    }
#endif

    return true;
}

// logging_checks - 执行与记录相关的解锁前检查
bool AP_Arming::logging_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_LOGGING)) {
        if (!AP::logger().logging_present()) {
            // 记录已禁用，无需检查。
            return true;
        }
        if (AP::logger().logging_failed()) {
            check_failed(ARMING_CHECK_LOGGING, report, "Logging failed");
            return false;
        }
        if (!AP::logger().CardInserted()) {
            check_failed(ARMING_CHECK_LOGGING, report, "No SD card");
            return false;
        }
        if (AP::logger().in_log_download()) {
            check_failed(ARMING_CHECK_LOGGING, report, "Downloading logs");
            return false;
        }
    }
    return true;
}

#if AP_INERTIALSENSOR_ENABLED
bool AP_Arming::ins_accels_consistent(const AP_InertialSensor& ins)
{
    const uint8_t accel_count = ins.get_accel_count();
    if (accel_count <= 1) {
        return true;
    }

    const Vector3f& prime_accel_vec = ins.get_accel();
    const uint32_t  now             = AP_HAL::millis();
    for (uint8_t i = 0; i < accel_count; i++) {
        if (!ins.use_accel(i)) {
            continue;
        }
        // get next accel vector
        const Vector3f& accel_vec = ins.get_accel(i);
        Vector3f        vec_diff  = accel_vec - prime_accel_vec;
        // allow for user-defined difference, typically 0.75 m/s/s. Has to pass in last 10 seconds
        float threshold = accel_error_threshold;
        if (i >= 2) {
            /*
              we allow for a higher threshold for IMU3 as it
              runs at a different temperature to IMU1/IMU2,
              and is not used for accel data in the EKF
            */
            threshold *= 3;
        }

        // EKF is less sensitive to Z-axis error
        vec_diff.z *= 0.5f;

        if (vec_diff.length() > threshold) {
            // this sensor disagrees with the primary sensor, so
            // accels are inconsistent:
            last_accel_pass_ms = 0;
            return false;
        }
    }

    if (last_accel_pass_ms == 0) {
        // we didn't return false in the loop above, so sensors are
        // consistent right now:
        last_accel_pass_ms = now;
    }

    // must pass for at least 10 seconds before we're considered consistent:
    if (now - last_accel_pass_ms < 10000) {
        return false;
    }

    return true;
}

bool AP_Arming::ins_gyros_consistent(const AP_InertialSensor& ins)
{
    const uint8_t gyro_count = ins.get_gyro_count();
    if (gyro_count <= 1) {
        return true;
    }

    const Vector3f& prime_gyro_vec = ins.get_gyro();
    const uint32_t  now            = AP_HAL::millis();
    for (uint8_t i = 0; i < gyro_count; i++) {
        if (!ins.use_gyro(i)) {
            continue;
        }
        // get next gyro vector
        const Vector3f& gyro_vec = ins.get_gyro(i);
        const Vector3f  vec_diff = gyro_vec - prime_gyro_vec;
        // allow for up to 5 degrees/s difference
        if (vec_diff.length() > radians(5)) {
            // this sensor disagrees with the primary sensor, so
            // gyros are inconsistent:
            last_gyro_pass_ms = 0;
            return false;
        }
    }

    // we didn't return false in the loop above, so sensors are
    // consistent right now:
    if (last_gyro_pass_ms == 0) {
        last_gyro_pass_ms = now;
    }

    // must pass for at least 10 seconds before we're considered consistent:
    if (now - last_gyro_pass_ms < 10000) {
        return false;
    }

    return true;
}

bool AP_Arming::ins_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_INS)) {
        const AP_InertialSensor& ins = AP::ins();
        if (!ins.get_gyro_health_all()) {
            check_failed(ARMING_CHECK_INS, report, "Gyros not healthy");
            return false;
        }
        if (!ins.gyro_calibrated_ok_all()) {
            check_failed(ARMING_CHECK_INS, report, "Gyros not calibrated");
            return false;
        }
        if (!ins.get_accel_health_all()) {
            check_failed(ARMING_CHECK_INS, report, "Accels not healthy");
            return false;
        }
        if (!ins.accel_calibrated_ok_all()) {
            check_failed(ARMING_CHECK_INS, report, "3D Accel calibration needed");
            return false;
        }

        // check if accelerometers have calibrated and require reboot
        if (ins.accel_cal_requires_reboot()) {
            check_failed(ARMING_CHECK_INS, report, "Accels calibrated requires reboot");
            return false;
        }

        // check all accelerometers point in roughly same direction
        if (!ins_accels_consistent(ins)) {
            check_failed(ARMING_CHECK_INS, report, "Accels inconsistent");
            return false;
        }

        // check all gyros are giving consistent readings
        if (!ins_gyros_consistent(ins)) {
            check_failed(ARMING_CHECK_INS, report, "Gyros inconsistent");
            return false;
        }

        // no arming while doing temp cal
        if (ins.temperature_cal_running()) {
            check_failed(ARMING_CHECK_INS, report, "temperature cal running");
            return false;
        }

# if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
        // If Batch sampling enabled it must be initialized
        if (ins.batchsampler.enabled() && !ins.batchsampler.is_initialised()) {
            check_failed(ARMING_CHECK_INS, report, "Batch sampling requires reboot");
            return false;
        }
# endif
    }

# if HAL_GYROFFT_ENABLED
    // gyros are healthy so check the FFT
    if (check_enabled(ARMING_CHECK_FFT)) {
        // Check that the noise analyser works
        AP_GyroFFT* fft = AP::fft();

        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        if (fft != nullptr && !fft->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_INS, report, "%s", fail_msg);
            return false;
        }
    }
# endif

    return true;
}
#endif // AP_INERTIALSENSOR_ENABLED

bool AP_Arming::compass_checks(bool report)
{
    Compass& _compass = AP::compass();

#if COMPASS_CAL_ENABLED
    // check if compass is calibrating
    if (_compass.is_calibrating()) {
        check_failed(report, "Compass calibration running");
        return false;
    }

    // check if compass has calibrated and requires reboot
    if (_compass.compass_cal_requires_reboot()) {
        check_failed(report, "Compass calibrated requires reboot");
        return false;
    }
#endif

    if (check_enabled(ARMING_CHECK_COMPASS)) {

        // avoid Compass::use_for_yaw(void) as it implicitly calls healthy() which can
        // incorrectly skip the remaining checks, pass the primary instance directly
        if (!_compass.use_for_yaw(0)) {
            // compass use is disabled
            return true;
        }

        if (!_compass.healthy()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass not healthy");
            return false;
        }
        // check compass learning is on or offsets have been set
#if !APM_BUILD_COPTER_OR_HELI && !APM_BUILD_TYPE(APM_BUILD_Blimp)
        // check compass offsets have been set if learning is off
        // copter and blimp always require configured compasses
        if (!_compass.learn_offsets_enabled())
#endif
        {
            char failure_msg[50] = {};
            if (!_compass.configured(failure_msg, ARRAY_SIZE(failure_msg))) {
                check_failed(ARMING_CHECK_COMPASS, report, "%s", failure_msg);
                return false;
            }
        }

        // check for unreasonable compass offsets
        const Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > _compass.get_offsets_max()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compass offsets too high");
            return false;
        }

        // check for unreasonable mag field length
        const float mag_field = _compass.get_field().length();
        if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN) {
            check_failed(ARMING_CHECK_COMPASS, report, "Check mag field: %4.0f, max %d, min %d", (double)mag_field, AP_ARMING_COMPASS_MAGFIELD_MAX, AP_ARMING_COMPASS_MAGFIELD_MIN);
            return false;
        }

        // check all compasses point in roughly same direction
        if (!_compass.consistent()) {
            check_failed(ARMING_CHECK_COMPASS, report, "Compasses inconsistent");
            return false;
        }

        // if ahrs is using compass and we have location, check mag field versus expected earth magnetic model
        Location ahrs_loc;
        AP_AHRS& ahrs = AP::ahrs();
        if ((magfield_error_threshold > 0) && ahrs.use_compass() && ahrs.get_location(ahrs_loc)) {
            const Vector3f veh_mag_field_ef   = ahrs.get_rotation_body_to_ned() * _compass.get_field();
            const Vector3f earth_field_mgauss = AP_Declination::get_earth_field_ga(ahrs_loc) * 1000.0;
            const Vector3f diff_mgauss        = veh_mag_field_ef - earth_field_mgauss;
            if (MAX(fabsf(diff_mgauss.x), fabsf(diff_mgauss.y)) > magfield_error_threshold) {
                check_failed(ARMING_CHECK_COMPASS, report, "Check mag field (xy diff:%.0f>%d)", (double)MAX(fabsf(diff_mgauss.x), (double)fabsf(diff_mgauss.y)), (int)magfield_error_threshold);
                return false;
            }
            if (fabsf(diff_mgauss.x) > magfield_error_threshold * 2.0) {
                check_failed(ARMING_CHECK_COMPASS, report, "Check mag field (z diff:%.0f>%d)", (double)fabsf(diff_mgauss.z), (int)magfield_error_threshold * 2);
                return false;
            }
        }
    }

    return true;
}

bool AP_Arming::gps_checks(bool report)
{
    const AP_GPS& gps = AP::gps();
    if (check_enabled(ARMING_CHECK_GPS)) {

        // Any failure messages from GPS backends
        char failure_msg[50] = {};
        if (!AP::gps().backends_healthy(failure_msg, ARRAY_SIZE(failure_msg))) {
            if (failure_msg[0] != '\0') {
                check_failed(ARMING_CHECK_GPS, report, "%s", failure_msg);
            }
            return false;
        }

        for (uint8_t i = 0; i < gps.num_sensors(); i++) {
#if defined(GPS_BLENDED_INSTANCE)
            if ((i != GPS_BLENDED_INSTANCE) &&
#else
            if (
#endif
                (gps.get_type(i) == AP_GPS::GPS_Type::GPS_TYPE_NONE)) {
                if (gps.primary_sensor() == i) {
                    check_failed(ARMING_CHECK_GPS, report, "GPS %i: primary but TYPE 0", i + 1);
                    return false;
                }
                continue;
            }

            // GPS OK?
            if (gps.status(i) < AP_GPS::GPS_OK_FIX_3D) {
                check_failed(ARMING_CHECK_GPS, report, "GPS %i: Bad fix", i + 1);
                return false;
            }

            // GPS update rate acceptable
            if (!gps.is_healthy(i)) {
                check_failed(ARMING_CHECK_GPS, report, "GPS %i: not healthy", i + 1);
                return false;
            }
        }

        if (!AP::ahrs().home_is_set()) {
            check_failed(ARMING_CHECK_GPS, report, "AHRS: waiting for home");
            return false;
        }

        // check GPSs are within 50m of each other and that blending is healthy
        float distance_m;
        if (!gps.all_consistent(distance_m)) {
            check_failed(ARMING_CHECK_GPS, report, "GPS positions differ by %4.1fm", (double)distance_m);
            return false;
        }
#if defined(GPS_BLENDED_INSTANCE)
        if (!gps.blend_health_check()) {
            check_failed(ARMING_CHECK_GPS, report, "GPS blending unhealthy");
            return false;
        }
#endif

        // check AHRS and GPS are within 10m of each other
        if (gps.num_sensors() > 0) {
            const Location gps_loc = gps.location();
            Location       ahrs_loc;
            if (AP::ahrs().get_location(ahrs_loc)) {
                const float distance = gps_loc.get_distance(ahrs_loc);
                if (distance > AP_ARMING_AHRS_GPS_ERROR_MAX) {
                    check_failed(ARMING_CHECK_GPS, report, "GPS and AHRS differ by %4.1fm", (double)distance);
                    return false;
                }
            }
        }
    }

    if (check_enabled(ARMING_CHECK_GPS_CONFIG)) {
        uint8_t first_unconfigured;
        if (gps.first_unconfigured_gps(first_unconfigured)) {
            check_failed(ARMING_CHECK_GPS_CONFIG,
                         report,
                         "GPS %d still configuring this GPS",
                         first_unconfigured + 1);
            if (report) {
                gps.broadcast_first_configuration_failure_reason();
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::battery_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_BATTERY)) {

        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] {};
        if (!AP::battery().arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_BATTERY, report, "%s", buffer);
            return false;
        }
    }
    return true;
}

bool AP_Arming::hardware_safety_check(bool report)
{
    if (check_enabled(ARMING_CHECK_SWITCH)) {

        // check if safety switch has been pushed
        if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
            check_failed(ARMING_CHECK_SWITCH, report, "Hardware safety switch");
            return false;
        }
    }

    return true;
}

bool AP_Arming::rc_arm_checks(AP_Arming::Method method)
{
    // don't check the trims if we are in a failsafe
    if (!rc().has_valid_input()) {
        return true;
    }

    // only check if we've received some form of input within the last second
    // this is a protection against a vehicle having never enabled an input
    uint32_t last_input_ms = rc().last_input_ms();
    if ((last_input_ms == 0) || ((AP_HAL::millis() - last_input_ms) > 1000)) {
        return true;
    }

    bool check_passed = true;
    // ensure all rc channels have different functions
    if (rc().duplicate_options_exist()) {
        check_failed(ARMING_CHECK_PARAMETERS, true, "Duplicate Aux Switch Options");
        check_passed = false;
    }
    if (rc().flight_mode_channel_conflicts_with_rc_option()) {
        check_failed(ARMING_CHECK_PARAMETERS, true, "Mode channel and RC%d_OPTION conflict", rc().flight_mode_channel_number());
        check_passed = false;
    }
    const RCMapper* rcmap = AP::rcmap();
    if (rcmap != nullptr) {
        if (!rc().option_is_enabled(RC_Channels::Option::ARMING_SKIP_CHECK_RPY)) {
            const char*   names[3]    = { "Roll", "Pitch", "Yaw" };
            const uint8_t channels[3] = { rcmap->roll(), rcmap->pitch(), rcmap->yaw() };
            for (uint8_t i = 0; i < ARRAY_SIZE(channels); i++) {
                const RC_Channel* c = rc().channel(channels[i] - 1);
                if (c == nullptr) {
                    continue;
                }

                if((c == rc().channel(CH_1)) || (c == rc().channel(CH_2))){
                    continue;
                }

                if (c->get_control_in() != 0) {
                    if ((method != Method::RUDDER) || (c != rc().get_arming_channel())) { // ignore the yaw input channel if rudder arming
                            check_failed(ARMING_CHECK_RC, true, "%s (RC%d) is not neutral", names[i], channels[i]);
                            check_passed = false;
                    }
                }
            }
        }

        // if throttle check is enabled, require zero input
        if (rc().arming_check_throttle()) {
            RC_Channel* c = rc().channel(rcmap->throttle() - 1);
            if (c != nullptr) {
                if (c->get_control_in() != 0) {
                    check_failed(ARMING_CHECK_RC, true, "Throttle (RC%d) is not neutral", rcmap->throttle());
                    check_passed = false;
                }
            }
            c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FWD_THR);
            if (c != nullptr) {
                uint8_t fwd_thr = c->percent_input();
                // require channel input within 2% of minimum
                if (fwd_thr > 2) {
                    check_failed(ARMING_CHECK_RC, true, "VTOL Fwd Throttle is not zero");
                    check_passed = false;
                }
            }
        }
    }
    return check_passed;
}

bool AP_Arming::rc_calibration_checks(bool report)
{
    bool          check_passed = true;
    const uint8_t num_channels = RC_Channels::get_valid_channel_count();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        const RC_Channel* c = rc().channel(i);
        if (c == nullptr) {
            continue;
        }
        if (i >= num_channels && !(c->has_override())) {
            continue;
        }
        const uint16_t trim = c->get_radio_trim();
        if (c->get_radio_min() > trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d_MIN is greater than RC%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
        if (c->get_radio_max() < trim) {
            check_failed(ARMING_CHECK_RC, report, "RC%d_MAX is less than RC%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
    }

    return check_passed;
}

bool AP_Arming::rc_in_calibration_check(bool report)
{
    if (rc().calibrating()) {
        check_failed(ARMING_CHECK_RC, report, "RC calibrating");
        return false;
    }
    return true;
}

bool AP_Arming::manual_transmitter_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_RC)) {

        if (AP_Notify::flags.failsafe_radio) {
            check_failed(ARMING_CHECK_RC, report, "Radio failsafe on");
            return false;
        }

        if (!rc_calibration_checks(report)) {
            return false;
        }
    }

    return rc_in_calibration_check(report);
}

bool AP_Arming::mission_checks(bool report)
{
    AP_Mission* mission = AP::mission();
    if (check_enabled(ARMING_CHECK_MISSION) && _required_mission_items) {
        if (mission == nullptr) {
            check_failed(ARMING_CHECK_MISSION, report, "No mission library present");
            return false;
        }

        const struct MisItemTable {
            MIS_ITEM_CHECK check;
            MAV_CMD        mis_item_type;
            const char*    type;
        } misChecks[] = {
            { MIS_ITEM_CHECK_LAND, MAV_CMD_NAV_LAND, "land" },
            { MIS_ITEM_CHECK_VTOL_LAND, MAV_CMD_NAV_VTOL_LAND, "vtol land" },
            { MIS_ITEM_CHECK_DO_LAND_START, MAV_CMD_DO_LAND_START, "do land start" },
            { MIS_ITEM_CHECK_TAKEOFF, MAV_CMD_NAV_TAKEOFF, "takeoff" },
            { MIS_ITEM_CHECK_VTOL_TAKEOFF, MAV_CMD_NAV_VTOL_TAKEOFF, "vtol takeoff" },
            { MIS_ITEM_CHECK_RETURN_TO_LAUNCH, MAV_CMD_NAV_RETURN_TO_LAUNCH, "RTL" },
        };
        for (uint8_t i = 0; i < ARRAY_SIZE(misChecks); i++) {
            if (_required_mission_items & misChecks[i].check) {
                if (!mission->contains_item(misChecks[i].mis_item_type)) {
                    check_failed(ARMING_CHECK_MISSION, report, "Missing mission item: %s", misChecks[i].type);
                    return false;
                }
            }
        }
        if (_required_mission_items & MIS_ITEM_CHECK_RALLY) {
#if HAL_RALLY_ENABLED
            AP_Rally* rally = AP::rally();
            if (rally == nullptr) {
                check_failed(ARMING_CHECK_MISSION, report, "No rally library present");
                return false;
            }
            Location ahrs_loc;
            if (!AP::ahrs().get_location(ahrs_loc)) {
                check_failed(ARMING_CHECK_MISSION, report, "Can't check rally without position");
                return false;
            }
            RallyLocation rally_loc = {};
            if (!rally->find_nearest_rally_point(ahrs_loc, rally_loc)) {
                check_failed(ARMING_CHECK_MISSION, report, "No sufficiently close rally point located");
                return false;
            }
#else
            check_failed(ARMING_CHECK_MISSION, report, "No rally library present");
            return false;
#endif
        }
    }

#if AP_SDCARD_STORAGE_ENABLED
    if (check_enabled(ARMING_CHECK_MISSION) && mission != nullptr && (mission->failed_sdcard_storage() || StorageManager::storage_failed())) {
        check_failed(ARMING_CHECK_MISSION, report, "Failed to open %s", AP_MISSION_SDCARD_FILENAME);
        return false;
    }
#endif

    // do not allow arming if there are no mission items and we are in
    // (e.g.) AUTO mode
    if (AP::vehicle()->current_mode_requires_mission() && (mission == nullptr || mission->num_commands() <= 1)) {
        check_failed(ARMING_CHECK_MISSION, report, "Mode requires mission");
        return false;
    }

    return true;
}

bool AP_Arming::rangefinder_checks(bool report)
{
    if (check_enabled(ARMING_CHECK_RANGEFINDER)) {
        RangeFinder* range = RangeFinder::get_singleton();
        if (range == nullptr) {
            return true;
        }

        char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        if (!range->prearm_healthy(buffer, ARRAY_SIZE(buffer))) {
            check_failed(ARMING_CHECK_RANGEFINDER, report, "%s", buffer);
            return false;
        }
    }

    return true;
}

bool AP_Arming::servo_checks(bool report) const
{
#if NUM_SERVO_CHANNELS
    bool check_passed = true;
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel* c = SRV_Channels::srv_channel(i);
        if (c == nullptr || c->get_function() <= SRV_Channel::k_none) {
            continue;
        }

        const uint16_t trim = c->get_trim();
        if (c->get_output_min() > trim) {
            check_failed(report, "SERVO%d_MIN is greater than SERVO%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }
        if (c->get_output_max() < trim) {
            check_failed(report, "SERVO%d_MAX is less than SERVO%d_TRIM", i + 1, i + 1);
            check_passed = false;
        }

        // check functions using PWM are enabled
        if (SRV_Channels::get_disabled_channel_mask() & 1U << i) {
            const SRV_Channel::Aux_servo_function_t ch_function = c->get_function();

            // motors, e-stoppable functions, neopixels and ProfiLEDs may be digital outputs and thus can be disabled
            // scripting can use its functions as labels for LED setup
            const bool disabled_ok = SRV_Channel::is_motor(ch_function) || SRV_Channel::should_e_stop(ch_function) || (ch_function >= SRV_Channel::k_LED_neopixel1 && ch_function <= SRV_Channel::k_LED_neopixel4) || (ch_function >= SRV_Channel::k_ProfiLED_1 && ch_function <= SRV_Channel::k_ProfiLED_Clock) || (ch_function >= SRV_Channel::k_scripting1 && ch_function <= SRV_Channel::k_scripting16);

            // for all other functions raise a pre-arm failure
            if (!disabled_ok) {
                check_failed(report, "SERVO%u_FUNCTION=%u on disabled channel", i + 1, (unsigned)ch_function);
                check_passed = false;
            }
        }
    }

# if HAL_WITH_IO_MCU
    if (!iomcu.healthy() && AP_BoardConfig::io_enabled()) {
        check_failed(report, "IOMCU is unhealthy");
        check_passed = false;
    }
# endif

    return check_passed;
#else
    return false;
#endif
}

bool AP_Arming::board_voltage_checks(bool report)
{
    // check board voltage
    if (check_enabled(ARMING_CHECK_VOLTAGE)) {
#if HAL_HAVE_BOARD_VOLTAGE
        const float bus_voltage = hal.analogin->board_voltage();
        const float vbus_min    = AP_BoardConfig::get_minimum_board_voltage();
        if (((bus_voltage < vbus_min) || (bus_voltage > AP_ARMING_BOARD_VOLTAGE_MAX))) {
            check_failed(ARMING_CHECK_VOLTAGE, report, "Board (%1.1fv) out of range %1.1f-%1.1fv", (double)bus_voltage, (double)vbus_min, (double)AP_ARMING_BOARD_VOLTAGE_MAX);
            return false;
        }
#endif // HAL_HAVE_BOARD_VOLTAGE

#if HAL_HAVE_SERVO_VOLTAGE
        const float vservo_min = AP_BoardConfig::get_minimum_servo_voltage();
        if (is_positive(vservo_min)) {
            const float servo_voltage = hal.analogin->servorail_voltage();
            if (servo_voltage < vservo_min) {
                check_failed(ARMING_CHECK_VOLTAGE, report, "Servo voltage to low (%1.2fv < %1.2fv)", (double)servo_voltage, (double)vservo_min);
                return false;
            }
        }
#endif // HAL_HAVE_SERVO_VOLTAGE
    }

    return true;
}

#if HAL_HAVE_IMU_HEATER
bool AP_Arming::heater_min_temperature_checks(bool report)
{
    if (checks_to_perform & ARMING_CHECK_ALL) {
        AP_BoardConfig* board = AP::boardConfig();
        if (board) {
            float  temperature;
            int8_t min_temperature;
            if (board->get_board_heater_temperature(temperature) && board->get_board_heater_arming_temperature(min_temperature) && (temperature < min_temperature)) {
                check_failed(ARMING_CHECK_SYSTEM, report, "heater temp low (%0.1f < %i)", temperature, min_temperature);
                return false;
            }
        }
    }
    return true;
}
#endif // HAL_HAVE_IMU_HEATER

/*
  check base system operations
 */
bool AP_Arming::system_checks(bool report)
{
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] {};

    if (check_enabled(ARMING_CHECK_SYSTEM)) {
        if (!hal.storage->healthy()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Param storage failed");
            return false;
        }

        // check main loop rate is at least 90% of expected value
        const float    actual_loop_rate   = AP::scheduler().get_filtered_loop_rate_hz();
        const uint16_t expected_loop_rate = AP::scheduler().get_loop_rate_hz();
        const float    loop_rate_pct      = actual_loop_rate / expected_loop_rate;
        if (loop_rate_pct < 0.90) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Main loop slow (%uHz < %uHz)", (unsigned)actual_loop_rate, (unsigned)expected_loop_rate);
            return false;
        }

#if AP_TERRAIN_AVAILABLE
        const AP_Terrain* terrain = AP_Terrain::get_singleton();
        if ((terrain != nullptr) && terrain->init_failed()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "Terrain out of memory");
            return false;
        }
#endif
#if AP_SCRIPTING_ENABLED
        const AP_Scripting* scripting = AP_Scripting::get_singleton();
        if ((scripting != nullptr) && !scripting->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_SYSTEM, report, "%s", buffer);
            return false;
        }
#endif
#if HAL_ADSB_ENABLED
        AP_ADSB* adsb = AP::ADSB();
        if ((adsb != nullptr) && adsb->enabled() && adsb->init_failed()) {
            check_failed(ARMING_CHECK_SYSTEM, report, "ADSB out of memory");
            return false;
        }
#endif
    }
    if (AP::internalerror().errors() != 0) {
        AP::internalerror().errors_as_string((uint8_t*)buffer, ARRAY_SIZE(buffer));
        check_failed(report, "Internal errors 0x%x l:%u %s", (unsigned int)AP::internalerror().errors(), AP::internalerror().last_error_line(), buffer);
        return false;
    }

    if (!hal.gpio->arming_checks(sizeof(buffer), buffer)) {
        check_failed(report, "%s", buffer);
        return false;
    }

    if (check_enabled(ARMING_CHECK_PARAMETERS)) {
#if AP_RPM_ENABLED
        auto* rpm = AP::rpm();
        if (rpm && !rpm->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
#if AP_RELAY_ENABLED
        auto* relay = AP::relay();
        if (relay && !relay->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
#if HAL_PARACHUTE_ENABLED
        auto* chute = AP::parachute();
        if (chute && !chute->arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
#if HAL_BUTTON_ENABLED
        const auto& button = AP::button();
        if (!button.arming_checks(sizeof(buffer), buffer)) {
            check_failed(ARMING_CHECK_PARAMETERS, report, "%s", buffer);
            return false;
        }
#endif
    }

    return true;
}

bool AP_Arming::terrain_database_required() const
{
    AP_Mission* mission = AP::mission();
    if (mission == nullptr) {
        // no mission support?
        return false;
    }
    if (mission->contains_terrain_alt_items()) {
        return true;
    }
    return false;
}

// check terrain database is fit-for-purpose
bool AP_Arming::terrain_checks(bool report) const
{
    if (!check_enabled(ARMING_CHECK_PARAMETERS)) {
        return true;
    }

    if (!terrain_database_required()) {
        return true;
    }

#if AP_TERRAIN_AVAILABLE

    const AP_Terrain* terrain = AP_Terrain::get_singleton();
    if (terrain == nullptr) {
        // this is also a system error, and it is already complaining
        // about it.
        return false;
    }

    if (!terrain->enabled()) {
        check_failed(ARMING_CHECK_PARAMETERS, report, "terrain disabled");
        return false;
    }

    char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    if (!terrain->pre_arm_checks(fail_msg, sizeof(fail_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, report, "%s", fail_msg);
        return false;
    }

    return true;

#else
    check_failed(ARMING_CHECK_PARAMETERS, report, "terrain required but disabled");
    return false;
#endif
}

// check nothing is too close to vehicle
bool AP_Arming::proximity_checks(bool report) const
{
#if HAL_PROXIMITY_ENABLED
    const AP_Proximity* proximity = AP::proximity();
    // return true immediately if no sensor present
    if (proximity == nullptr) {
        return true;
    }
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    if (!proximity->prearm_healthy(buffer, ARRAY_SIZE(buffer))) {
        check_failed(report, "%s", buffer);
        return false;
    }
    return true;
#endif

    return true;
}

bool AP_Arming::can_checks(bool report)
{
#if HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_CANMANAGER_ENABLED
    if (check_enabled(ARMING_CHECK_SYSTEM)) {
        char fail_msg[50] = {};
        (void)fail_msg; // might be left unused
        uint8_t num_drivers = AP::can().get_num_drivers();

        for (uint8_t i = 0; i < num_drivers; i++) {
            switch (AP::can().get_driver_type(i)) {
                case AP_CAN::Protocol::PiccoloCAN: {
# if HAL_PICCOLO_CAN_ENABLE
                    AP_PiccoloCAN* ap_pcan = AP_PiccoloCAN::get_pcan(i);

                    if (ap_pcan != nullptr && !ap_pcan->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "PiccoloCAN: %s", fail_msg);
                        return false;
                    }

# else
                    check_failed(ARMING_CHECK_SYSTEM, report, "PiccoloCAN not enabled");
                    return false;
# endif
                    break;
                }
                case AP_CAN::Protocol::DroneCAN: {
# if HAL_ENABLE_DRONECAN_DRIVERS
                    AP_DroneCAN* ap_dronecan = AP_DroneCAN::get_dronecan(i);
                    if (ap_dronecan != nullptr && !ap_dronecan->prearm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
                        check_failed(ARMING_CHECK_SYSTEM, report, "DroneCAN: %s", fail_msg);
                        return false;
                    }
# endif
                    break;
                }
                case AP_CAN::Protocol::USD1:
                case AP_CAN::Protocol::TOFSenseP:
                case AP_CAN::Protocol::NanoRadar_NRA24:
                case AP_CAN::Protocol::Benewake: {
                    for (uint8_t j = i; j; j--) {
                        if (AP::can().get_driver_type(i) == AP::can().get_driver_type(j - 1)) {
                            check_failed(ARMING_CHECK_SYSTEM, report, "Same rfnd on different CAN ports");
                            return false;
                        }
                    }
                    break;
                }
                case AP_CAN::Protocol::EFI_NWPMU:
                case AP_CAN::Protocol::None:
                case AP_CAN::Protocol::Scripting:
                case AP_CAN::Protocol::Scripting2:
                case AP_CAN::Protocol::KDECAN:

                    break;
            }
        }
    }
#endif
    return true;
}

bool AP_Arming::fence_checks(bool display_failure)
{
#if AP_FENCE_ENABLED
    const AC_Fence* fence = AP::fence();
    if (fence == nullptr) {
        return true;
    }

    // check fence is ready
    const char* fail_msg = nullptr;
    if (fence->pre_arm_check(fail_msg)) {
        return true;
    }

    if (fail_msg == nullptr) {
        check_failed(display_failure, "Check fence");
    } else {
        check_failed(display_failure, "%s", fail_msg);
    }

    return false;
#else
    return true;
#endif
}

bool AP_Arming::camera_checks(bool display_failure)
{
    if (check_enabled(ARMING_CHECK_CAMERA)) {
#if HAL_RUNCAM_ENABLED
        AP_RunCam* runcam = AP::runcam();
        if (runcam == nullptr) {
            return true;
        }

        // check camera is ready
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        if (!runcam->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_CAMERA, display_failure, "%s", fail_msg);
            return false;
        }
#endif
    }
    return true;
}

bool AP_Arming::osd_checks(bool display_failure) const
{
#if OSD_ENABLED
    if (check_enabled(ARMING_CHECK_OSD)) {
        // if no OSD then pass
        const AP_OSD* osd = AP::osd();
        if (osd == nullptr) {
            return true;
        }
        // do osd checks for configuration
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        if (!osd->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_OSD, display_failure, "%s", fail_msg);
            return false;
        }
    }
#endif
    return true;
}

bool AP_Arming::mount_checks(bool display_failure) const
{
#if HAL_MOUNT_ENABLED
    if (check_enabled(ARMING_CHECK_CAMERA)) {
        AP_Mount* mount = AP::mount();
        if (mount == nullptr) {
            return true;
        }
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
        if (!mount->pre_arm_checks(fail_msg, sizeof(fail_msg))) {
            check_failed(ARMING_CHECK_CAMERA, display_failure, "Mount: %s", fail_msg);
            return false;
        }
    }
#endif
    return true;
}

bool AP_Arming::fettec_checks(bool display_failure) const
{
#if AP_FETTEC_ONEWIRE_ENABLED
    const AP_FETtecOneWire* f = AP_FETtecOneWire::get_singleton();
    if (f == nullptr) {
        return true;
    }

    // check ESCs are ready
    char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
    if (!f->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
        check_failed(ARMING_CHECK_ALL, display_failure, "FETtec: %s", fail_msg);
        return false;
    }
#endif
    return true;
}

#if AP_ARMING_AUX_AUTH_ENABLED
// request an auxiliary authorisation id.  This id should be used in subsequent calls to set_aux_auth_passed/failed
// returns true on success
bool AP_Arming::get_aux_auth_id(uint8_t& auth_id)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // check we have enough room to allocate another id
    if (aux_auth_count >= aux_auth_count_max) {
        aux_auth_error = true;
        return false;
    }

    // allocate buffer for failure message
    if (aux_auth_fail_msg == nullptr) {
        aux_auth_fail_msg = (char*)calloc(aux_auth_str_len, sizeof(char));
        if (aux_auth_fail_msg == nullptr) {
            aux_auth_error = true;
            return false;
        }
    }
    auth_id = aux_auth_count;
    aux_auth_count++;
    return true;
}

// set auxiliary authorisation passed
void AP_Arming::set_aux_auth_passed(uint8_t auth_id)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // sanity check auth_id
    if (auth_id >= aux_auth_count) {
        return;
    }

    aux_auth_state[auth_id] = AuxAuthStates::AUTH_PASSED;
}

// set auxiliary authorisation failed and provide failure message
void AP_Arming::set_aux_auth_failed(uint8_t auth_id, const char* fail_msg)
{
    WITH_SEMAPHORE(aux_auth_sem);

    // sanity check auth_id
    if (auth_id >= aux_auth_count) {
        return;
    }

    // update state
    aux_auth_state[auth_id] = AuxAuthStates::AUTH_FAILED;

    // store failure message if this authoriser has the lowest auth_id
    for (uint8_t i = 0; i < auth_id; i++) {
        if (aux_auth_state[i] == AuxAuthStates::AUTH_FAILED) {
            return;
        }
    }
    if (aux_auth_fail_msg != nullptr) {
        if (fail_msg == nullptr) {
            strncpy(aux_auth_fail_msg, "Auxiliary authorisation refused", aux_auth_str_len);
        } else {
            strncpy(aux_auth_fail_msg, fail_msg, aux_auth_str_len);
        }
        aux_auth_fail_msg_source = auth_id;
    }
}

bool AP_Arming::aux_auth_checks(bool display_failure)
{
    // handle error cases
    if (aux_auth_error) {
        if (aux_auth_fail_msg == nullptr) {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "memory low for auxiliary authorisation");
        } else {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Too many auxiliary authorisers");
        }
        return false;
    }

    WITH_SEMAPHORE(aux_auth_sem);

    // check results for each auxiliary authorisation id
    bool some_failures         = false;
    bool failure_msg_sent      = false;
    bool waiting_for_responses = false;
    for (uint8_t i = 0; i < aux_auth_count; i++) {
        switch (aux_auth_state[i]) {
            case AuxAuthStates::NO_RESPONSE:
                waiting_for_responses = true;
                break;
            case AuxAuthStates::AUTH_FAILED:
                some_failures = true;
                if (i == aux_auth_fail_msg_source) {
                    check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "%s", aux_auth_fail_msg);
                    failure_msg_sent = true;
                }
                break;
            case AuxAuthStates::AUTH_PASSED:
                break;
        }
    }

    // send failure or waiting message
    if (some_failures) {
        if (!failure_msg_sent) {
            check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Auxiliary authorisation refused");
        }
        return false;
    } else if (waiting_for_responses) {
        check_failed(ARMING_CHECK_AUX_AUTH, display_failure, "Waiting for auxiliary authorisation");
        return false;
    }

    // if we got this far all auxiliary checks must have passed
    return true;
}
#endif // AP_ARMING_AUX_AUTH_ENABLED

bool AP_Arming::generator_checks(bool display_failure) const
{
#if HAL_GENERATOR_ENABLED
    const AP_Generator* generator = AP::generator();
    if (generator == nullptr) {
        return true;
    }
    char failure_msg[50] = {};
    if (!generator->pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "Generator: %s", failure_msg);
        return false;
    }
#endif
    return true;
}

// OpenDroneID Checks
bool AP_Arming::opendroneid_checks(bool display_failure)
{
#if AP_OPENDRONEID_ENABLED
    auto& opendroneid = AP::opendroneid();

    char failure_msg[50] {};
    if (!opendroneid.pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "OpenDroneID: %s", failure_msg);
        return false;
    }
#endif
    return true;
}

// Check for multiple RC in serial protocols
bool AP_Arming::serial_protocol_checks(bool display_failure)
{
    if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_RCIN, 1)) {
        check_failed(display_failure, "Multiple SERIAL ports configured for RC input");
        return false;
    }
    return true;
}

// Check for estop
bool AP_Arming::estop_checks(bool display_failure)
{
    if (!SRV_Channels::get_emergency_stop()) {
        // not emergency-stopped, so no prearm failure:
        return true;
    }
    // vehicle is emergency-stopped; if this *appears* to have been done via switch then we do not fail prearms:
    const RC_Channel* chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP);
    if (chan != nullptr) {
        // an RC channel is configured for arm_emergency_stop option, so estop maybe activated via this switch
        if (chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::LOW) {
            // switch is configured and is in estop position, so likely the reason we are estopped, so no prearm failure
            return true; // no prearm failure
        }
    }
    check_failed(display_failure, "Motors Emergency Stopped");
    return false;
}

bool AP_Arming::pre_arm_checks(bool report)
{
#if !APM_BUILD_COPTER_OR_HELI
    if (armed || arming_required() == Required::NO) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }
#endif

    return hardware_safety_check(report)
#if HAL_HAVE_IMU_HEATER
        & heater_min_temperature_checks(report)
#endif
        & barometer_checks(report)
#if AP_INERTIALSENSOR_ENABLED
        & ins_checks(report)
#endif
        & compass_checks(report)
        & gps_checks(report)
        & battery_checks(report)
        & logging_checks(report)
        & manual_transmitter_checks(report)
        & mission_checks(report)
        & rangefinder_checks(report)
        & servo_checks(report)
        & board_voltage_checks(report)
        & system_checks(report)
        & terrain_checks(report)
        & can_checks(report)
        & generator_checks(report)
        & proximity_checks(report)
        & camera_checks(report)
        & osd_checks(report)
        & mount_checks(report)
        & fettec_checks(report)
        & visodom_checks(report)
#if AP_ARMING_AUX_AUTH_ENABLED
        & aux_auth_checks(report)
#endif
        & disarm_switch_checks(report)
        & fence_checks(report)
        & opendroneid_checks(report)
        & serial_protocol_checks(report)
        & estop_checks(report);
}

bool AP_Arming::arm_checks(AP_Arming::Method method)
{
    if (check_enabled(ARMING_CHECK_RC)) {
        if (!rc_arm_checks(method)) {
            return false;
        }
    }

    // ensure the GPS drivers are ready on any final changes
    if (check_enabled(ARMING_CHECK_GPS_CONFIG)) {
        if (!AP::gps().prepare_for_arming()) {
            return false;
        }
    }

    // note that this will prepare AP_Logger to start logging
    // so should be the last check to be done before arming

    // Note also that we need to PrepForArming() regardless of whether
    // the arming check flag is set - disabling the arming check
    // should not stop logging from working.

    AP_Logger* logger = AP_Logger::get_singleton();
    if (logger->logging_present()) {
        // If we're configured to log, prep it
        logger->PrepForArming();
        if (!logger->logging_started() && check_enabled(ARMING_CHECK_LOGGING)) {
            check_failed(ARMING_CHECK_LOGGING, true, "Logging not started");
            return false;
        }
    }
    return true;
}

bool AP_Arming::mandatory_checks(bool report)
{
    bool ret = true;
#if AP_OPENDRONEID_ENABLED
    ret &= opendroneid_checks(report);
#endif
    ret &= rc_in_calibration_check(report);
    ret &= serial_protocol_checks(report);
    return ret;
}

// returns true if arming occurred successfully
bool AP_Arming::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    if (armed) { // already armed
        return false;
    }

    running_arming_checks = true; // so we show Arm: rather than Disarm: in messages

    if ((!do_arming_checks && mandatory_checks(true)) || (pre_arm_checks(true) && arm_checks(method))) {
        armed = true;

        _last_arm_method = method;

        Log_Write_Arm(!do_arming_checks, method); // note Log_Write_Armed takes forced not do_arming_checks

    } else {
        AP::logger().arming_failure();
        armed = false;
    }

    running_arming_checks = false;

    if (armed && do_arming_checks && checks_to_perform == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Warning: Arming Checks Disabled");
    }

#if HAL_GYROFFT_ENABLED
    // make sure the FFT subsystem is enabled if arming checks have been disabled
    AP_GyroFFT* fft = AP::fft();
    if (fft != nullptr) {
        fft->prepare_for_arming();
    }
#endif

#if AP_TERRAIN_AVAILABLE
    if (armed) {
        // tell terrain we have just armed, so it can setup
        // a reference location for terrain adjustment
        auto* terrain = AP::terrain();
        if (terrain != nullptr) {
            terrain->set_reference_location();
        }
    }
#endif

#if AP_FENCE_ENABLED
    if (armed) {
        auto* fence = AP::fence();
        if (fence != nullptr) {
            // If a fence is set to auto-enable, turn on the fence
            if (fence->auto_enabled() == AC_Fence::AutoEnable::ONLY_WHEN_ARMED) {
                fence->enable(true);
                gcs().send_text(MAV_SEVERITY_INFO, "Fence: auto-enabled");
            }
        }
    }
#endif

    return armed;
}

// returns true if disarming occurred successfully
bool AP_Arming::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (!armed) { // already disarmed
        return false;
    }
    armed               = false;
    _last_disarm_method = method;

    Log_Write_Disarm(!do_disarm_checks, method); // Log_Write_Disarm takes "force"

    check_forced_logging(method);

#if HAL_HAVE_SAFETY_SWITCH
    AP_BoardConfig* board_cfg = AP_BoardConfig::get_singleton();
    if ((board_cfg != nullptr) && (board_cfg->get_safety_button_options() & AP_BoardConfig::BOARD_SAFETY_OPTION_SAFETY_ON_DISARM)) {
        hal.rcout->force_safety_on();
    }
#endif // HAL_HAVE_SAFETY_SWITCH

#if HAL_GYROFFT_ENABLED
    AP_GyroFFT* fft = AP::fft();
    if (fft != nullptr) {
        fft->save_params_on_disarm();
    }
#endif

#if AP_FENCE_ENABLED
    AC_Fence* fence = AP::fence();
    if (fence != nullptr) {
        if (fence->auto_enabled() == AC_Fence::AutoEnable::ONLY_WHEN_ARMED) {
            fence->enable(false);
        }
    }
#endif

    return true;
}

AP_Arming::Required AP_Arming::arming_required() const
{
#if AP_OPENDRONEID_ENABLED
    // cannot be disabled if OpenDroneID is present
    if (AP_OpenDroneID::get_singleton() != nullptr && AP::opendroneid().enabled()) {
        if (require != Required::YES_MIN_PWM && require != Required::YES_ZERO_PWM) {
            return Required::YES_MIN_PWM;
        }
    }
#endif
    return require;
}

// Copter and sub share the same RC input limits
// Copter checks that min and max have been configured by default, Sub does not
bool AP_Arming::rc_checks_copter_sub(const bool display_failure, const RC_Channel* channels[4]) const
{
    // set rc-checks to success if RC checks are disabled
    if (!check_enabled(ARMING_CHECK_RC)) {
        return true;
    }

    bool ret = true;

    const char* channel_names[] = { "Roll", "Pitch", "Throttle", "Yaw" };

    for (uint8_t i = 0; i < ARRAY_SIZE(channel_names); i++) {
        const RC_Channel* channel      = channels[i];
        const char*       channel_name = channel_names[i];
        // check if radio has been calibrated
        if (channel->get_radio_min() > RC_Channel::RC_CALIB_MIN_LIMIT_PWM) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio min too high", channel_name);
            ret = false;
        }
        if (channel->get_radio_max() < RC_Channel::RC_CALIB_MAX_LIMIT_PWM) {
            check_failed(ARMING_CHECK_RC, display_failure, "%s radio max too low", channel_name);
            ret = false;
        }
    }
    return ret;
}

// check visual odometry is working
bool AP_Arming::visodom_checks(bool display_failure) const
{
    if (!check_enabled(ARMING_CHECK_VISION)) {
        return true;
    }

#if HAL_VISUALODOM_ENABLED
    AP_VisualOdom* visual_odom = AP::visualodom();
    if (visual_odom != nullptr) {
        char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        if (!visual_odom->pre_arm_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_VISION, display_failure, "VisOdom: %s", fail_msg);
            return false;
        }
    }
#endif

    return true;
}

// check disarm switch is asserted
bool AP_Arming::disarm_switch_checks(bool display_failure) const
{
    const RC_Channel* chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DISARM);
    if (chan != nullptr && chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
        check_failed(display_failure, "Disarm Switch on");
        return false;
    }

    return true;
}

void AP_Arming::Log_Write_Arm(const bool forced, const AP_Arming::Method method)
{
    const struct log_Arm_Disarm pkt {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
            time_us : AP_HAL::micros64(),
                      arm_state : is_armed(),
                                  arm_checks : get_enabled_checks(),
                                               forced : forced,
                                                        method : (uint8_t)method,
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
    AP::logger().Write_Event(LogEvent::ARMED);
}

void AP_Arming::Log_Write_Disarm(const bool forced, const AP_Arming::Method method)
{
    const struct log_Arm_Disarm pkt {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
            time_us : AP_HAL::micros64(),
                      arm_state : is_armed(),
                                  arm_checks : 0,
                                  forced : forced,
                                           method : (uint8_t)method
    };
    AP::logger().WriteCriticalBlock(&pkt, sizeof(pkt));
    AP::logger().Write_Event(LogEvent::DISARMED);
}

// check if we should keep logging after disarming
void AP_Arming::check_forced_logging(const AP_Arming::Method method)
{
    // keep logging if disarmed for a bad reason
    switch (method) {
        case Method::TERMINATION:
        case Method::CPUFAILSAFE:
        case Method::BATTERYFAILSAFE:
        case Method::AFS:
        case Method::ADSBCOLLISIONACTION:
        case Method::PARACHUTE_RELEASE:
        case Method::CRASH:
        case Method::FENCEBREACH:
        case Method::RADIOFAILSAFE:
        case Method::GCSFAILSAFE:
        case Method::TERRRAINFAILSAFE:
        case Method::FAILSAFE_ACTION_TERMINATE:
        case Method::TERRAINFAILSAFE:
        case Method::BADFLOWOFCONTROL:
        case Method::EKFFAILSAFE:
        case Method::GCS_FAILSAFE_SURFACEFAILED:
        case Method::GCS_FAILSAFE_HOLDFAILED:
        case Method::PILOT_INPUT_FAILSAFE:
        case Method::DEADRECKON_FAILSAFE:
        case Method::BLACKBOX:
            // keep logging for longer if disarmed for a bad reason
            AP::logger().set_long_log_persist(true);
            return;

        case Method::RUDDER:
        case Method::MAVLINK:
        case Method::AUXSWITCH:
        case Method::MOTORTEST:
        case Method::SCRIPTING:
        case Method::SOLOPAUSEWHENLANDED:
        case Method::LANDED:
        case Method::MISSIONEXIT:
        case Method::DISARMDELAY:
        case Method::MOTORDETECTDONE:
        case Method::TAKEOFFTIMEOUT:
        case Method::AUTOLANDED:
        case Method::TOYMODELANDTHROTTLE:
        case Method::TOYMODELANDFORCE:
        case Method::LANDING:
        case Method::DDS:
        case Method::UNKNOWN:
            AP::logger().set_long_log_persist(false);
            return;
    }
}

AP_Arming* AP_Arming::_singleton = nullptr;

/*
 * Get the AP_Arming singleton
 */
AP_Arming* AP_Arming::get_singleton()
{
    return AP_Arming::_singleton;
}

namespace AP {

AP_Arming& arming()
{
    return *AP_Arming::get_singleton();
}

};

#pragma GCC diagnostic pop
