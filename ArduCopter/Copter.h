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
#pragma once

/*
  这是主要的Copter类
 */

////////////////////////////////////////////////////////////////////////////////
// 包含的头文件
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

// 通用依赖
#include <AP_Common/AP_Common.h>           // ArduPilot库的通用定义和实用程序
#include <AP_Common/Location.h>            // 包含位置类的实现
#include <AP_Param/AP_Param.h>             // 用于管理和存储系统通用兴趣变量的系统
#include <StorageManager/StorageManager.h> // 用于管理hal.storage的库，允许将存储偏移映射到可用存储的向后兼容映射

// 应用程序依赖
#include <AC_AttitudeControl/AC_AttitudeControl_Heli.h>       // 传统直升机的姿态控制库
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>      // 姿态控制库
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h> // 6自由度姿态控制库
#include <AC_AttitudeControl/AC_CommandModel.h>               // 命令模型库
#include <AC_AttitudeControl/AC_PosControl.h>                 // 位置控制库
#include <AC_AutoTune/AC_AutoTune_Heli.h>                     // ArduCopter自动调整库，支持直升机自动调整
#include <AC_AutoTune/AC_AutoTune_Multi.h>                    // ArduCopter自动调整库，支持多旋翼自动调整
#include <AC_InputManager/AC_InputManager.h>                  // 飞行员输入处理库
#include <AC_InputManager/AC_InputManager_Heli.h>             // 直升机特定的飞行员输入处理库
#include <AC_PrecLand/AC_PrecLand_config.h>
#include <AC_Sprayer/AC_Sprayer.h>                // 农田喷洒库
#include <AC_WPNav/AC_Circle.h>                   // 圆形导航库
#include <AC_WPNav/AC_Loiter.h>                   // ArduCopter悬停模式库
#include <AC_WPNav/AC_WPNav.h>                    // ArduCopter航点导航库
#include <AP_ADSB/AP_ADSB.h>                      // 基于ADS-B RF的碰撞回避模块库
#include <AP_AHRS/AP_AHRS.h>                      // ArduPilot AHRS（姿态航向参考系统）接口库
#include <AP_AccelCal/AP_AccelCal.h>              // 加速度计校准的接口和数学
#include <AP_Arming/AP_Arming.h>                  // ArduPilot电机解锁库
#include <AP_BattMonitor/AP_BattMonitor.h>        // 电池监控库
#include <AP_Declination/AP_Declination.h>        // ArduPilot Mega磁偏角辅助库
#include <AP_InertialNav/AP_InertialNav.h>        // 惯性导航库
#include <AP_InertialSensor/AP_InertialSensor.h>  // ArduPilot Mega惯性传感器（加速度计和陀螺仪）库
#include <AP_LandingGear/AP_LandingGear.h>        // 起落架库
#include <AP_Logger/AP_Logger.h>                  // ArduPilot Mega Flash存储库
#include <AP_Math/AP_Math.h>                      // ArduPilot Mega矢量/矩阵数学库
#include <AP_Mission/AP_Mission.h>                // 任务命令库
#include <AP_Mission/AP_Mission_ChangeDetector.h> // 任务命令更改检测库
#include <AP_Motors/AP_Motors.h>                  // AP电机库
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Parachute/AP_Parachute.h>             // ArduPilot降落伞释放库
#include <AP_Proximity/AP_Proximity.h>             // ArduPilot接近传感器库
#include <AP_RCMapper/AP_RCMapper.h>               // RC输入映射库
#include <AP_SmartRTL/AP_SmartRTL.h>               // ArduPilot智能返航模式（SRTL）库
#include <AP_Stats/AP_Stats.h>                     // 统计库
#include <AP_TempCalibration/AP_TempCalibration.h> // 温度校准库
#include <AP_Vehicle/AP_Vehicle.h>                 // AHRS构建所需
#include <AP_Winch/AP_Winch_config.h>
#include <Filter/Filter.h> // 过滤器库

// 配置
#include "config.h"  // 包含配置文件
#include "defines.h" // 包含定义文件

#if FRAME_CONFIG == HELI_FRAME
# define AC_AttitudeControl_t AC_AttitudeControl_Heli // 如果框架配置为直升机，则使用直升机的姿态控制器
#else
# define AC_AttitudeControl_t AC_AttitudeControl_Multi // 否则使用多旋翼的姿态控制器
#endif

#if FRAME_CONFIG == HELI_FRAME
# define MOTOR_CLASS AP_MotorsHeli // 如果框架配置为直升机，则使用直升机电机类
#else
# define MOTOR_CLASS AP_MotorsMulticopter // 否则使用多旋翼电机类
#endif

#if MODE_AUTOROTATE_ENABLED == ENABLED
# include <AC_Autorotation/AC_Autorotation.h> // 自转控制器
#endif

#include "AP_Arming.h"
#include "AP_Rally.h" // 集结点库
#include "GCS_Copter.h"
#include "GCS_Mavlink.h"
#include "RC_Channel.h" // RC通道库

#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
# include "AP_ExternalControl_Copter.h"
#endif

#include <AP_Beacon/AP_Beacon_config.h>
#if AP_BEACON_ENABLED
# include <AP_Beacon/AP_Beacon.h>
#endif

#if AC_AVOID_ENABLED == ENABLED
# include <AC_Avoidance/AC_Avoid.h>
#endif
#if AC_OAPATHPLANNER_ENABLED == ENABLED
# include <AC_Avoidance/AP_OAPathPlanner.h>
# include <AC_WPNav/AC_WPNav_OA.h>
#endif
#include <AP_Gripper/AP_Gripper_config.h>
#if AP_GRIPPER_ENABLED
# include <AP_Gripper/AP_Gripper.h>
#endif
#if AC_PRECLAND_ENABLED
# include <AC_PrecLand/AC_PrecLand.h>
# include <AC_PrecLand/AC_PrecLand_StateMachine.h>
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
# include <AP_Follow/AP_Follow.h>
#endif
#if AP_TERRAIN_AVAILABLE
# include <AP_Terrain/AP_Terrain.h>
#endif
#if RANGEFINDER_ENABLED == ENABLED
# include <AP_RangeFinder/AP_RangeFinder.h>
#endif

#include <AP_Mount/AP_Mount.h>

#include <AP_Camera/AP_Camera.h>

#if HAL_BUTTON_ENABLED
# include <AP_Button/AP_Button.h>
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
# include <AP_OSD/AP_OSD.h>
#endif

#if ADVANCED_FAILSAFE == ENABLED
# include "afs_copter.h"
#endif
#if TOY_MODE_ENABLED == ENABLED
# include "toy_mode.h"
#endif
#if AP_WINCH_ENABLED
# include <AP_Winch/AP_Winch.h>
#endif
#include <AP_RPM/AP_RPM.h>

#if AP_SCRIPTING_ENABLED
# include <AP_Scripting/AP_Scripting.h>
#endif

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
# include <AC_CustomControl/AC_CustomControl.h> // Custom control library
#endif

#if AC_AVOID_ENABLED && !AP_FENCE_ENABLED
# error AC_Avoidance relies on AP_FENCE_ENABLED which is disabled
#endif

#if AC_OAPATHPLANNER_ENABLED && !AP_FENCE_ENABLED
# error AP_OAPathPlanner relies on AP_FENCE_ENABLED which is disabled
#endif

// Local modules
#ifdef USER_PARAMS_ENABLED
# include "UserParameters.h"
#endif
#include "Parameters.h"
#if HAL_ADSB_ENABLED
# include "avoidance_adsb.h"
#endif

#include "mode.h"

#include <AP_SAWProtocol/AP_SAWProtocol.h>

class Copter : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Copter;
    friend class GCS_Copter;
    friend class AP_Rally_Copter;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Avoidance_Copter;

#if ADVANCED_FAILSAFE == ENABLED
    friend class AP_AdvancedFailsafe_Copter;
#endif
    friend class AP_Arming_Copter;
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif
    friend class ToyMode;
    friend class RC_Channel_Copter;
    friend class RC_Channels_Copter;

    friend class AutoTune;

    friend class Mode;
    friend class ModeAcro;
    friend class ModeAcro_Heli;
    friend class ModeAltHold;
    friend class ModeAuto;
    friend class ModeAutoTune;
    friend class ModeAvoidADSB;
    friend class ModeBrake;
    friend class ModeCircle;
    friend class ModeDrift;
    friend class ModeFlip;
    friend class ModeFlowHold;
    friend class ModeFollow;
    friend class ModeGuided;
    friend class ModeLand;
    friend class ModeLoiter;
    friend class ModePosHold;
    friend class ModeRTL;
    friend class ModeSmartRTL;
    friend class ModeSport;
    friend class ModeStabilize;
    friend class ModeStabilize_Heli;
    friend class ModeSystemId;
    friend class ModeThrow;
    friend class ModeZigZag;
    friend class ModeAutorotate;
    friend class ModeTurtle;

    friend class _AutoTakeoff;

    friend class PayloadPlace;

    Copter(void);

private:
    // 主要飞行器参数传递给多个库
    AP_MultiCopter aparm;

    // 所有全局参数都包含在 'g' 类中
    Parameters   g;
    ParametersG2 g2;

    // 用于检测来自地面站的 MAVLink 应答以停止罗盘干扰检测
    uint8_t command_ack_counter;

    // 主要输入控制通道
    RC_Channel* channel_roll;
    RC_Channel* channel_pitch;
    RC_Channel* channel_throttle;
    RC_Channel* channel_yaw;

    AP_Logger logger;

    // 飞行模式便利数组
    AP_Int8*      flight_modes;
    const uint8_t num_flight_modes = 6;

    // 测距仪状态结构体
    struct RangeFinderState {
        bool               enabled : 1;
        bool               alt_healthy : 1; // 如果我们可以信任测距仪的高度，则为 true
        int16_t            alt_cm;          // 经过倾斜补偿的高度（单位：厘米）来自测距仪
        float              inertial_alt_cm; // 最后一次测距仪采样时的惯性高度
        uint32_t           last_healthy_ms;
        LowPassFilterFloat alt_cm_filt;             // 高度滤波器
        int16_t            alt_cm_glitch_protected; // 上一个受到保护免受干扰的高度
        int8_t             glitch_count;            // 非零数字表示测距仪存在干扰
        uint32_t           glitch_cleared_ms;       // 系统时间清除干扰
        float              terrain_offset_cm;       // 经过滤波的地形偏移（例如，地形相对于 EKF 原点的高度）
    } rangefinder_state, rangefinder_up_state;

    // 返回使用惯性高度插值的测距仪高度
    bool get_rangefinder_height_interpolated_cm(int32_t& ret) const;

    class SurfaceTracking {
    public:
        // 更新表面偏移 - 管理位置控制器的垂直偏移，以跟随测距仪测量的地面或天花板水平面。
        void update_surface_offset();

        // 获取目标高度（单位：厘米）相对地面
        bool get_target_alt_cm(float& target_alt_cm) const;
        // 设置目标高度（单位：厘米）相对地面
        void set_target_alt_cm(float target_alt_cm);

        // 用于记录目标距离（单位：米）的获取和设置
        bool get_target_dist_for_logging(float& target_dist) const;
        // 获取实际距离（单位：米）以进行记录
        float get_dist_for_logging() const;
        // 使记录无效
        void invalidate_for_logging() { valid_for_logging = false; }

        // 表面跟踪水平面
        enum class Surface {
            NONE    = 0,
            GROUND  = 1,
            CEILING = 2
        };
        // 设置要跟踪的水平面
        void set_surface(Surface new_surface);
        // 初始化表面跟踪
        void init(Surface surf) { surface = surf; }

    private:
        Surface  surface;                // 当前跟踪的水平面
        uint32_t last_update_ms;         // 上次更新目标高度的系统时间
        uint32_t last_glitch_cleared_ms; // 上次处理干扰恢复的系统时间
        bool     valid_for_logging;      // 如果我们有期望的目标高度，则为 true
        bool     reset_target;           // 如果由于跟踪的水平面变化，应重置目标高度，则为 true
    } surface_tracking;

// 如果启用了 RPM 传感器，创建 RPM 传感器对象
#if AP_RPM_ENABLED
    AP_RPM rpm_sensor;
#endif

    // 惯性导航 EKF - 不同的视角
    AP_AHRS_View* ahrs_view;

    // 起飞/降落管理类
    AP_Arming_Copter arming;

// 光流传感器
#if AP_OPTICALFLOW_ENABLED
    AP_OpticalFlow optflow;
#endif

// 外部控制库
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Copter external_control;
#endif

    // 上次从 EKF 记录的偏航重置的系统时间（以毫秒为单位）
    uint32_t ekfYawReset_ms;
    // EKF 主核心
    int8_t ekf_primary_core;

    // 振动检测
    struct {
        bool     high_vibes; // 在检测到高振动时为 true
        uint32_t start_ms;   // 上次检测到高振动的系统时间
        uint32_t clear_ms;   // 高振动停止的系统时间
    } vibration_check;

    // 起飞检查
    uint32_t takeoff_check_warning_ms; // 上次警告用户起飞检查失败的系统时间

    // GCS 选择
    GCS_Copter  _gcs; // 避免使用此变量；使用 gcs() 函数
    GCS_Copter& gcs() { return _gcs; }

// 用户自定义变量
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    // 全局变量的文档说明
    typedef union {
        struct {
            uint8_t unused1 : 1;                   // 0
            uint8_t unused_was_simple_mode : 2;    // 1,2
            uint8_t pre_arm_rc_check : 1;          // 3       // 如果 RC 输入的预防撤销检查已成功完成，则为 true
            uint8_t pre_arm_check : 1;             // 4       // 如果已执行所有预防撤销检查（RC、加速度校准、GPS 锁定）
            uint8_t auto_armed : 1;                // 5       // 防止自动任务在抬高油门之前开始
            uint8_t logging_started : 1;           // 6       // 如果记录已开始
            uint8_t land_complete : 1;             // 7       // 如果我们检测到着陆
            uint8_t new_radio_frame : 1;           // 8       // 如果我们有新的 PWM 数据来自遥控器以采取行动
            uint8_t usb_connected_unused : 1;      // 9       // 未使用
            uint8_t rc_receiver_present : 1;       // 10      // 如果我们有遥控器接收器存在（即，如果我们曾经接收到更新）
            uint8_t compass_mot : 1;               // 11      // 如果我们当前正在执行罗盘干扰检查
            uint8_t motor_test : 1;                // 12      // 如果我们当前正在执行电机测试
            uint8_t initialised : 1;               // 13      // 一旦 init_ardupilot 函数完成，就为 true。在此完成之前不会向 GCS 发送扩展状态
            uint8_t land_complete_maybe : 1;       // 14      // 如果我们可能已经着陆（较不严格的 land_complete 版本）
            uint8_t throttle_zero : 1;             // 15      // 如果油门杆位于零，去抖动，确定飞手是否打算在不使用电机互锁时关闭电机
            uint8_t system_time_set_unused : 1;    // 16      // 如果系统时间已从 GPS 设置
            uint8_t gps_glitching : 1;             // 17      // 如果 GPS 故障影响导航精度
            uint8_t using_interlock : 1;           // 20      // 辅助开关电机互锁功能正在使用
            uint8_t land_repo_active : 1;          // 21      // 如果飞手正在覆盖着陆位置
            uint8_t motor_interlock_switch : 1;    // 22      // 如果飞手请求电机互锁启用
            uint8_t in_arming_delay : 1;           // 23      // 当我们武装时，但正在等待旋转电机时为 true
            uint8_t initialised_params : 1;        // 24      // 所有参数已初始化时为 true。在此之前，我们无法将参数发送给 GCS
            uint8_t unused3 : 1;                   // 25      // 曾经是 compass_init_location；当罗盘的初始位置已设置时为 true
            uint8_t unused2 : 1;                   // 26      // 允许辅助开关 rc_override
            uint8_t armed_with_airmode_switch : 1; // 27      // 使用 arming 开关武装
            uint8_t prec_land_active : 1;          // 28      // 如果 precland 活跃
        };
        uint32_t value;
    } ap_t;

    ap_t ap; // ArduPilot 全局状态

    AirMode air_mode;     // 空中模式，0 = 未配置；1 = 禁用；2 = 启用；
    bool    force_flying; // 强制飞行模式，当为 true 时启用；

    // 使用静态断言检查 ap_t 和 uint32_t 的大小是否相同，必须保证 ap_t 的大小等于 uint32_t
    static_assert(sizeof(uint32_t) == sizeof(ap), "ap_t must be uint32_t");

    // 这是飞行控制系统的状态
    // 定义了多个状态，如 STABILIZE、ACRO 等
    Mode*        flightmode;        // 当前飞行模式
    Mode::Number prev_control_mode; // 上一个控制模式

    RCMapper rcmap; // 遥控器通道映射

    // 武装时的惯性导航高度
    float arming_altitude_m; // 单位：米

    // 失效安全
    struct {
        uint32_t terrain_first_failure_ms; // 第一次地形数据访问失败的时间 - 用于计算失败的持续时间
        uint32_t terrain_last_failure_ms;  // 最近一次地形数据访问失败的时间

        int8_t radio_counter; // 油门低于油门失败值的迭代次数

        uint8_t radio : 1;      // 无线电失效的状态标志
        uint8_t gcs : 1;        // 地面站失效的状态标志
        uint8_t ekf : 1;        // 如果发生 ekf 失效则为 true
        uint8_t terrain : 1;    // 如果发生缺失地形数据的失效则为 true
        uint8_t adsb : 1;       // 如果发生与 ADS-B 相关的失效则为 true
        uint8_t deadreckon : 1; // 如果触发了死推定失效则为 true
    } failsafe;

    // 如果任何一种失效安全触发，返回 true
    bool any_failsafe_triggered() const
    {
        return failsafe.radio || battery.has_failsafed() || failsafe.gcs || failsafe.ekf || failsafe.terrain || failsafe.adsb || failsafe.deadreckon;
    }

    // 死推定状态
    struct {
        bool     active;   // 如果正在进行死推定（使用估计的空速进行位置估计，没有位置或速度源）
        bool     timeout;  // 如果死推定已超时，EKF 的位置和速度估计不再可信
        uint32_t start_ms; // EKF 开始死推定的系统时间
    } dead_reckoning;

    // 电机输出
    MOTOR_CLASS*                      motors;          // 电机对象
    const struct AP_Param::GroupInfo* motors_var_info; // 电机参数信息

    int32_t  _home_bearing;  // 起始点方向
    uint32_t _home_distance; // 起始点距离

    // SIMPLE 模式
    // 用于跟踪 Simple 模式下飞行器的方向。该值在每次解锁或在超级简单模式下飞行器离开家的 20 米半径范围时重置。
    enum class SimpleMode {
        NONE        = 0,
        SIMPLE      = 1,
        SUPERSIMPLE = 2,
    } simple_mode;

    float   simple_cos_yaw;            // Simple 模式的方向余弦值
    float   simple_sin_yaw;            // Simple 模式的方向正弦值
    int32_t super_simple_last_bearing; // 最近一次的超级简单模式方向
    float   super_simple_cos_yaw;      // 超级简单模式的方向余弦值
    float   super_simple_sin_yaw;      // 超级简单模式的方向正弦值

    // 存储解锁时的初始方向 - 初始简单模式方向在超级简单模式下会被修改，因此不适用
    int32_t initial_armed_bearing;

    // 电池传感器
    AP_BattMonitor battery { MASK_LOG_CURRENT,
                             FUNCTOR_BIND_MEMBER(&Copter::handle_battery_failsafe, void, const char*, const int8_t),
                             _failsafe_priorities };

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif

    // 海拔
    int32_t               baro_alt;             // 气压计测得的相对家的高度（单位：厘米）
    LowPassFilterVector3f land_accel_ef_filter; // 用于着陆和坠机检测测试的加速度

    // 过滤后的飞手油门输入，用于在油门保持高位时取消着陆
    LowPassFilterFloat rc_throttle_control_in_filter;

    // 3D 位置矢量
    // 飞行器的当前位置（高度相对于家）
    Location current_loc;

    // 惯性导航
    AP_InertialNav inertial_nav;

    // 姿态、位置和航点导航对象
    // 待办事项：将惯性导航移至上方或其他导航变量移至此处
    AC_AttitudeControl_t* attitude_control;
    AC_PosControl*        pos_control;
    AC_WPNav*             wp_nav;
    AC_Loiter*            loiter_nav;

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    AC_CustomControl custom_control { ahrs_view, attitude_control, motors, scheduler.get_loop_period_s() };
#endif

#if MODE_CIRCLE_ENABLED == ENABLED
    AC_Circle* circle_nav;
#endif

    // 系统计时器
    // --------------
    // arm_time_ms - 记录飞行器解锁时间。如果未解锁，则为零。
    uint32_t arm_time_ms;

    // 用于退出滚转和俯仰自动修正功能
    uint8_t auto_trim_counter;
    bool    auto_trim_started = false;

// 相机
#if AP_CAMERA_ENABLED
    AP_Camera camera { MASK_LOG_CAMERA };
#endif

// 相机/天线支架跟踪和稳定控制
#if HAL_MOUNT_ENABLED
    AP_Mount camera_mount;
#endif

#if AC_AVOID_ENABLED == ENABLED
    AC_Avoid avoid;
#endif

// 集结点库
#if HAL_RALLY_ENABLED
    AP_Rally_Copter rally;
#endif

// 农药喷洒器
#if HAL_SPRAYER_ENABLED
    AC_Sprayer sprayer;
#endif

// 降落伞释放
#if PARACHUTE == ENABLED
    AP_Parachute parachute;
#endif

    AP_SAWProtocol ap_saw;

// 起落架控制器
#if AP_LANDINGGEAR_ENABLED
    AP_LandingGear landinggear;
#endif

// 地形处理
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain;
#endif

// 精准着陆
#if AC_PRECLAND_ENABLED
    AC_PrecLand              precland;
    AC_PrecLand_StateMachine precland_statemachine;
#endif

// 飞行员输入管理库
// 目前仅用于直升机
#if FRAME_CONFIG == HELI_FRAME
    AC_InputManager_Heli input_manager;
#endif

#if HAL_ADSB_ENABLED
    AP_ADSB adsb;

    // 避障 ADS-B 启用飞行器（通常是有人驾驶的飞行器）的避障
    AP_Avoidance_Copter avoidance_adsb { adsb };
#endif

    // 最后一次有效的遥控输入时间
    uint32_t last_radio_update_ms;

    // 最后一次电调校准通知更新时间
    uint32_t esc_calibration_notify_update_ms;

    // 顶层逻辑
    // 设置 var_info 表
    AP_Param param_loader;

#if FRAME_CONFIG == HELI_FRAME
    // 模式滤波器，用于拒绝遥控输入干扰。滤波器大小为5，绘制第4个元素，因此可以拒绝3次低干扰和1次高干扰。
    // 这是因为任何“关”干扰对于运行电调速度控制的直升机可能会带来严重问题。即使一个“关”帧也会导致旋翼急剧减速并需要很长时间重新启动。
    ModeFilterInt16_Size5 rotor_speed_deglitch_filter { 4 };

    // 直升机标志
    typedef struct {
        uint8_t dynamic_flight : 1;  // 0   // 如果我们正在以显著速度运动（用于开启/关闭漏电I项）
        uint8_t inverted_flight : 1; // 1   // 倒置飞行模式标志
        uint8_t in_autorotation : 1; // 2   // 当直升机处于自转状态时为真
        bool    coll_stk_low;        // 3   // 集体操纵杆在下限时为真
    } heli_flags_t;
    heli_flags_t heli_flags;

    int16_t hover_roll_trim_scalar_slew;
#endif

    // 地效检测器
    struct {
        bool     takeoff_expected;
        bool     touchdown_expected;
        uint32_t takeoff_time_ms;
        float    takeoff_alt_cm;
    } gndeffect_state;

    bool standby_active;

    static const AP_Scheduler::Task  scheduler_tasks[];
    static const AP_Param::Info      var_info[];
    static const struct LogStructure log_structure[];

    // 电调校准的枚举
    enum ESCCalibrationModes : uint8_t {
        ESCCAL_NONE                         = 0,
        ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
        ESCCAL_PASSTHROUGH_ALWAYS           = 2,
        ESCCAL_AUTO                         = 3,
        ESCCAL_DISABLED                     = 9,
    };

    enum class FailsafeAction : uint8_t {
        NONE               = 0,
        LAND               = 1,
        RTL                = 2,
        SMARTRTL           = 3,
        SMARTRTL_LAND      = 4,
        TERMINATE          = 5,
        AUTO_DO_LAND_START = 6,
        BRAKE_LAND         = 7
    };

    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO           = (1 << 0), // 1
        GCS_CONTINUE_IF_AUTO          = (1 << 1), // 2
        RC_CONTINUE_IF_GUIDED         = (1 << 2), // 4
        CONTINUE_IF_LANDING           = (1 << 3), // 8
        GCS_CONTINUE_IF_PILOT_CONTROL = (1 << 4), // 16
        RELEASE_GRIPPER               = (1 << 5), // 32
    };

    enum class FlightOptions {
        DISABLE_THRUST_LOSS_CHECK      = (1 << 0), // 1
        DISABLE_YAW_IMBALANCE_WARNING  = (1 << 1), // 2
        RELEASE_GRIPPER_ON_THRUST_LOSS = (1 << 2), // 4
    };

    static constexpr int8_t _failsafe_priorities[] = {
        (int8_t)FailsafeAction::TERMINATE,
        (int8_t)FailsafeAction::LAND,
        (int8_t)FailsafeAction::RTL,
        (int8_t)FailsafeAction::SMARTRTL_LAND,
        (int8_t)FailsafeAction::SMARTRTL,
        (int8_t)FailsafeAction::NONE,
        -1 // 优先级列表必须以 -1 作为标志结束
    };

#define FAILSAFE_LAND_PRIORITY 1
    static_assert(_failsafe_priorities[FAILSAFE_LAND_PRIORITY] == (int8_t)FailsafeAction::LAND,
                  "FAILSAFE_LAND_PRIORITY must match the entry in _failsafe_priorities");
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // AP_State.cpp
    void set_auto_armed(bool b);
    void set_simple_mode(SimpleMode b);
    void set_failsafe_radio(bool b);
    void set_failsafe_gcs(bool b);
    void update_using_interlock();

    // Copter.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task*& tasks,
                             uint8_t&                   task_count,
                             uint32_t&                  log_bit) override;
#if AP_SCRIPTING_ENABLED
# if MODE_GUIDED_ENABLED == ENABLED
    bool start_takeoff(float alt) override;
    bool set_target_location(const Location& target_loc) override;
    bool set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt) override;
    bool set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel) override;
    bool set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative) override;
    bool set_target_velocity_NED(const Vector3f& vel_ned) override;
    bool set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw) override;
    bool set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs) override;
# endif
# if MODE_CIRCLE_ENABLED == ENABLED
    bool get_circle_radius(float& radius_m) override;
    bool set_circle_rate(float rate_dps) override;
# endif
    bool set_desired_speed(float speed) override;
# if MODE_AUTO_ENABLED == ENABLED
    bool nav_scripting_enable(uint8_t mode) override;
    bool nav_script_time(uint16_t& id, uint8_t& cmd, float& arg1, float& arg2, int16_t& arg3, int16_t& arg4) override;
    void nav_script_time_done(uint16_t id) override;
# endif
    // lua scripts use this to retrieve EKF failsafe state
    // returns true if the EKF failsafe has triggered
    bool has_ekf_failsafed() const override;
#endif // AP_SCRIPTING_ENABLED
    bool is_landing() const override;
    bool is_taking_off() const override;
    void rc_loop();
    void throttle_loop();
    void update_batt_compass(void);
    void loop_rate_logging();
    void ten_hz_logging_loop();
    void twentyfive_hz_logging();
    void three_hz_loop();
    void one_hz_loop();
    void init_simple_bearing();
    void update_simple_mode(void);
    void update_super_simple_bearing(bool force_update);
    void read_AHRS(void);
    void update_altitude();
    bool get_wp_distance_m(float& distance) const override;
    bool get_wp_bearing_deg(float& bearing) const override;
    bool get_wp_crosstrack_error_m(float& xtrack_error) const override;
    bool get_rate_ef_targets(Vector3f& rate_ef_targets) const override;

    // Attitude.cpp
    void     update_throttle_hover();
    float    get_pilot_desired_climb_rate(float throttle_control);
    float    get_non_takeoff_throttle();
    void     set_accel_throttle_I_from_pilot_throttle();
    void     rotate_body_frame_to_NE(float& x, float& y);
    uint16_t get_pilot_speed_dn() const;
    void     run_rate_controller();

#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    void run_custom_controller() { custom_control.update(); }
#endif

    // avoidance.cpp
    void low_alt_avoidance();

#if HAL_ADSB_ENABLED
    // avoidance_adsb.cpp
    void avoidance_adsb_update(void);
#endif

    // baro_ground_effect.cpp
    void update_ground_effect_detector(void);
    void update_ekf_terrain_height_stable();

    // commands.cpp
    void update_home_from_EKF();
    void set_home_to_current_location_inflight();
    bool set_home_to_current_location(bool lock) WARN_IF_UNUSED;
    bool set_home(const Location& loc, bool lock) WARN_IF_UNUSED;
    bool far_from_EKF_origin(const Location& loc);

    // compassmot.cpp
    MAV_RESULT mavlink_compassmot(const GCS_MAVLINK& gcs_chan);

    // crash_check.cpp
    void               crash_check();
    void               thrust_loss_check();
    void               yaw_imbalance_check();
    LowPassFilterFloat yaw_I_filt { 0.05f };
    uint32_t           last_yaw_warn_ms;
    void               parachute_check();
    void               parachute_release();
    void               parachute_manual_release();

    // ekf_check.cpp
    void ekf_check();
    bool ekf_over_threshold();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);
    void failsafe_ekf_recheck();
    void check_ekf_reset();
    void check_vibration();

    // esc_calibration.cpp
    void esc_calibration_startup_check();
    void esc_calibration_passthrough();
    void esc_calibration_auto();
    void esc_calibration_notify();
    void esc_calibration_setup();

    // events.cpp
    bool failsafe_option(FailsafeOption opt) const;
    void failsafe_radio_on_event();
    void failsafe_radio_off_event();
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    void failsafe_gcs_check();
    void failsafe_gcs_on_event(void);
    void failsafe_gcs_off_event(void);
    void failsafe_terrain_check();
    void failsafe_terrain_set_status(bool data_ok);
    void failsafe_terrain_on_event();
    void gpsglitch_check();
    void failsafe_deadreckon_check();
    void set_mode_RTL_or_land_with_pause(ModeReason reason);
    void set_mode_SmartRTL_or_RTL(ModeReason reason);
    void set_mode_SmartRTL_or_land_with_pause(ModeReason reason);
    void set_mode_auto_do_land_start_or_RTL(ModeReason reason);
    void set_mode_brake_or_land_with_pause(ModeReason reason);
    bool should_disarm_on_failsafe();
    void do_failsafe_action(FailsafeAction action, ModeReason reason);
    void announce_failsafe(const char* type, const char* action_undertaken = nullptr);

    // failsafe.cpp
    void failsafe_enable();
    void failsafe_disable();
#if ADVANCED_FAILSAFE == ENABLED
    void afs_fs_check(void);
#endif

    // fence.cpp
#if AP_FENCE_ENABLED
    void fence_check();
#endif

    // heli.cpp
    void  heli_init();
    void  check_dynamic_flight(void);
    bool  should_use_landing_swash() const;
    void  update_heli_control_dynamics(void);
    void  heli_update_landing_swash();
    float get_pilot_desired_rotor_speed() const;
    void  heli_update_rotor_speed_targets();
    void  heli_update_autorotation();
    void  update_collective_low_flag(int16_t throttle_control);

    // inertia.cpp
    void read_inertia();

    // landing_detector.cpp
    void update_land_and_crash_detectors();
    void update_land_detector();
    void set_land_complete(bool b);
    void set_land_complete_maybe(bool b);
    void update_throttle_mix();

#if AP_LANDINGGEAR_ENABLED
    // landing_gear.cpp
    void landinggear_update();
#endif

    // standby.cpp
    void standby_update();

    // Log.cpp
    void Log_Write_Control_Tuning();
    void Log_Write_Attitude();
    void Log_Write_EKF_POS();
    void Log_Write_PIDS();
    void Log_Write_Data(LogDataID id, int32_t value);
    void Log_Write_Data(LogDataID id, uint32_t value);
    void Log_Write_Data(LogDataID id, int16_t value);
    void Log_Write_Data(LogDataID id, uint16_t value);
    void Log_Write_Data(LogDataID id, float value);
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max);
    void Log_Video_Stabilisation();
#if FRAME_CONFIG == HELI_FRAME
    void Log_Write_Heli(void);
#endif
    void Log_Write_Guided_Position_Target(ModeGuided::SubMode submode, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target);
    void Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f& ang_vel, float thrust, float climb_rate);
    void Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out);
    void Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z);
    void Log_Write_Vehicle_Startup_Messages();
    void log_init(void);

    // mode.cpp
    bool       set_mode(Mode::Number mode, ModeReason reason);
    bool       set_mode(const uint8_t new_mode, const ModeReason reason) override;
    ModeReason _last_reason;
    // called when an attempt to change into a mode is unsuccessful:
    void    mode_change_failed(const Mode* mode, const char* reason);
    uint8_t get_mode() const override { return (uint8_t)flightmode->mode_number(); }
    bool    current_mode_requires_mission() const override;
    void    update_flight_mode();
    void    notify_flight_mode();

    // Check if this mode can be entered from the GCS
    bool gcs_mode_enabled(const Mode::Number mode_num);

    // mode_land.cpp
    void set_mode_land_with_pause(ModeReason reason);
    bool landing_with_GPS();

    // motor_test.cpp
    void       motor_test_output();
    bool       mavlink_motor_control_check(const GCS_MAVLINK& gcs_chan, bool check_rc, const char* mode);
    MAV_RESULT mavlink_motor_test_start(const GCS_MAVLINK& gcs_chan, uint8_t motor_seq, uint8_t throttle_type, float throttle_value, float timeout_sec, uint8_t motor_count);
    void       motor_test_stop();

    // motors.cpp
    void arm_motors_check();
    void auto_disarm_check();
    void motors_output();
    void lost_vehicle_check();

    // navigation.cpp
    void     run_nav_updates(void);
    int32_t  home_bearing();
    uint32_t home_distance();

    // Parameters.cpp
    void load_parameters(void) override;
    void convert_pid_parameters(void);
#if HAL_PROXIMITY_ENABLED
    void convert_prx_parameters();
#endif
    void convert_lgr_parameters(void);
    void convert_tradheli_parameters(void) const;

    // precision_landing.cpp
    void init_precland();
    void update_precland();

    // radio.cpp
    void    default_dead_zones();
    void    init_rc_in();
    void    init_rc_out();
    void    read_radio();
    void    set_throttle_and_failsafe(uint16_t throttle_pwm);
    void    set_throttle_zero_flag(int16_t throttle_control);
    void    radio_passthrough_to_motors();
    int16_t get_throttle_mid(void);

    // sensors.cpp
    void read_barometer(void);
    void init_rangefinder(void);
    void read_rangefinder(void);
    bool rangefinder_alt_ok() const;
    bool rangefinder_up_ok() const;
    void update_rangefinder_terrain_offset();
    void update_optical_flow(void);

    // takeoff_check.cpp
    void takeoff_check();

    // RC_Channel.cpp
    void save_trim();
    void auto_trim();
    void auto_trim_cancel();

    // system.cpp
    void        init_ardupilot() override;
    void        startup_INS_ground();
    bool        position_ok() const;
    bool        ekf_has_absolute_position() const;
    bool        ekf_has_relative_position() const;
    bool        ekf_alt_ok() const;
    void        update_auto_armed();
    bool        should_log(uint32_t mask);
    const char* get_frame_string() const;
    void        allocate_motors(void);
    bool        is_tradheli() const;

    // terrain.cpp
    void terrain_update();
    void terrain_logging();

    // tuning.cpp
    void tuning();

    // UserCode.cpp
    void userhook_init();
    void userhook_FastLoop();
    void userhook_50Hz();
    void userhook_MediumLoop();
    void userhook_SlowLoop();
    void userhook_SuperSlowLoop();
    void userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag);
    void userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag);
    void userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag);

#if MODE_ACRO_ENABLED == ENABLED
# if FRAME_CONFIG == HELI_FRAME
    ModeAcro_Heli mode_acro;
# else
    ModeAcro mode_acro;
# endif
#endif
    ModeAltHold mode_althold;
#if MODE_AUTO_ENABLED == ENABLED
    ModeAuto mode_auto;
#endif
#if AUTOTUNE_ENABLED == ENABLED
    ModeAutoTune mode_autotune;
#endif
#if MODE_BRAKE_ENABLED == ENABLED
    ModeBrake mode_brake;
#endif
#if MODE_CIRCLE_ENABLED == ENABLED
    ModeCircle mode_circle;
#endif
#if MODE_DRIFT_ENABLED == ENABLED
    ModeDrift mode_drift;
#endif
#if MODE_FLIP_ENABLED == ENABLED
    ModeFlip mode_flip;
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    ModeFollow mode_follow;
#endif
#if MODE_GUIDED_ENABLED == ENABLED
    ModeGuided mode_guided;
#endif
    ModeLand mode_land;
#if MODE_LOITER_ENABLED == ENABLED
    ModeLoiter mode_loiter;
#endif
#if MODE_POSHOLD_ENABLED == ENABLED
    ModePosHold mode_poshold;
#endif
#if MODE_RTL_ENABLED == ENABLED
    ModeRTL mode_rtl;
#endif
#if FRAME_CONFIG == HELI_FRAME
    ModeStabilize_Heli mode_stabilize;
#else
    ModeStabilize mode_stabilize;
#endif
#if MODE_SPORT_ENABLED == ENABLED
    ModeSport mode_sport;
#endif
#if MODE_SYSTEMID_ENABLED == ENABLED
    ModeSystemId mode_systemid;
#endif
#if HAL_ADSB_ENABLED
    ModeAvoidADSB mode_avoid_adsb;
#endif
#if MODE_THROW_ENABLED == ENABLED
    ModeThrow mode_throw;
#endif
#if MODE_GUIDED_NOGPS_ENABLED == ENABLED
    ModeGuidedNoGPS mode_guided_nogps;
#endif
#if MODE_SMARTRTL_ENABLED == ENABLED
    ModeSmartRTL mode_smartrtl;
#endif
#if MODE_FLOWHOLD_ENABLED == ENABLED
    ModeFlowHold mode_flowhold;
#endif
#if MODE_ZIGZAG_ENABLED == ENABLED
    ModeZigZag mode_zigzag;
#endif
#if MODE_AUTOROTATE_ENABLED == ENABLED
    ModeAutorotate mode_autorotate;
#endif
#if MODE_TURTLE_ENABLED == ENABLED
    ModeTurtle mode_turtle;
#endif

    // mode.cpp
    Mode* mode_from_mode_num(const Mode::Number mode);
    void  exit_mode(Mode*& old_flightmode, Mode*& new_flightmode);

public:
    void failsafe_check(); // failsafe.cpp
};

extern Copter copter;

using AP_HAL::micros;
using AP_HAL::millis;
