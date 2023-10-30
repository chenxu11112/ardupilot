#include "Copter.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

/*****************************************************************************
 *   The init_ardupilot function processes everything we need for an in - air restart
 *        We will determine later if we are actually on the ground and process a
 *        ground start in that case.
 *
 *****************************************************************************/

/**
 * @brief 静态函数用于执行故障安全检查。
 *
 * 该函数负责执行故障安全检查，以确保飞行器的操作安全性。
 */
static void failsafe_check_static()
{
    copter.failsafe_check();
}

/**
 * @brief 初始化 ArduPilot 飞行控制系统
 *
 * 这个函数执行了飞行控制系统的初始化过程。以下是初始化的一些重要步骤：
 *
 * - 启用统计模块
 * - 初始化板载配置
 * - 初始化 CAN 总线管理器（如果已启用）
 * - 初始化货物夹具（如果已启用）
 * - 初始化绞盘（如果已启用）
 * - 初始化通知系统
 * - 初始化电池监控
 * - 初始化 RSSI（接收信号强度指示）
 * - 初始化气压计
 * - 设置串口通信（GCS）
 * - 初始化 OSD（显示屏信息，如果已启用）
 * - 初始化日志系统
 * - 更新电机互锁状态
 * - 初始化直升机特定配置（如果框架配置为直升机）
 * - 初始化遥控输入管理器
 * - 初始化地面或天花板追踪
 * - 分配电机类
 * - 初始化遥控通道，包括设置飞行模式
 * - 设置电机并输出到电调（电子速度控制器）
 * - 检查是否应进入电调校准模式
 * - 标记电机初始化参数已完成
 * - 启用主循环死亡检查
 * - 执行 GPS 初始化
 * - 执行指南针初始化
 * - 初始化空速传感器（如果已启用）
 * - 初始化障碍物避障路径规划（如果已启用）
 * - 执行姿态控制参数检查
 * - 初始化光流传感器（如果已启用）
 * - 初始化相机云台（如果已启用）
 * - 初始化相机（如果已启用）
 * - 初始化精确着陆系统（如果已启用）
 * - 初始化起落架位置（如果已启用）
 * - 执行用户自定义初始化钩子
 * - 读取大气压数据以进行高度校准
 * - 初始化超声波传感器（如果已启用）
 * - 初始化近程传感器（如果已启用）
 * - 初始化信标（用于非 GPS 位置估计）
 * - 初始化 RPM 传感器（如果已启用）
 * - 初始化自动飞行模式（如果已启用）
 * - 初始化 SmartRTL 模式（如果已启用）
 * - 初始化日志记录器
 * - 初始化 INS（惯性导航系统）状态为地面状态
 * - 初始化用户脚本引擎（如果已启用）
 * - 设置已降落标志
 * - 启用 CPU 失控保护
 * - 设置原始 IMU 数据记录标志
 * - 输出最低可能的电机值
 * - 尝试设置初始飞行模式，否则设置为 STABILIZE 模式
 * - 标记初始化已完成
 */
void Copter::init_ardupilot()
{

#if STATS_ENABLED == ENABLED
    // 启用统计模块
    g2.stats.init();
#endif

    // 初始化板载配置
    BoardConfig.init();

    // 如果启用了 CAN 总线协议驱动，初始化 CAN 总线管理器
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // 如果已启用货物夹具，执行初始化
#if AP_GRIPPER_ENABLED
    g2.gripper.init();
#endif

    // 如果已启用绞盘，执行初始化
#if AP_WINCH_ENABLED
    g2.winch.init();
#endif

    // 初始化通知系统
    notify.init();
    notify_flight_mode();

    // 初始化电池监控
    battery.init();

    // 初始化 RSSI
    rssi.init();

    // 初始化气压计
    barometer.init();

    // 设置串口通信（GCS）
    gcs().setup_uarts();

    // 如果启用 OSD，初始化 OSD
#if OSD_ENABLED == ENABLED
    osd.init();
#endif

    // 如果启用日志，初始化日志系统
#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // 更新电机互锁状态
    update_using_interlock();

    // 如果框架配置为直升机，执行直升机特定初始化
#if FRAME_CONFIG == HELI_FRAME
    heli_init();
#endif

    // 如果框架配置为直升机，设置输入管理器的循环速率
#if FRAME_CONFIG == HELI_FRAME
    input_manager.set_loop_rate(scheduler.get_loop_rate_hz());
#endif

    // 设置遥控输入，包括从无线电设置遥控通道
    init_rc_in();

    // 初始化用于地面或天花板追踪的表面
    // 必须在初始化遥控器通道之前执行以不覆盖初始开关位置
    surface_tracking.init((SurfaceTracking::Surface)copter.g2.surftrak_mode.get());

    // 分配电机类
    allocate_motors();

    // 初始化遥控通道，包括设置飞行模式
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE);
    rc().init();

    // 设置电机并输出到电调
    init_rc_out();

    // 检查是否应进入电调校准模式
    esc_calibration_startup_check();

    // 电机初始化完成，可以发送参数
    ap.initialised_params = true;

    // 如果启用了继电器，初始化继电器
#if AP_RELAY_ENABLED
    relay.init();
#endif

    // 设置“主循环死亡”检查，依赖于 RC 库的初始化
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // 执行 GPS 初始化
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    // 初始化 SAWS 协议
    ap_saw.init(serial_manager);

    // 初始化指南针（磁罗盘）
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // 如果启用了空速传感器，设置空速传感器的日志位
#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

    // 如果启用了避障路径规划，执行初始化
#if AC_OAPATHPLANNER_ENABLED == ENABLED
    g2.oa.init();
#endif

    // 执行姿态控制参数检查
    attitude_control->parameter_sanity_check();

    // 如果启用了光流传感器，执行光流传感器初始化
#if AP_OPTICALFLOW_ENABLED
    optflow.init(MASK_LOG_OPTFLOW);
#endif

    // 如果启用了相机云台，初始化相机云台
#if HAL_MOUNT_ENABLED
    camera_mount.init();
#endif

    // 如果启用了相机，初始化相机
#if AP_CAMERA_ENABLED
    camera.init();
#endif

    // 如果启用了精确着陆系统，执行初始化
#if AC_PRECLAND_ENABLED
    init_precland();
#endif

    // 如果启用了起落架，初始化起落架位置
#if AP_LANDINGGEAR_ENABLED
    landinggear.init();
#endif

    // 用户自定义初始化钩子
#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // 读取大气压数据以进行高度校准
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // 如果启用了超声波传感器，执行初始化
#if RANGEFINDER_ENABLED == ENABLED
    init_rangefinder();
#endif

    // 如果启用了近程传感器，初始化近程传感器
#if HAL_PROXIMITY_ENABLED
    g2.proximity.init();
#endif

    // 如果启用了信标，初始化信标
#if AP_BEACON_ENABLED
    g2.beacon.init();
#endif

    // 如果启用了 RPM 传感器，初始化 RPM 传感器
#if AP_RPM_ENABLED
    rpm_sensor.init();
#endif

    // 如果启用了自动飞行模式，执行初始化
#if MODE_AUTO_ENABLED == ENABLED
    mode_auto.mission.init();
#endif

    // 如果启用了 SmartRTL 模式，执行初始化
#if MODE_SMARTRTL_ENABLED == ENABLED
    g2.smart_rtl.init();
#endif

    // 初始化AP_Logger库
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&copter, &Copter::Log_Write_Vehicle_Startup_Messages, void));

    // 启动惯性导航系统
    startup_INS_ground();

    // 初始化用户脚本引擎（如果已启用）
#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

    // 初始化自定义飞行控制器（如果已启用）
#if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
    custom_control.init();
#endif

    // 设置已降落标志
    set_land_complete(true);
    set_land_complete_maybe(true);

    // 启用CPU失控保护
    failsafe_enable();

    // 记录原始IMU数据
    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // 输出电机的最小可能值
    motors->output_min();

    // 尝试设置初始模式，否则设置为STABILIZE模式
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        set_mode(Mode::Number::STABILIZE, ModeReason::UNAVAILABLE);
    }

    // 标志初始化已经完成
    ap.initialised = true;
}

/**
 * @brief 执行地面起飞时所需的所有校准等操作
 *
 * 这个函数执行了地面起飞时的一系列操作，包括初始化AHRS（姿态和方向传感器融合系统）、校准陀螺仪偏移、
 * 以及重置AHRS。地面起飞时，这些操作有助于确保飞行控制系统处于正确的状态。
 *
 */
void Copter::startup_INS_ground()
{
    // 初始化AHRS，设置飞行器类型为COPTER
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // 预热和校准陀螺仪偏移
    ins.init(scheduler.get_loop_rate_hz());

    // 重置AHRS，包括陀螺仪偏移
    ahrs.reset();
}

/**
 * @brief 检查水平绝对位置是否可用并且家点位置已设置
 *
 * 该函数用于检查水平绝对位置是否可用并且家点位置是否已设置。它首先检查EKF（扩展卡尔曼滤波器）的失败保护，
 * 如果EKF失败保护触发，则返回`false`。然后它检查EKF的绝对位置估计和相对位置估计是否可用，如果其中之一可用，
 * 则返回`true`，否则返回`false`。
 *
 * @return 如果水平绝对位置可用并且家点位置已设置，返回`true`；否则返回`false`。
 */
bool Copter::position_ok() const
{
    // 如果EKF失败保护已触发，则返回false
    if (failsafe.ekf) {
        return false;
    }

    // 检查EKF的绝对位置估计或相对位置估计是否可用
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

/**
 * @brief 检查EKF是否能提供绝对的WGS-84位置估计
 *
 * 该函数用于检查EKF（扩展卡尔曼滤波器）是否能够提供绝对的WGS-84位置估计。首先它检查是否有惯性导航信息，如果没有，
 * 则不允许使用DCM（方向余弦矩阵）位置，然后它使用EKF的过滤器状态和EKF检查来判断是否可以提供绝对位置估计。
 * 当未解锁（disarmed）时，允许接受预测的水平位置，但一旦解锁（armed），则要求绝对位置估计是有效的，并且EKF不能处于const_pos_mode。
 *
 * @return 如果EKF能够提供绝对的WGS-84位置估计，返回`true`；否则返回`false`。
 */
bool Copter::ekf_has_absolute_position() const
{
    if (!ahrs.have_inertial_nav()) {
        // 不允许使用DCM位置信息
        return false;
    }

    // 使用EKF的过滤器状态和EKF检查
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // 如果未解锁，接受预测的水平位置
    if (!motors->armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // 一旦解锁，要求绝对位置估计有效，且EKF不能处于const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

/**
 * @brief 检查EKF是否能提供相对于其起始位置的位置估计
 *
 * 该函数用于检查EKF（扩展卡尔曼滤波器）是否能够提供相对于其起始位置的位置估计。它首先检查是否使用惯性导航信息（inertial navigation）。
 * 如果未使用，立即返回`false`。接着，它检查光流（optical flow）、视觉测距（visual odometry）和惯性导航的状态，
 * 如果它们中的任何一个处于启用状态且惯性导航的惯性导航是活跃的（即未超时），则返回`true`。
 * 当未解锁（disarmed）时，允许接受预测的水平相对位置，但一旦解锁（armed），则要求相对位置估计是有效的，并且EKF不能处于const_pos_mode。
 *
 * @return 如果EKF能够提供相对于其起始位置的位置估计，返回`true`；否则返回`false`。
 */
bool Copter::ekf_has_relative_position() const
{
    // 如果EKF没有使用，立即返回false
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // 立即返回false，如果光流和视觉测距都未启用且惯性导航处于非活跃状态
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (dead_reckoning.active && !dead_reckoning.timeout) {
        enabled = true;
    }
    if (!enabled) {
        return false;
    }

    // 从EKF获取过滤器状态
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // 如果未解锁，接受预测的水平相对位置
    if (!motors->armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
}

/**
 * @brief 检查EKF是否具有良好的高度估计（对于需要进行高度保持的模式）
 *
 * 该函数用于检查EKF（扩展卡尔曼滤波器）是否具有良好的高度估计，以满足需要进行高度保持的模式的要求。它首先检查是否使用惯性导航信息（inertial navigation）。
 * 如果未使用，立即返回`false`。接着，它检查EKF的过滤器状态，要求垂直速度和垂直位置都可用时才返回`true`。
 *
 * @return 如果EKF具有良好的高度估计，返回`true`；否则返回`false`。
 */
bool Copter::ekf_alt_ok() const
{
    // 如果EKF没有使用，立即返回false
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // 使用EKF的过滤器状态和EKF检查
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // 需要垂直速度和垂直位置都可用时才返回true
    return (filt_status.flags.vert_vel && filt_status.flags.vert_pos);
}

/**
 * @brief 更新 `auto_armed` 标志的状态，该标志指示飞行器是否已解锁（"armed"）。
 *
 * 这个函数根据不同情况执行解锁或锁定飞行器。解锁通常是在飞行前进行的，它允许电机旋转以启动
 * 飞行器。该函数检查多个条件，以确保 `auto_armed` 标志与飞行器的状态一致。
 * 如果飞行器已解锁，`auto_armed` 为真，否则为假。
 */
void Copter::update_auto_armed()
{
    // 进行解锁状态检查
    if (ap.auto_armed) {
        // 如果电机已锁定，`auto_armed` 也应该是假的
        if (!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // 如果处于稳定（stabilize）或空翻（acro）飞行模式且油门为零，`auto_armed` 也应该是假的
        if (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
    } else {
        // 进行锁定状态检查

        // 对于传统直升机，如果电机已解锁、油门大于零且电机已启动，则`auto_armed` 应为真
        if (motors->armed() && ap.using_interlock) {
            if (!ap.throttle_zero && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                set_auto_armed(true);
            }
            // 如果电机已解锁且油门大于零，`auto_armed` 应为真
            // 如果电机已解锁且飞行器处于抛掷（throw）模式，则`auto_armed` 应为真
        } else if (motors->armed() && !ap.using_interlock) {
            if (!ap.throttle_zero || flightmode->mode_number() == Mode::Number::THROW) {
                set_auto_armed(true);
            }
        }
    }
}

/**
 * @brief 判断是否应该记录特定类型的消息。
 *
 * 这个函数用于确定是否应记录指定类型的消息。它会检查日志记录是否已启动以及是否应记录特定的消息类型。
 * 如果日志记录未启动或不需要记录指定类型的消息，函数将返回false，否则返回true。
 *
 * @param mask 用于指定消息类型的掩码。可以是一组不同消息类型的按位或（|）结果。
 * @return 如果应记录指定类型的消息，则返回true；否则返回false。
 */
bool Copter::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    // 获取日志记录是否已启动的状态
    ap.logging_started = logger.logging_started();
    // 检查是否应记录指定类型的消息
    return logger.should_log(mask);
#else
    return false; // 如果日志记录未启用，不需要记录任何消息
#endif
}

/**
 * @brief 分配并初始化飞行器的电机控制对象
 *
 * 此函数负责分配并初始化适用于不同飞行器框架的电机控制对象，以确保正确控制电机并实现飞行控制。
 *
 * @note 这个函数还会加载相应的参数和执行参数升级操作，以确保参数的一致性和正确性。
 */
void Copter::allocate_motors(void)
{
    // 根据飞行器框架选择适当的电机控制对象
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
#if FRAME_CONFIG != HELI_FRAME
        // 对于一些框架，例如四旋翼、六旋翼等
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
        case AP_Motors::MOTOR_FRAME_DECA:
        case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX:
        default:
            motors          = new AP_MotorsMatrix(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        // 对于三旋翼
        case AP_Motors::MOTOR_FRAME_TRI:
            motors          = new AP_MotorsTri(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTri::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
        // 对于单旋翼
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors          = new AP_MotorsSingle(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        // 对于双旋翼
        case AP_Motors::MOTOR_FRAME_COAX:
            motors          = new AP_MotorsCoax(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        // 对于垂直起降飞行器（tailsitter）
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors          = new AP_MotorsTailsitter(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
        // 对于六自由度脚本控制矩阵
        case AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING:
# if AP_SCRIPTING_ENABLED
            motors          = new AP_MotorsMatrix_6DoF_Scripting(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_6DoF_Scripting::var_info;
# endif // AP_SCRIPTING_ENABLED
            break;
        // 对于动态脚本控制矩阵
        case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
# if AP_SCRIPTING_ENABLED
            motors          = new AP_MotorsMatrix_Scripting_Dynamic(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
# endif // AP_SCRIPTING_ENABLED
            break;
#else // FRAME_CONFIG == HELI_FRAME
      // 对于传统直升机
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors          = new AP_MotorsHeli_Dual(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            motors          = new AP_MotorsHeli_Quad(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Quad::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors          = new AP_MotorsHeli_Single(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
#endif
    }
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo* ac_var_info;

#if FRAME_CONFIG != HELI_FRAME
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() == AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING) {
# if AP_SCRIPTING_ENABLED
        attitude_control = new AC_AttitudeControl_Multi_6DoF(*ahrs_view, aparm, *motors);
        ac_var_info      = AC_AttitudeControl_Multi_6DoF::var_info;
# endif // AP_SCRIPTING_ENABLED
    } else {
        attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors);
        ac_var_info      = AC_AttitudeControl_Multi::var_info;
    }
#else
    attitude_control = new AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors);
    ac_var_info      = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_BoardConfig::allocation_error("AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);

    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    if (pos_control == nullptr) {
        AP_BoardConfig::allocation_error("PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    wp_nav = new AC_WPNav_OA(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#else
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#endif
    if (wp_nav == nullptr) {
        AP_BoardConfig::allocation_error("WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_BoardConfig::allocation_error("LoiterNav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

#if MODE_CIRCLE_ENABLED == ENABLED
    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_BoardConfig::allocation_error("CircleNav");
    }
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);
#endif

    // 从默认文件中重新加载参数行，以确保可以访问
    AP_Param::reload_defaults_file(true);

    // 现在设置一些特定于框架类别的默认值
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_Y6:
            attitude_control->get_rate_roll_pid().kP().set_default(0.1);
            attitude_control->get_rate_roll_pid().kD().set_default(0.006);
            attitude_control->get_rate_pitch_pid().kP().set_default(0.1);
            attitude_control->get_rate_pitch_pid().kD().set_default(0.006);
            attitude_control->get_rate_yaw_pid().kP().set_default(0.15);
            attitude_control->get_rate_yaw_pid().kI().set_default(0.015);
            break;
        case AP_Motors::MOTOR_FRAME_TRI:
            attitude_control->get_rate_yaw_pid().filt_D_hz().set_default(100);
            break;
        default:
            break;
    }

    // 如果电机使用的是刷式PWM类型，将默认的RC速度设为16kHz脉冲
    if (motors->is_brushed_pwm_type()) {
        g.rc_speed.set_default(16000);
    }

    // 升级参数。这必须在分配对象之后执行
    convert_pid_parameters();
#if FRAME_CONFIG == HELI_FRAME
    convert_tradheli_parameters();
#endif

#if HAL_PROXIMITY_ENABLED
    // 将PRX参数升级为PRX1_参数
    convert_prx_parameters();
#endif

    // 参数计数可能已更改
    AP_Param::invalidate_count();
}

bool Copter::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
