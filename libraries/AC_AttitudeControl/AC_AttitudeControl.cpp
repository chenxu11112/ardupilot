#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
// 针对ArduPlane的默认参数
# define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT 0.2f // 输入平滑时间常数默认值（较小值，适用于飞机）
# define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN  5.0  // 最小倾斜角度，以确保飞行器能够维持有限的控制
#else
// 针对Copter和Sub的默认参数
# define AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT 0.15f // 输入平滑时间常数默认值（中等值，适用于直升机和潜艇）
# define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN  10.0  // 最小倾斜角度，以确保飞行器能够维持有限的控制
#endif

AC_AttitudeControl* AC_AttitudeControl::_singleton;

// 表示用户可设置参数的参数表
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 用于RATE_RP_MAX和RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw目标斜率
    // @Description: 在Loiter、RTL、Auto飞行模式下，允许的最大偏航目标斜率
    // @Units: 厘度/秒（centidegrees per second）
    // @Range: 500 18000
    // @Increment: 100
    // @User: 高级用户
    AP_GROUPINFO("SLEW_YAW", 2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 用于ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: 偏航轴最大加速度
    // @Description: 偏航轴最大加速度
    // @Units: 厘度/秒²（centidegrees per second squared）
    // @Range: 0 72000
    // @Values: 0:禁用, 9000:非常慢, 18000:慢, 36000:中等, 54000:快
    // @Increment: 1000
    // @User: 高级用户
    AP_GROUPINFO("ACCEL_Y_MAX", 4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: 角速度前馈启用
    // @Description: 控制是否启用或禁用机体坐标系的角速度前馈
    // @Values: 0:禁用, 1:启用
    // @User: 高级用户
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: 横滚轴最大加速度
    // @Description: 横滚轴最大加速度
    // @Units: 厘度/秒²（centidegrees per second squared）
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:禁用, 30000:非常慢, 72000:慢, 108000:中等, 162000:快
    // @User: 高级用户
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: 俯仰轴最大加速度
    // @Description: 俯仰轴最大加速度
    // @Units: 厘度/秒²（centidegrees per second squared）
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:禁用, 30000:非常慢, 72000:慢, 108000:中等, 162000:快
    // @User: 高级用户
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // ID 8,9,10,11 预留（在Solo上使用）

    // @Param: ANGLE_BOOST
    // @DisplayName: 倾斜补偿
    // @Description: 当飞行器倾斜以减小高度损失时，倾斜补偿会增加油门输出
    // @Values: 0:禁用, 1:启用
    // @User: 高级用户
    AP_GROUPINFO("ANGLE_BOOST", 12, AC_AttitudeControl, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: 横滚轴角度控制P增益
    // @Description: 横滚轴角度控制P增益。将期望横滚角度与实际角度之间的误差转换为期望横滚速度
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: 标准用户
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: 俯仰轴角度控制P增益
    // @Description: 俯仰轴角度控制P增益。将期望俯仰角度与实际角度之间的误差转换为期望俯仰速度
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: 标准用户
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: 偏航轴角度控制P增益
    // @Description: 偏航轴角度控制P增益。将期望偏航角度与实际角度之间的误差转换为期望偏航速度
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 6.000
    // @User: 标准用户
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, AC_AttitudeControl, AC_P),

    // @Param: ANG_LIM_TC
    // @DisplayName: 角度限制（以维持高度）的时间常数
    // @Description: 角度限制（以维持高度）的时间常数
    // @Range: 0.5 10.0
    // @User: 高级用户
    AP_GROUPINFO("ANG_LIM_TC", 16, AC_AttitudeControl, _angle_limit_tc, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT),

    // @Param: RATE_R_MAX
    // @DisplayName: 横滚轴最大角速度
    // @Description: 横滚轴的最大角速度
    // @Units: 度/秒（degrees per second）
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:禁用, 60:慢, 180:中等, 360:快
    // @User: 高级用户
    AP_GROUPINFO("RATE_R_MAX", 17, AC_AttitudeControl, _ang_vel_roll_max, 0.0f),

    // @Param: RATE_P_MAX
    // @DisplayName: 俯仰轴最大角速度
    // @Description: 俯仰轴的最大角速度
    // @Units: 度/秒（degrees per second）
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:禁用, 60:慢, 180:中等, 360:快
    // @User: 高级用户
    AP_GROUPINFO("RATE_P_MAX", 18, AC_AttitudeControl, _ang_vel_pitch_max, 0.0f),

    // @Param: RATE_Y_MAX
    // @DisplayName: 偏航轴最大角速度
    // @Description: 偏航轴的最大角速度
    // @Units: 度/秒（degrees per second）
    // @Range: 0 1080
    // @Increment: 1
    // @Values: 0:禁用, 60:慢, 180:中等, 360:快
    // @User: 高级用户
    AP_GROUPINFO("RATE_Y_MAX", 19, AC_AttitudeControl, _ang_vel_yaw_max, 0.0f),

    // @Param: INPUT_TC
    // @DisplayName: 姿态控制输入时间常数
    // @Description: 姿态控制输入时间常数。较小的数字会导致更快的响应，较大的数字会导致更平滑的响应
    // @Units: 秒（seconds）
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:非常平滑, 0.2:平滑, 0.15:中等, 0.1:灵敏, 0.05:非常灵敏
    // @User: 标准用户
    AP_GROUPINFO("INPUT_TC", 20, AC_AttitudeControl, _input_tc, AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT),

    AP_GROUPEND
};

constexpr Vector3f AC_AttitudeControl::VECTORF_111;
// 获取偏航速率限制（以度/秒为单位）
float AC_AttitudeControl::get_slew_yaw_max_degs() const
{
    // 如果偏航最大角速度不是正数，则返回偏航目标斜率的1%作为偏航速率限制
    if (!is_positive(_ang_vel_yaw_max)) {
        return _slew_yaw * 0.01;
    }
    // 否则，返回偏航最大角速度和偏航目标斜率的1%中较小的一个
    return MIN(_ang_vel_yaw_max, _slew_yaw * 0.01);
}

// 确保姿态控制器没有误差以放松角速度控制器的输出
void AC_AttitudeControl::relax_attitude_controllers()
{
    // 将姿态变量初始化为当前姿态
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target);
    _attitude_ang_error.initialise();

    // 将角速度变量初始化为当前速率
    _ang_vel_target = _ahrs.get_gyro();
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
    _ang_vel_body = _ahrs.get_gyro();

    // 初始化其余变量
    _thrust_error_angle = 0.0f;

    // 重置PID过滤器
    get_rate_roll_pid().reset_filter();
    get_rate_pitch_pid().reset_filter();
    get_rate_yaw_pid().reset_filter();

    // 重置I项（积分项）
    reset_rate_controller_I_terms();
}

// 重置角速度控制器的I项（积分项）
void AC_AttitudeControl::reset_rate_controller_I_terms()
{
    // 分别重置横滚、俯仰和偏航角速度控制器的I项
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();
}

// 平滑地将角速度控制器的I项重置为零，需要0.5秒的时间
void AC_AttitudeControl::reset_rate_controller_I_terms_smoothly()
{
    // 平滑地将横滚、俯仰和偏航角速度控制器的I项重置为零，需要0.5秒的时间
    get_rate_roll_pid().relax_integrator(0.0, _dt, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_pitch_pid().relax_integrator(0.0, _dt, AC_ATTITUDE_RATE_RELAX_TC);
    get_rate_yaw_pid().relax_integrator(0.0, _dt, AC_ATTITUDE_RATE_RELAX_TC);
}

// 姿态控制器用于调整飞行器的姿态，它的核心思想是维持期望姿态、目标姿态和测量姿态之间的一致性。
// 期望姿态（attitude_desired_quat）是由上层控制器确定的目标姿态，表示我们希望飞行器到达的姿态。
// 目标姿态（_attitude_target）通过施加角加速度和角速度限制来逐渐调整，以逼近期望姿态。
// 目标角速度（_ang_vel_target）直接输入到角速度控制器，以控制飞行器的角速度。
// 姿态误差（attitude_error_quat）是测量姿态与目标姿态之间的差异，这个误差传递给角度控制器。

// 所有的输入函数都遵循相同的流程：
// 1. 根据输入变量，定义飞行器应该达到的期望姿态。
// 2. 使用期望姿态和输入变量，定义目标角速度，以逐渐将目标姿态移动到期望姿态。
// 3. 如果未启用 _rate_bf_ff_enabled，则将目标姿态和目标角速度设置为期望姿态和期望角速度。
// 4. 确保已经定义 _attitude_target、_euler_angle_target、_euler_rate_target 和 _ang_vel_target。
//    这确保可以更改输入模式而不会出现不连续性。
// 5. 调用 attitude_controller_run_quat 函数，将目标角速度传递到角速度控制器并将其整合到目标姿态中。
//    任何目标姿态和测量姿态之间的误差首先通过校正推力矢量来进行校正，直到目标推力矢量与测量推力矢量之间的夹角小于 2*AC_ATTITUDE_THRUST_ERROR_ANGLE。
//    此时还会校正航向。

// 使用四元数姿态命令，包括前馈和平滑
// attitude_desired_quat：每个时间步长都通过角速度积分来更新，表示期望的姿态
void AC_AttitudeControl::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target)
{
    // 计算姿态误差四元数
    Quaternion attitude_error_quat = _attitude_target.inverse() * attitude_desired_quat;
    Vector3f   attitude_error_angle;
    attitude_error_quat.to_axis_angle(attitude_error_angle);

    // 限制角速度
    ang_vel_limit(ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

    if (_rate_bf_ff_enabled) {
        // 当启用加速度限制和前馈时，使用平方根控制器计算一个欧拉角速度，
        // 该角速度将导致欧拉角平滑停止在输入角度处，其减速受到输入_tc指定的限制，
        // 并在最后由_dt确定的指数衰减。
        _ang_vel_target.x = input_shaping_angle(wrap_PI(attitude_error_angle.x), _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, ang_vel_target.x, radians(_ang_vel_roll_max), _dt);
        _ang_vel_target.y = input_shaping_angle(wrap_PI(attitude_error_angle.y), _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, ang_vel_target.y, radians(_ang_vel_pitch_max), _dt);
        _ang_vel_target.z = input_shaping_angle(wrap_PI(attitude_error_angle.z), _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z, ang_vel_target.z, radians(_ang_vel_yaw_max), _dt);
    } else {
        _attitude_target = attitude_desired_quat;
        _ang_vel_target  = ang_vel_target;
    }

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 将机体坐标系的角速度转换为期望姿态的欧拉角速度导数
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

    // 旋转目标并归一化
    Quaternion attitude_desired_update;
    attitude_desired_update.from_axis_angle(ang_vel_target * _dt);
    attitude_desired_quat = attitude_desired_quat * attitude_desired_update;
    attitude_desired_quat.normalize();

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 通过输入欧拉角（滚转和俯仰）和欧拉角速率（偏航速率）来控制飞行器的姿态，同时使用角速度前馈和平滑处理
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // 将公共接口上的角度从厘度转换为弧度
    float euler_roll_angle  = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_rate    = radians(euler_yaw_rate_cds * 0.01f);

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 添加滚转修正，以补偿直升机中的尾桨推力（在多旋翼飞行器上返回零）
    euler_roll_angle += get_roll_trim_rad();

    if (_rate_bf_ff_enabled) {
        // 将欧拉轴上的角加速度限制转化为欧拉轴
        const Vector3f euler_accel = euler_accel_limit(_euler_angle_target, Vector3f { get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss() });

        // 当启用加速度限制和前馈时，使用平方根控制器计算一个欧拉角速度，
        // 该角速度将导致欧拉角平滑停止在输入角度处，其减速受到输入_tc指定的限制。
        _euler_rate_target.x = input_shaping_angle(wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x, _euler_rate_target.x, _dt);
        _euler_rate_target.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y, _euler_rate_target.y, _dt);

        // 当启用偏航角加速度限制时，偏航输入整形器会限制偏航角加速度，逐渐调整输出速率以趋近输入速率。
        _euler_rate_target.z = input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt, _rate_y_tc);

        // 将期望姿态的欧拉角速度导数转化为机体坐标系下的角速度前馈
        euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
        // 限制角速度
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // 将机体坐标系下的角速度转化为期望姿态的欧拉角速度导数
        ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
    } else {
        // 当未启用前馈时，将目标欧拉角直接输入目标，并将前馈速率归零。
        _euler_angle_target.x = euler_roll_angle;
        _euler_angle_target.y = euler_pitch_angle;
        _euler_angle_target.z += euler_yaw_rate * _dt;
        // 计算四元数目标姿态
        _attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

        // 将角速度前馈请求设为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 通过输入欧拉角（滚转、俯仰和偏航）来控制飞行器的姿态，同时使用角速度前馈和平滑处理
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // 将公共接口上的角度从厘度转换为弧度
    float euler_roll_angle  = radians(euler_roll_angle_cd * 0.01f);
    float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);
    float euler_yaw_angle   = radians(euler_yaw_angle_cd * 0.01f);

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 添加滚转修正，以补偿直升机中的尾桨推力（在多旋翼飞行器上返回零）
    euler_roll_angle += get_roll_trim_rad();

    const float slew_yaw_max_rads = get_slew_yaw_max_rads();
    if (_rate_bf_ff_enabled) {
        // 将欧拉轴上的角加速度限制转化为欧拉轴
        const Vector3f euler_accel = euler_accel_limit(_euler_angle_target, Vector3f { get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss() });

        // 当启用加速度限制和前馈时，使用平方根控制器计算一个欧拉角速度，
        // 该角速度将导致欧拉角平滑停止在输入角度处，其减速受到输入_tc指定的限制。
        _euler_rate_target.x = input_shaping_angle(wrap_PI(euler_roll_angle - _euler_angle_target.x), _input_tc, euler_accel.x, _euler_rate_target.x, _dt);
        _euler_rate_target.y = input_shaping_angle(wrap_PI(euler_pitch_angle - _euler_angle_target.y), _input_tc, euler_accel.y, _euler_rate_target.y, _dt);
        _euler_rate_target.z = input_shaping_angle(wrap_PI(euler_yaw_angle - _euler_angle_target.z), _input_tc, euler_accel.z, _euler_rate_target.z, _dt);
        if (slew_yaw) {
            // 如果启用偏航限制，将偏航速率限制在最大允许范围内
            _euler_rate_target.z = constrain_float(_euler_rate_target.z, -slew_yaw_max_rads, slew_yaw_max_rads);
        }

        // 将期望姿态的欧拉角速度导数转化为机体坐标系下的角速度前馈
        euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
        // 限制角速度
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
        // 将机体坐标系下的角速度转化为期望姿态的欧拉角速度导数
        ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
    } else {
        // 当未启用前馈时，将目标欧拉角直接输入目标，并将前馈速率归零。
        _euler_angle_target.x = euler_roll_angle;
        _euler_angle_target.y = euler_pitch_angle;
        if (slew_yaw) {
            // 计算受限制的角度误差
            float angle_error = constrain_float(wrap_PI(euler_yaw_angle - _euler_angle_target.z), -slew_yaw_max_rads * _dt, slew_yaw_max_rads * _dt);
            // 从受限制的角度误差更新姿态目标
            _euler_angle_target.z = wrap_PI(angle_error + _euler_angle_target.z);
        } else {
            _euler_angle_target.z = euler_yaw_angle;
        }
        // 计算四元数目标姿态
        _attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

        // 将角速度前馈请求设为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 通过输入欧拉角速度（滚转、俯仰和偏航速度）来控制飞行器的姿态，同时使用角速度前馈和平滑处理
void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
{
    // 将公共接口上的角速度从厘度转换为弧度
    float euler_roll_rate  = radians(euler_roll_rate_cds * 0.01f);
    float euler_pitch_rate = radians(euler_pitch_rate_cds * 0.01f);
    float euler_yaw_rate   = radians(euler_yaw_rate_cds * 0.01f);

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    if (_rate_bf_ff_enabled) {
        // 将欧拉轴上的角加速度限制转化为欧拉轴
        const Vector3f euler_accel = euler_accel_limit(_euler_angle_target, Vector3f { get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss() });

        // 当启用加速度限制时，输入整形器将限制角加速度，使输出速率朝着输入速率趋近。
        _euler_rate_target.x = input_shaping_ang_vel(_euler_rate_target.x, euler_roll_rate, euler_accel.x, _dt, _rate_rp_tc);
        _euler_rate_target.y = input_shaping_ang_vel(_euler_rate_target.y, euler_pitch_rate, euler_accel.y, _dt, _rate_rp_tc);
        _euler_rate_target.z = input_shaping_ang_vel(_euler_rate_target.z, euler_yaw_rate, euler_accel.z, _dt, _rate_y_tc);

        // 将期望姿态的欧拉角速度导数转化为机体坐标系下的角速度前馈
        euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
    } else {
        // 当未启用前馈时，直接将目标欧拉角输入目标，同时将前馈速率归零。
        // 俯仰角受限于+- 85.0 度，以避免云台锁定的不连续性。
        _euler_angle_target.x = wrap_PI(_euler_angle_target.x + euler_roll_rate * _dt);
        _euler_angle_target.y = constrain_float(_euler_angle_target.y + euler_pitch_rate * _dt, radians(-85.0f), radians(85.0f));
        _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + euler_yaw_rate * _dt);

        // 将角速度前馈请求设为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();

        // 计算四元数目标姿态
        _attitude_target.from_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);
    }

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 使用角速度（滚转、俯仰和偏航速度）控制飞行器，同时使用角速度前馈和平滑处理
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // 将公共接口上的角速度从厘度转换为弧度
    float roll_rate_rads  = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    if (_rate_bf_ff_enabled) {
        // 计算受限加速度的机体坐标系角速度
        // 当启用加速度限制时，输入整形器将限制绕轴的角加速度，将输出速度趋近于输入速度。
        _ang_vel_target.x = input_shaping_ang_vel(_ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
        _ang_vel_target.y = input_shaping_ang_vel(_ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
        _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

        // 将机体坐标系的角速度转化为期望姿态的欧拉角导数
        ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
    } else {
        // 当未启用前馈时，计算四元数，并将其输入到目标中，同时将前馈速率归零。
        Quaternion attitude_target_update;
        attitude_target_update.from_axis_angle(Vector3f { roll_rate_rads * _dt, pitch_rate_rads * _dt, yaw_rate_rads * _dt });
        _attitude_target = _attitude_target * attitude_target_update;
        _attitude_target.normalize();

        // 将角速度前馈请求设为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 使用仅使用速率回路而没有姿态回路稳定来控制飞行器的角速度，同时使用速率平滑
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // 将公共接口上的角速度从厘度转换为弧度
    float roll_rate_rads  = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // 计算受限加速度的机体坐标系角速度
    // 当启用加速度限制时，输入整形器将限制绕轴的角加速度，将输出速度趋近于输入速度。
    _ang_vel_target.x = input_shaping_ang_vel(_ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.y = input_shaping_ang_vel(_ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

    // 基于当前姿态更新未使用的目标姿态，以适应模式更改
    _ahrs.get_quat_body_to_ned(_attitude_target);
    _attitude_target.to_euler(_euler_angle_target);
    // 将机体坐标系的角速度转化为期望姿态的欧拉角导数
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);
    _ang_vel_body = _ang_vel_target;
}

// 使用速率回路进行角速度控制，同时使用速率回路积分来稳定姿态
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // 将公共接口上的角速度从厘度转换为弧度
    float roll_rate_rads  = radians(roll_rate_bf_cds * 0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds * 0.01f);
    float yaw_rate_rads   = radians(yaw_rate_bf_cds * 0.01f);

    // 更新姿态误差
    Vector3f attitude_error;
    _attitude_ang_error.to_axis_angle(attitude_error);

    Quaternion attitude_ang_error_update_quat;
    // 限制积分误差角度
    float err_mag = attitude_error.length();
    if (err_mag > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        attitude_error *= AC_ATTITUDE_THRUST_ERROR_ANGLE / err_mag;
        _attitude_ang_error.from_axis_angle(attitude_error);
    }

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    attitude_ang_error_update_quat.from_axis_angle(Vector3f { (_ang_vel_target.x - gyro_latest.x) * _dt, (_ang_vel_target.y - gyro_latest.y) * _dt, (_ang_vel_target.z - gyro_latest.z) * _dt });
    _attitude_ang_error = attitude_ang_error_update_quat * _attitude_ang_error;

    // 计算受限加速度的机体坐标系角速度
    // 当启用加速度限制时，输入整形器将限制绕轴的角加速度，将输出速度趋近于输入速度。
    _ang_vel_target.x = input_shaping_ang_vel(_ang_vel_target.x, roll_rate_rads, get_accel_roll_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.y = input_shaping_ang_vel(_ang_vel_target.y, pitch_rate_rads, get_accel_pitch_max_radss(), _dt, _rate_rp_tc);
    _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, yaw_rate_rads, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

    // 检索机体姿态的四元数
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // 基于当前姿态更新未使用的目标姿态，以适应模式更改
    _attitude_target = attitude_body * _attitude_ang_error;

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 将机体坐标系的角速度转化为期望姿态的欧拉角导数
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

    // 从积分速率误差计算角速度目标
    _attitude_ang_error.to_axis_angle(attitude_error);
    _ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);
    _ang_vel_body += _ang_vel_target;

    // 确保四元数保持标准化
    _attitude_ang_error.normalize();
}

// 命令机体坐标系角度的角度步进（即更改）
// 用于在自动调整期间命令一个角度的步进，而不会激发正交轴
void AC_AttitudeControl::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd)
{
    // 将公共接口上的角度步进从厘度转换为弧度
    float roll_step_rads  = radians(roll_angle_step_bf_cd * 0.01f);
    float pitch_step_rads = radians(pitch_angle_step_bf_cd * 0.01f);
    float yaw_step_rads   = radians(yaw_angle_step_bf_cd * 0.01f);

    // 通过期望的步进旋转目标姿态
    Quaternion attitude_target_update;
    attitude_target_update.from_axis_angle(Vector3f { roll_step_rads, pitch_step_rads, yaw_step_rads });
    _attitude_target = _attitude_target * attitude_target_update;
    _attitude_target.normalize();

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 设置速率前馈请求为零
    _euler_rate_target.zero();
    _ang_vel_target.zero();

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 控制飞行器的姿态和姿态速率，根据传递的推力矢量和航向速率。
// 函数的工作原理包括将输入值从厘度转换为弧度，根据一些条件和参数进行姿态控制计算，包括角速度限制、角速度整形和航向速率限制。
// 最后，将计算得到的目标姿态和姿态速率传递给四元数姿态控制器以控制飞行器的姿态。
// 这个函数在飞行控制系统中起到关键作用，以确保飞行器按照期望的姿态飞行。
// Command a thrust vector and heading rate
void AC_AttitudeControl::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{
    // 将以厘度为单位的公共接口中的值转换为弧度
    float heading_rate = radians(heading_rate_cds * 0.01f);

    if (slew_yaw) {
        // 如果启用了航向速率控制，则限制航向速率在指定范围内
        // 如果航向速率限制设置为零，表示禁用限制
        const float slew_yaw_max_rads = get_slew_yaw_max_rads();
        heading_rate                  = constrain_float(heading_rate, -slew_yaw_max_rads, slew_yaw_max_rads);
    }

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 将推力矢量转换为四元数姿态
    Quaternion thrust_vec_quat = attitude_from_thrust_vector(thrust_vector, 0.0f);

    // 计算x和y轴上的角度误差
    float      thrust_vector_diff_angle;
    Quaternion thrust_vec_correction_quat;
    Vector3f   attitude_error;
    float      returned_thrust_vector_angle;
    thrust_vector_rotation_angles(thrust_vec_quat, _attitude_target, thrust_vec_correction_quat, attitude_error, returned_thrust_vector_angle, thrust_vector_diff_angle);

    if (_rate_bf_ff_enabled) {
        // 当启用偏航加速度限制时，偏航输入整形器约束绕偏航轴的角加速度，将输出速率转向输入速率。
        _ang_vel_target.x = input_shaping_angle(attitude_error.x, _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, _dt);
        _ang_vel_target.y = input_shaping_angle(attitude_error.y, _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, _dt);

        // 当启用偏航加速度限制时，偏航输入整形器约束绕偏航轴的角加速度，将输出速率转向输入速率。
        _ang_vel_target.z = input_shaping_ang_vel(_ang_vel_target.z, heading_rate, get_accel_yaw_max_radss(), _dt, _rate_y_tc);

        // 限制角速度
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));
    } else {
        // 当未启用偏航加速度限制时，通过四元数姿态计算姿态目标
        Quaternion yaw_quat;
        yaw_quat.from_axis_angle(Vector3f { 0.0f, 0.0f, heading_rate * _dt });
        _attitude_target = _attitude_target * thrust_vec_correction_quat * yaw_quat;

        // 将速率前馈请求设置为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 将机体坐标系下的角速度转换为期望姿态的欧拉角导数
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 命令一个推力矢量、航向和航向速率
void AC_AttitudeControl::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // 一个零_slew_yaw_max_rads 表示禁用了这个设置
    const float slew_yaw_max_rads = get_slew_yaw_max_rads();

    // 从公共接口的厘度转换为弧度
    float heading_rate  = constrain_float(radians(heading_rate_cds * 0.01f), -slew_yaw_max_rads, slew_yaw_max_rads);
    float heading_angle = radians(heading_angle_cd * 0.01f);

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);

    // 将推力矢量和航向转换为四元数姿态
    const Quaternion desired_attitude_quat = attitude_from_thrust_vector(thrust_vector, heading_angle);

    if (_rate_bf_ff_enabled) {
        // 计算x和y轴上的角度误差。
        Vector3f   attitude_error;
        float      thrust_vector_diff_angle;
        Quaternion thrust_vec_correction_quat;
        float      returned_thrust_vector_angle;
        thrust_vector_rotation_angles(desired_attitude_quat, _attitude_target, thrust_vec_correction_quat, attitude_error, returned_thrust_vector_angle, thrust_vector_diff_angle);

        // 当启用航向加速度限制时，航向输入整形器约束绕航向轴的角加速度，将输出速率趋于输入速率。
        _ang_vel_target.x = input_shaping_angle(attitude_error.x, _input_tc, get_accel_roll_max_radss(), _ang_vel_target.x, _dt);
        _ang_vel_target.y = input_shaping_angle(attitude_error.y, _input_tc, get_accel_pitch_max_radss(), _ang_vel_target.y, _dt);
        _ang_vel_target.z = input_shaping_angle(attitude_error.z, _input_tc, get_accel_yaw_max_radss(), _ang_vel_target.z, heading_rate, slew_yaw_max_rads, _dt);

        // 限制角速度
        ang_vel_limit(_ang_vel_target, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), slew_yaw_max_rads);
    } else {
        // 设置持久的四元数目标姿态
        _attitude_target = desired_attitude_quat;

        // 将速率前馈请求设置为零
        _euler_rate_target.zero();
        _ang_vel_target.zero();
    }

    // 将机体坐标系的角速度转换为期望姿态的欧拉角导数
    ang_vel_to_euler_rate(_euler_angle_target, _ang_vel_target, _euler_rate_target);

    // 调用四元数姿态控制器
    attitude_controller_run_quat();
}

// 命令一个推力矢量和航向速率
void AC_AttitudeControl::input_thrust_vector_heading(const Vector3f& thrust_vector, HeadingCommand heading)
{
    // 根据航向模式选择不同的操作
    switch (heading.heading_mode) {
        case HeadingMode::Rate_Only:
            // 当仅为速率模式时，调用命令推力矢量和航向速率的函数
            input_thrust_vector_rate_heading(thrust_vector, heading.yaw_rate_cds);
            break;
        case HeadingMode::Angle_Only:
            // 当仅为角度模式时，调用命令推力矢量和航向角度的函数，并将速率设置为零
            input_thrust_vector_heading(thrust_vector, heading.yaw_angle_cd, 0.0);
            break;
        case HeadingMode::Angle_And_Rate:
            // 当角度和速率模式时，调用命令推力矢量和航向角度的函数，并指定速率
            input_thrust_vector_heading(thrust_vector, heading.yaw_angle_cd, heading.yaw_rate_cds);
            break;
    }
}

// 从给定的推力矢量和航向角创建四元数姿态
Quaternion AC_AttitudeControl::attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle) const
{
    // 定义一个朝上的单位矢量，表示垂直向上
    const Vector3f thrust_vector_up { 0.0f, 0.0f, -1.0f };

    // 如果推力矢量的长度平方接近零，将其重置为朝上的单位矢量
    if (is_zero(thrust_vector.length_squared())) {
        thrust_vector = thrust_vector_up;
    } else {
        thrust_vector.normalize(); // 将推力矢量标准化为单位矢量
    }

    // 通过叉积计算期望推力矢量和目标推力矢量之间的旋转向量
    Vector3f thrust_vec_cross = thrust_vector_up % thrust_vector;

    // 使用点积计算期望推力矢量和目标推力矢量之间的夹角
    const float thrust_vector_angle = acosf(constrain_float(thrust_vector_up * thrust_vector, -1.0f, 1.0f));

    // 标准化推力旋转矢量
    const float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_vector_angle)) {
        thrust_vec_cross = thrust_vector_up; // 如果长度或角度接近零，则将旋转矢量设置为朝上
    } else {
        thrust_vec_cross /= thrust_vector_length; // 将旋转矢量标准化为单位矢量
    }

    // 创建四元数表示推力矢量的旋转
    Quaternion thrust_vec_quat;
    thrust_vec_quat.from_axis_angle(thrust_vec_cross, thrust_vector_angle);

    // 创建一个表示给定航向角的四元数
    Quaternion yaw_quat;
    yaw_quat.from_axis_angle(Vector3f { 0.0f, 0.0f, 1.0f }, heading_angle);

    // 返回推力矢量四元数和航向四元数的组合
    return thrust_vec_quat * yaw_quat;
}

// 计算在目标姿态下跟随的机体坐标系角速度
void AC_AttitudeControl::attitude_controller_run_quat()
{
    // 获取当前的机体坐标系到NED坐标系的四元数旋转表示当前的飞行器姿态
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // 计算使推力矢量旋转以匹配期望姿态所需的角度误差
    Vector3f attitude_error;
    thrust_heading_rotation_angles(_attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // 使用角度误差来计算机体坐标系中的角速度修正
    _ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);

    // 确保角速度不超过用户配置的角速度限制
    ang_vel_limit(_ang_vel_body, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), radians(_ang_vel_yaw_max));

    // 计算机体坐标系中的前馈角速度目标，以便将推力矢量旋转到期望姿态
    Quaternion rotation_target_to_body  = attitude_body.inverse() * _attitude_target;
    Vector3f   ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target;

    // 修正推力矢量，并平滑添加前馈和航向输入
    _feedforward_scalar = 1.0f;

    // 当推力角误差大于2倍的阈值时，将航向角速度设置为当前的陀螺仪测量值
    if (_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE * 2.0f) {
        _ang_vel_body.z = _ahrs.get_gyro().z;
    } else if (_thrust_error_angle > AC_ATTITUDE_THRUST_ERROR_ANGLE) {
        // 当推力角误差大于阈值时，进行平滑修正，以保持飞行器稳定
        _feedforward_scalar = (1.0f - (_thrust_error_angle - AC_ATTITUDE_THRUST_ERROR_ANGLE) / AC_ATTITUDE_THRUST_ERROR_ANGLE);
        _ang_vel_body.x += ang_vel_body_feedforward.x * _feedforward_scalar;
        _ang_vel_body.y += ang_vel_body_feedforward.y * _feedforward_scalar;
        _ang_vel_body.z += ang_vel_body_feedforward.z;

        // 航向角速度平滑修正
        _ang_vel_body.z = _ahrs.get_gyro().z * (1.0 - _feedforward_scalar) + _ang_vel_body.z * _feedforward_scalar;
    } else {
        // 否则，将前馈角速度添加到机体坐标系中的角速度
        _ang_vel_body += ang_vel_body_feedforward;
    }

    // 如果启用机体坐标系的角速度前馈，根据机体坐标系中的角速度目标进行四元数的更新
    if (_rate_bf_ff_enabled) {
        Quaternion attitude_target_update;
        attitude_target_update.from_axis_angle(Vector3f { _ang_vel_target.x * _dt, _ang_vel_target.y * _dt, _ang_vel_target.z * _dt });
        _attitude_target = _attitude_target * attitude_target_update;
    }

    // 确保四元数保持标准化，以避免异常姿态
    _attitude_target.normalize();

    // 记录角度误差，以便在EKF（扩展卡尔曼滤波器）重置时进行处理，确保飞行器保持稳定
    _attitude_ang_error = attitude_body.inverse() * _attitude_target;
}

// thrust_heading_rotation_angles - 计算两个有序的旋转，将 attitude_body 四元数移动到 attitude_target 四元数。
// 根据静态输出饱和，限制偏航轴上的最大误差。
void AC_AttitudeControl::thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const
{
    // 计算推力矢量修正四元数，以使 attitude_target 和 attitude_body 的推力矢量一致
    Quaternion thrust_vector_correction;
    thrust_vector_rotation_angles(attitude_target, attitude_body, thrust_vector_correction, attitude_error, thrust_angle, thrust_error_angle);

    // 待完成：基于输出饱和和最大误差，限制横滚和俯仰误差。

    // 根据会饱和输出时的情况，限制偏航误差
    Quaternion heading_vec_correction_quat;

    // 计算偏航角加速度限制，将角速度限制的一半用作偏航加速度限制
    float heading_accel_max = constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS);

    if (!is_zero(get_rate_yaw_pid().kP())) {
        // 计算最大的偏航误差，以便在给定比例、P 值和最大加速度下产生饱和输出
        float heading_error_max = MIN(inv_sqrt_controller(1.0 / get_rate_yaw_pid().kP(), _p_angle_yaw.kP(), heading_accel_max), AC_ATTITUDE_YAW_MAX_ERROR_ANGLE);

        if (!is_zero(_p_angle_yaw.kP()) && fabsf(attitude_error.z) > heading_error_max) {
            // 如果偏航误差超过最大允许值，对其进行修正并更新 attitude_target
            attitude_error.z = constrain_float(wrap_PI(attitude_error.z), -heading_error_max, heading_error_max);
            heading_vec_correction_quat.from_axis_angle(Vector3f { 0.0f, 0.0f, attitude_error.z });
            attitude_target = attitude_body * thrust_vector_correction * heading_vec_correction_quat;
        }
    }
}

// thrust_vector_rotation_angles - 计算两个有序的旋转，将 attitude_body 四元数移动到 attitude_target 四元数。
// 第一个旋转校正推力矢量，第二个旋转校正偏航矢量。
void AC_AttitudeControl::thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const
{
    // 推力方向在任何固定于机体坐标系的坐标系中都是 [0,0,-1]，包括机体坐标系和目标坐标系。
    const Vector3f thrust_vector_up { 0.0f, 0.0f, -1.0f };

    // attitude_target 和 attitude_body 是从目标 / 机体坐标系到 NED 坐标系的被动旋转

    // 通过 attitude_target 旋转 [0,0,-1] 表示（获得视角）惯性坐标系中的目标推力矢量
    Vector3f att_target_thrust_vec = attitude_target * thrust_vector_up; // 目标推力矢量

    // 通过 attitude_body 旋转 [0,0,-1] 表示（获得视角）惯性坐标系中的当前推力矢量
    Vector3f att_body_thrust_vec = attitude_body * thrust_vector_up; // 当前推力矢量

    // 使用点积来计算当前倾斜角，以供外部函数使用
    thrust_angle = acosf(constrain_float(thrust_vector_up * att_body_thrust_vec, -1.0f, 1.0f));

    // 期望和当前推力矢量之间的叉积定义了旋转矢量
    Vector3f thrust_vec_cross = att_body_thrust_vec % att_target_thrust_vec;

    // 使用点积来计算目标和期望推力矢量之间的夹角
    thrust_error_angle = acosf(constrain_float(att_body_thrust_vec * att_target_thrust_vec, -1.0f, 1.0f));

    // 规范化推力旋转矢量
    float thrust_vector_length = thrust_vec_cross.length();
    if (is_zero(thrust_vector_length) || is_zero(thrust_error_angle)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // thrust_vector_correction 相对于机体坐标系定义，但其轴 thrust_vec_cross 是在惯性坐标系中计算的。
    // 首先，通过 attitude_body 的逆旋转将它从惯性坐标系中表示回机体坐标系
    thrust_vec_cross = attitude_body.inverse() * thrust_vec_cross;
    thrust_vector_correction.from_axis_angle(thrust_vec_cross, thrust_error_angle);

    // 计算 x 和 y 上的角度误差
    Vector3f rotation;
    thrust_vector_correction.to_axis_angle(rotation);
    attitude_error.x = rotation.x;
    attitude_error.y = rotation.y;

    // 在将推力矢量旋转到机体坐标系后计算所需的剩余旋转
    // heading_vector_correction
    Quaternion heading_vec_correction_quat = thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;

    // 在 z 上计算角度误差（x 和 y 应该为零）。
    heading_vec_correction_quat.to_axis_angle(rotation);
    attitude_error.z = rotation.z;
}

// input_shaping_angle - 根据角度误差计算角速度校正，以实现角速度的平滑调整，考虑加速度、速度和最大角速度限制。
//
// 参数：
//   error_angle: 角度误差
//   input_tc: 输入时间常数
//   accel_max: 最大加速度
//   target_ang_vel: 目标角速度
//   desired_ang_vel: 期望角速度
//   max_ang_vel: 最大角速度限制
//   dt: 时间步长
//
// 返回值：平滑调整后的角速度
//
float AC_AttitudeControl::input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float desired_ang_vel, float max_ang_vel, float dt)
{
    // 使用 sqrt 控制器，通过角度误差逐渐减小，来调整期望角速度
    desired_ang_vel += sqrt_controller(error_angle, 1.0f / MAX(input_tc, 0.01f), accel_max, dt);

    // 如果存在最大角速度限制，则将期望角速度限制在这个范围内
    if (is_positive(max_ang_vel)) {
        desired_ang_vel = constrain_float(desired_ang_vel, -max_ang_vel, max_ang_vel);
    }

    // 直接限制加速度以平滑曲线的起始部分
    return input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt, 0.0f);
}

// input_shaping_ang_vel - 根据速度时间常数来调整角速度请求，限制角加速度和减速度。
//
// 参数：
//   target_ang_vel: 目标角速度
//   desired_ang_vel: 期望的角速度
//   accel_max: 角加速度的最大限制
//   dt: 时间步长
//   input_tc: 输入时间常数，用于控制速度调整的速度
//
// 返回值：
//   经过角速度校正后的角速度
//
float AC_AttitudeControl::input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt, float input_tc)
{
    if (is_positive(input_tc)) {
        // 使用 sqrt 控制器，通过角速度误差逐渐减小，来调整角速度
        float error_rate        = desired_ang_vel - target_ang_vel;
        float desired_ang_accel = sqrt_controller(error_rate, 1.0f / MAX(input_tc, 0.01f), 0.0f, dt);
        desired_ang_vel         = target_ang_vel + desired_ang_accel * dt;
    }

    // 如果存在最大角加速度限制，则直接限制加速度，以平滑曲线的起始部分
    if (is_positive(accel_max)) {
        float delta_ang_vel = accel_max * dt;
        return constrain_float(desired_ang_vel, target_ang_vel - delta_ang_vel, target_ang_vel + delta_ang_vel);
    } else {
        return desired_ang_vel;
    }
}

// input_shaping_rate_predictor - 根据 AC_AttitudeControl 的设置，计算基于角度误差的期望角速度校正。
// 此函数可用于预测与角度请求相关的延迟。
//
// 参数：
//   error_angle: 角度误差（包括横滚和俯仰）
//   target_ang_vel: 期望的角速度（包括横滚和俯仰）
//   dt: 时间步长
//
void AC_AttitudeControl::input_shaping_rate_predictor(const Vector2f& error_angle, Vector2f& target_ang_vel, float dt) const
{
    if (_rate_bf_ff_enabled) {
        // 将横滚和俯仰的角加速度限制转换为欧拉轴
        target_ang_vel.x = input_shaping_angle(wrap_PI(error_angle.x), _input_tc, get_accel_roll_max_radss(), target_ang_vel.x, dt);
        target_ang_vel.y = input_shaping_angle(wrap_PI(error_angle.y), _input_tc, get_accel_pitch_max_radss(), target_ang_vel.y, dt);
    } else {
        // 使用 P 控制器来校正角速度
        const float angleP_roll  = _p_angle_roll.kP() * _angle_P_scale.x;
        const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
        target_ang_vel.x         = angleP_roll * wrap_PI(error_angle.x);
        target_ang_vel.y         = angleP_pitch * wrap_PI(error_angle.y);
    }

    // 限制角速度校正的幅度
    Vector3f ang_vel(target_ang_vel.x, target_ang_vel.y, 0.0f);
    ang_vel_limit(ang_vel, radians(_ang_vel_roll_max), radians(_ang_vel_pitch_max), 0.0f);

    target_ang_vel.x = ang_vel.x;
    target_ang_vel.y = ang_vel.y;
}

// ang_vel_limit - 限制角速度
//
// 参数：
//   euler_rad: 欧拉角的弧度表示
//   ang_vel_roll_max: 最大横滚角速度
//   ang_vel_pitch_max: 最大俯仰角速度
//   ang_vel_yaw_max: 最大偏航角速度
//
void AC_AttitudeControl::ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const
{
    // 如果横滚或俯仰角速度限制为零，则分别对横滚和俯仰角速度进行限制
    if (is_zero(ang_vel_roll_max) || is_zero(ang_vel_pitch_max)) {
        if (!is_zero(ang_vel_roll_max)) {
            euler_rad.x = constrain_float(euler_rad.x, -ang_vel_roll_max, ang_vel_roll_max);
        }
        if (!is_zero(ang_vel_pitch_max)) {
            euler_rad.y = constrain_float(euler_rad.y, -ang_vel_pitch_max, ang_vel_pitch_max);
        }
    } else {
        // 计算横滚和俯仰角速度的长度，将其限制在单位长度内
        Vector2f thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max, euler_rad.y / ang_vel_pitch_max);
        float    thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max / thrust_vector_length;
        }
    }

    // 对偏航角速度进行限制
    if (!is_zero(ang_vel_yaw_max)) {
        euler_rad.z = constrain_float(euler_rad.z, -ang_vel_yaw_max, ang_vel_yaw_max);
    }
}

// euler_accel_limit - 将机体坐标系的加速度限制转化为欧拉轴的限制
//
// 参数：
//   euler_rad: 欧拉角（弧度表示）
//   euler_accel: 欧拉轴的加速度
//
// 返回值：
//   Vector3f：转化后的欧拉轴加速度
//
Vector3f AC_AttitudeControl::euler_accel_limit(const Vector3f& euler_rad, const Vector3f& euler_accel)
{
    // 计算 sin 和 cos 值，将其限制在特定范围内，以防止出现除零错误
    float sin_phi   = constrain_float(fabsf(sinf(euler_rad.x)), 0.1f, 1.0f);
    float cos_phi   = constrain_float(fabsf(cosf(euler_rad.x)), 0.1f, 1.0f);
    float sin_theta = constrain_float(fabsf(sinf(euler_rad.y)), 0.1f, 1.0f);
    float cos_theta = constrain_float(fabsf(cosf(euler_rad.y)), 0.1f, 1.0f);

    Vector3f rot_accel;

    // 如果欧拉轴的任何一个分量为零或负值，或者加速度的任何一个分量为零，直接将结果设为欧拉轴的加速度
    if (is_zero(euler_accel.x) || is_zero(euler_accel.y) || is_zero(euler_accel.z) || is_negative(euler_accel.x) || is_negative(euler_accel.y) || is_negative(euler_accel.z)) {
        rot_accel.x = euler_accel.x;
        rot_accel.y = euler_accel.y;
        rot_accel.z = euler_accel.z;
    } else {
        // 否则，根据欧拉角的 sin 和 cos 值来限制加速度
        rot_accel.x = euler_accel.x;
        rot_accel.y = MIN(euler_accel.y / cos_phi, euler_accel.z / sin_phi);
        rot_accel.z = MIN(MIN(euler_accel.x / sin_theta, euler_accel.y / (sin_phi * cos_theta)), euler_accel.z / (cos_phi * cos_theta));
    }
    return rot_accel;
}

// reset_target_and_rate - 重置姿态目标为当前飞行器姿态，并将所有角速度设为零
//
// 参数：
//   reset_rate: 如果为 true，则同时将角速度重置为零
//
void AC_AttitudeControl::reset_target_and_rate(bool reset_rate)
{
    // 将姿态目标设置为当前姿态
    _ahrs.get_quat_body_to_ned(_attitude_target);

    if (reset_rate) {
        // 如果需要，将角速度设为零
        _ang_vel_target.zero();
        _euler_angle_target.zero();
    }
}

// reset_yaw_target_and_rate - 重置偏航目标为当前航向，并将偏航角速度设为零
//
// 参数：
//   reset_rate: 如果为 true，则同时将偏航角速度重置为零
//
void AC_AttitudeControl::reset_yaw_target_and_rate(bool reset_rate)
{
    // 将姿态目标的偏航角调整为当前航向
    float      yaw_shift = _ahrs.yaw - _euler_angle_target.z;
    Quaternion _attitude_target_update;
    _attitude_target_update.from_axis_angle(Vector3f{0.0f, 0.0f, yaw_shift});
    _attitude_target = _attitude_target_update * _attitude_target;

    if (reset_rate) {
        // 如果需要，将偏航角速度设为零
        _euler_rate_target.z = 0.0f;

        // 将期望姿态的欧拉角导数转换为机体坐标系下的角速度向量以供前馈使用
        euler_rate_to_ang_vel(_euler_angle_target, _euler_rate_target, _ang_vel_target);
    }
}

// inertial_frame_reset - 在发生EKF重置时，调整目标姿态以保持当前误差
//
void AC_AttitudeControl::inertial_frame_reset()
{
    // 获取飞行器的机体坐标系下姿态四元数
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // 重新计算目标姿态的四元数
    _attitude_target = attitude_body * _attitude_ang_error;

    // 计算目标姿态的欧拉角
    _attitude_target.to_euler(_euler_angle_target);
}

// euler_rate_to_ang_vel - 将欧拉角速率（Roll，Pitch，Yaw）转换为角速度矢量
void AC_AttitudeControl::euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads)
{
    // 计算 Roll (Phi) 和 Pitch (Theta) 角的正弦和余弦值，从欧拉角 euler_rad 中获取
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi   = sinf(euler_rad.x);
    float cos_phi   = cosf(euler_rad.x);

    // 计算角速度矢量的三个分量
    ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
    ang_vel_rads.y = cos_phi * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
    ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

// ang_vel_to_euler_rate - 将角速度矢量转换为 321-本体坐标系中的欧拉角导数
// 如果飞行器俯仰角度为正负90度，返回false
bool AC_AttitudeControl::ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads)
{
    // 计算欧拉角中俯仰角（Pitch）的正弦和余弦值，从欧拉角 euler_rad 中获取
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi   = sinf(euler_rad.x);
    float cos_phi   = cosf(euler_rad.x);

    // 当飞行器俯仰角度为正负90度时，欧拉角变得不连续。在这种情况下，返回false。
    if (is_zero(cos_theta)) {
        return false;
    }

    // 计算欧拉角导数的三个分量
    euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta / cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta / cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

// update_ang_vel_target_from_att_error - 根据姿态误差角度矢量更新角速度目标
Vector3f AC_AttitudeControl::update_ang_vel_target_from_att_error(const Vector3f& attitude_error_rot_vec_rad)
{
    Vector3f rate_target_ang_vel;

    // 从横滚角误差计算横滚角速度需求
    const float angleP_roll = _p_angle_roll.kP() * _angle_P_scale.x;
    if (_use_sqrt_controller && !is_zero(get_accel_roll_max_radss())) {
        rate_target_ang_vel.x = sqrt_controller(attitude_error_rot_vec_rad.x, angleP_roll, constrain_float(get_accel_roll_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt);
    } else {
        rate_target_ang_vel.x = angleP_roll * attitude_error_rot_vec_rad.x;
    }

    // 从俯仰角误差计算俯仰角速度需求
    const float angleP_pitch = _p_angle_pitch.kP() * _angle_P_scale.y;
    if (_use_sqrt_controller && !is_zero(get_accel_pitch_max_radss())) {
        rate_target_ang_vel.y = sqrt_controller(attitude_error_rot_vec_rad.y, angleP_pitch, constrain_float(get_accel_pitch_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS), _dt);
    } else {
        rate_target_ang_vel.y = angleP_pitch * attitude_error_rot_vec_rad.y;
    }

    // 从偏航角误差计算偏航角速度需求
    const float angleP_yaw = _p_angle_yaw.kP() * _angle_P_scale.z;
    if (_use_sqrt_controller && !is_zero(get_accel_yaw_max_radss())) {
        rate_target_ang_vel.z = sqrt_controller(attitude_error_rot_vec_rad.z, angleP_yaw, constrain_float(get_accel_yaw_max_radss() / 2.0f, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS), _dt);
    } else {
        rate_target_ang_vel.z = angleP_yaw * attitude_error_rot_vec_rad.z;
    }

    // 重置角度 P 比例缩放，保存已使用的值
    _angle_P_scale_used = _angle_P_scale;
    _angle_P_scale      = VECTORF_111;

    return rate_target_ang_vel;
}

// accel_limiting - 启用或禁用机体坐标系的前馈控制
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // 如果启用限制，从EEPROM加载或设置为默认值
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        // 禁用限制时将加速度限制设置为零
        _accel_roll_max.set(0.0f);
        _accel_pitch_max.set(0.0f);
        _accel_yaw_max.set(0.0f);
    }
}

// get_althold_lean_angle_max_cd - 获取优先保持高度而不是倾斜角度的飞行员输入的倾斜角度限制
float AC_AttitudeControl::get_althold_lean_angle_max_cd() const
{
    // 将弧度转换为厘度以供公共接口使用
    return MAX(ToDeg(_althold_lean_angle_max), AC_ATTITUDE_CONTROL_ANGLE_LIMIT_MIN) * 100.0f;
}

// max_rate_step_bf_roll - 返回使输出在4个时间步后达到最大值的滚转速率步长，单位为厘度/秒
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float dt_average      = AP::scheduler().get_filtered_loop_time();
    float alpha           = MIN(get_rate_roll_pid().get_filt_E_alpha(dt_average), get_rate_roll_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // TODO: 当存在thrust_max时，应该将0.5f替换为0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max       = 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_roll_pid().kD()) / _dt + get_rate_roll_pid().kP());
    if (is_positive(_ang_vel_roll_max)) {
        rate_max = MIN(rate_max, get_ang_vel_roll_max_rads());
    }
    return rate_max;
}

// max_rate_step_bf_pitch - 返回使输出在4个时间步后达到最大值的俯仰速率步长，单位为厘度/秒
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float dt_average      = AP::scheduler().get_filtered_loop_time();
    float alpha           = MIN(get_rate_pitch_pid().get_filt_E_alpha(dt_average), get_rate_pitch_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // TODO: 当存在thrust_max时，应该将0.5f替换为0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max       = 2.0f * throttle_hover * AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_pitch_pid().kD()) / _dt + get_rate_pitch_pid().kP());
    if (is_positive(_ang_vel_pitch_max)) {
        rate_max = MIN(rate_max, get_ang_vel_pitch_max_rads());
    }
    return rate_max;
}

// max_rate_step_bf_yaw - 返回使输出在4个时间步后达到最大值的偏航速率步长，单位为厘度/秒
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float dt_average      = AP::scheduler().get_filtered_loop_time();
    float alpha           = MIN(get_rate_yaw_pid().get_filt_E_alpha(dt_average), get_rate_yaw_pid().get_filt_D_alpha(dt_average));
    float alpha_remaining = 1 - alpha;
    // TODO: 当存在thrust_max时，应该将0.5f替换为0.5f * _motors.thrust_max
    float throttle_hover = constrain_float(_motors.get_throttle_hover(), 0.1f, 0.5f);
    float rate_max       = 2.0f * throttle_hover * AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX / ((alpha_remaining * alpha_remaining * alpha_remaining * alpha * get_rate_yaw_pid().kD()) / _dt + get_rate_yaw_pid().kP());
    if (is_positive(_ang_vel_yaw_max)) {
        rate_max = MIN(rate_max, get_ang_vel_yaw_max_rads());
    }
    return rate_max;
}

// pre_arm_checks - 执行预起飞检查以确保控制参数设置正确
// 如果存在无效参数，则返回 false 并设置 failure_msg，否则返回 true
bool AC_AttitudeControl::pre_arm_checks(const char* param_prefix, char* failure_msg, const uint8_t failure_msg_len)
{
    // 验证 AC_P 成员:
    const struct {
        const char* pid_name;
        AC_P&       p;
    } ps[] = {
        { "ANG_PIT", get_angle_pitch_p() },
        { "ANG_RLL", get_angle_roll_p() },
        { "ANG_YAW", get_angle_yaw_p() }
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(ps); i++) {
        // 所有 AC_P 必须具有正的 P 值:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // 验证 AC_PID 成员:
    const struct {
        const char* pid_name;
        AC_PID&     pid;
    } pids[] = {
        { "RAT_RLL", get_rate_roll_pid() },
        { "RAT_PIT", get_rate_pitch_pid() },
        { "RAT_YAW", get_rate_yaw_pid() },
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(pids); i++) {
        // 如果 PID 具有正的 FF，则只需确保 kP 和 kI 不为负数
        AC_PID&     pid      = pids[i].pid;
        const char* pid_name = pids[i].pid_name;

        if (is_positive(pid.ff())) {
            // kP 和 kI 必须为非负数:
            if (is_negative(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be 0", param_prefix, pid_name);
                return false;
            }

            if (is_negative(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be 0", param_prefix, pid_name);
                return false;
            }
        } else {
            // kP 和 kI 必须为正数:
            if (!is_positive(pid.kP())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be 0", param_prefix, pid_name);
                return false;
            }

            if (!is_positive(pid.kI())) {
                hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_I must be 0", param_prefix, pid_name);
                return false;
            }
        }

        // 不允许负的 D 项（但允许为零）
        if (is_negative(pid.kD())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_D must be 0", param_prefix, pid_name);
            return false;
        }
    }

    return true;
}

/*
  获取滚转、俯仰和偏航的斜率限制，用于 Lua 脚本中的振荡检测
*/
void AC_AttitudeControl::get_rpy_srate(float& roll_srate, float& pitch_srate, float& yaw_srate)
{
    roll_srate  = get_rate_roll_pid().get_pid_info().slew_rate;
    pitch_srate = get_rate_pitch_pid().get_pid_info().slew_rate;
    yaw_srate   = get_rate_yaw_pid().get_pid_info().slew_rate;
}
