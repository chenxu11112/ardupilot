#pragma once

/// @file    AC_AttitudeControl.h
/// @brief   ArduCopter attitude control library

#include <AC_PID/AC_P.h>
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_MultiCopter.h>

#define AC_ATTITUDE_CONTROL_ANGLE_P                   4.5f // 默认的滚转、俯仰和偏航角P增益

#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS     radians(40.0f)  // 用于稳定控制器的滚转和俯仰轴的最小机体坐标系加速度限制
#define AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS     radians(720.0f) // 用于稳定控制器的滚转和俯仰轴的最大机体坐标系加速度限制
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS      radians(10.0f)  // 用于稳定控制器的偏航轴的最小机体坐标系加速度限制
#define AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS      radians(120.0f) // 用于稳定控制器的偏航轴的最大机体坐标系加速度限制
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS      6000            // 偏航角误差的限制，单位为度。这应该导致最大转弯速度为10度/秒乘以姿态速率P，因此默认值为45度/秒。
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS 110000.0f       // 默认的滚转/俯仰轴的最大加速度，单位为百分度/秒/秒
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS  27000.0f        // 默认的偏航轴的最大加速度，单位为百分度/秒/秒

#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX        1.0f  // 机体坐标系速率控制器的最大输出（用于滚转-俯仰轴）
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX       1.0f  // 机体坐标系速率控制器的最大输出（用于偏航轴）
#define AC_ATTITUDE_RATE_RELAX_TC                     0.16f // 用于将速率I项衰减到其一半的时间常数

#define AC_ATTITUDE_THRUST_ERROR_ANGLE                radians(30.0f) // 推力角度误差，超过这个角度，偏航校正将受限
#define AC_ATTITUDE_YAW_MAX_ERROR_ANGLE               radians(45.0f) // 推力角度误差，超过这个角度，偏航校正将受限

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT        1 // 默认启用机体坐标系速率前馈控制

#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT    1.0f // 用于限制倾斜角，以防止飞行器失去高度的时间常数
#define AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX  0.8f // 用于限制倾斜角的最大油门，以防止飞行器失去高度

#define AC_ATTITUDE_CONTROL_MIN_DEFAULT               0.1f // 默认的最小油门混合
#define AC_ATTITUDE_CONTROL_MAN_DEFAULT               0.1f // 默认的手动油门混合
#define AC_ATTITUDE_CONTROL_MAX_DEFAULT               0.5f // 默认的最大油门混合
#define AC_ATTITUDE_CONTROL_MIN_LIMIT                 0.5f // 最小油门混合上限
#define AC_ATTITUDE_CONTROL_MAN_LIMIT                 4.0f // 手动油门混合上限
#define AC_ATTITUDE_CONTROL_MAX                       5.0f // 默认的最大油门混合
#define AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT           0.5f // 控制低油门要求和姿态控制之间的最大油门输出比率。更高的值有利于姿态优先于飞行员输入
#define AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH        1.0f // 默认的角度P/PD油门增强阈值

class AC_AttitudeControl {
public:
    AC_AttitudeControl(AP_AHRS_View&         ahrs,
                       const AP_MultiCopter& aparm,
                       AP_Motors&            motors)
        : _p_angle_roll(AC_ATTITUDE_CONTROL_ANGLE_P)
        , _p_angle_pitch(AC_ATTITUDE_CONTROL_ANGLE_P)
        , _p_angle_yaw(AC_ATTITUDE_CONTROL_ANGLE_P)
        , _angle_boost(0)
        , _use_sqrt_controller(true)
        , _throttle_rpy_mix_desired(AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT)
        , _throttle_rpy_mix(AC_ATTITUDE_CONTROL_THR_MIX_DEFAULT)
        , _ahrs(ahrs)
        , _aparm(aparm)
        , _motors(motors)
    {
        _singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    }

    static AC_AttitudeControl* get_singleton(void)
    {
        return _singleton;
    }

    // 空的析构函数，用于抑制编译器警告
    virtual ~AC_AttitudeControl() { }

    // 设置_dt / 获取_dt - _dt 是自上次更新姿态控制器以来的时间
    // _dt 应该基于上次由这些控制器使用的IMU读数的时间来设置
    // 姿态控制器应该在每个循环中运行激活控制器的更新，以确保正常运行
    void set_dt(float dt) { _dt = dt; }

    float get_dt() const { return _dt; }

    // PID存取器
    AC_P&                 get_angle_roll_p() { return _p_angle_roll; }   // 获取横滚角PID参数对象
    AC_P&                 get_angle_pitch_p() { return _p_angle_pitch; } // 获取俯仰角PID参数对象
    AC_P&                 get_angle_yaw_p() { return _p_angle_yaw; }     // 获取偏航角PID参数对象
    virtual AC_PID&       get_rate_roll_pid()        = 0;                // 获取横滚角速度PID对象（虚函数）
    virtual AC_PID&       get_rate_pitch_pid()       = 0;                // 获取俯仰角速度PID对象（虚函数）
    virtual AC_PID&       get_rate_yaw_pid()         = 0;                // 获取偏航角速度PID对象（虚函数）
    virtual const AC_PID& get_rate_roll_pid() const  = 0;                // 获取横滚角速度PID对象（虚函数，只读）
    virtual const AC_PID& get_rate_pitch_pid() const = 0;                // 获取俯仰角速度PID对象（虚函数，只读）
    virtual const AC_PID& get_rate_yaw_pid() const   = 0;                // 获取偏航角速度PID对象（虚函数，只读）

    // 获取横滚加速度限制（单位：cm/s²）
    float get_accel_roll_max_cdss() const { return _accel_roll_max; }

    // 获取横滚加速度限制（单位：rad/s²）
    float get_accel_roll_max_radss() const { return radians(_accel_roll_max * 0.01f); }

    // 设置横滚加速度限制（单位：cm/s²）
    void set_accel_roll_max_cdss(float accel_roll_max) { _accel_roll_max.set(accel_roll_max); }

    // 设置并保存横滚加速度限制（单位：cm/s²）
    void save_accel_roll_max_cdss(float accel_roll_max) { _accel_roll_max.set_and_save(accel_roll_max); }

    // 获取俯仰加速度限制（单位：cm/s²）
    float get_accel_pitch_max_cdss() const { return _accel_pitch_max; }

    // 获取俯仰加速度限制（单位：rad/s²）
    float get_accel_pitch_max_radss() const { return radians(_accel_pitch_max * 0.01f); }

    // 设置俯仰加速度限制（单位：cm/s²）
    void set_accel_pitch_max_cdss(float accel_pitch_max) { _accel_pitch_max.set(accel_pitch_max); }

    // 设置并保存俯仰加速度限制（单位：cm/s²）
    void save_accel_pitch_max_cdss(float accel_pitch_max) { _accel_pitch_max.set_and_save(accel_pitch_max); }

    // 获取偏航加速度限制（单位：cm/s²）
    float get_accel_yaw_max_cdss() const { return _accel_yaw_max; }

    // 获取偏航加速度限制（单位：rad/s²）
    float get_accel_yaw_max_radss() const { return radians(_accel_yaw_max * 0.01f); }

    // 设置偏航加速度限制（单位：cm/s²）
    void set_accel_yaw_max_cdss(float accel_yaw_max) { _accel_yaw_max.set(accel_yaw_max); }

    // 设置并保存偏航加速度限制（单位：cm/s²）
    void save_accel_yaw_max_cdss(float accel_yaw_max) { _accel_yaw_max.set_and_save(accel_yaw_max); }

    // 获取横滚角速度限制（单位：rad/s）
    float get_ang_vel_roll_max_rads() const { return radians(_ang_vel_roll_max); }

    // 获取俯仰角速度限制（单位：rad/s）
    float get_ang_vel_pitch_max_rads() const { return radians(_ang_vel_pitch_max); }

    // 获取偏航角速度限制（单位：rad/s）
    float get_ang_vel_yaw_max_rads() const { return radians(_ang_vel_yaw_max); }

    // 获取偏航速率限制（单位：deg/s）
    float get_slew_yaw_max_degs() const;

    // 获取角速率控制输入平滑时间常数
    float get_input_tc() const { return _input_tc; }

    // 设置角速率控制输入平滑时间常数
    void set_input_tc(float input_tc) { _input_tc.set(constrain_float(input_tc, 0.0f, 1.0f)); }

    // 确保姿态控制器误差为零以减小角速率控制器输出
    void relax_attitude_controllers();

    // 由子类 AC_AttitudeControl_TS 用于更改垂尾四旋翼的行为
    virtual void relax_attitude_controllers(bool exclude_pitch) { relax_attitude_controllers(); }

    // 重置角速率控制器的积分项
    void reset_rate_controller_I_terms();

    // 平滑地将角速率控制器的积分项重置为零（在0.5秒内）
    void reset_rate_controller_I_terms_smoothly();

    // 将姿态目标重置为飞行器姿态，并将所有角速率设为零
    // 如果 reset_rate 为 false，不重置角速率，以允许角速率控制器运行
    void reset_target_and_rate(bool reset_rate = true);

    // 将偏航目标重置为飞行器航向并将偏航角速率设为零
    // 如果 reset_rate 为 false，不重置角速率，以允许角速率控制器运行
    void reset_yaw_target_and_rate(bool reset_rate = true);

    // 处理来自EKF的姿态重置，自上一次迭代以来
    void inertial_frame_reset();

    // 命令一个带有前馈和平滑的四元数姿态
    // attitude_desired_quat：由角速度积分在每个时间步（_dt）更新
    virtual void input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target);

    // 通过角速度前馈和平滑命令欧拉横滚和俯仰角以及欧拉偏航角速率
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);
    // 通过角速度前馈和平滑命令欧拉横滚、俯仰和偏航角
    virtual void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

    // 通过角速度前馈和平滑命令欧拉偏航角速率和俯仰角，以机体坐标系中指定横滚角
    // （仅在 AC_AttitudeControl_TS 中用于垂尾四旋翼）
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd,
                                                                float euler_pitch_angle_cd, float euler_yaw_rate_cds) { }

    // 通过角速度前馈和平滑命令欧拉横滚、俯仰和偏航角速率
    virtual void input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds);

    // 通过角速度前馈和平滑命令机体坐标系中的角速率
    virtual void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // 通过角速度前馈和平滑命令机体坐标系中的角速率
    virtual void input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // 通过角速度前馈和平滑，仅使用速率环中的积分速率误差稳定化，命令机体坐标系中的角速率
    virtual void input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds);

    // 命令机体坐标系中的角度变化步进（即改变）
    virtual void input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd);

    // 命令地面坐标系中的推力矢量和偏航角速率和/或角度
    virtual void input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw = true);
    virtual void input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds);
    void         input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_cd) { input_thrust_vector_heading(thrust_vector, heading_cd, 0.0f); }

    // 将推力矢量和偏航角转换为地球坐标系中的四元数旋转
    Quaternion attitude_from_thrust_vector(Vector3f thrust_vector, float heading_angle) const;

    // 运行角速度控制器并将输出发送到电机
    virtual void rate_controller_run() = 0;

    // 将 321-本体欧拉角导数转换为角速度矢量
    void euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads);

    // 将角速度矢量转换为 321-本体欧拉角导数
    // 如果飞行器倾斜了90度以上，返回 false
    bool ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads);

    // 指定姿态控制器是否应在姿态修正中使用平方根控制器。
    // 在进行自动调整（Autotune）时使用，以确保调整 P 项时不受平方根控制器的加速度限制的影响。
    void use_sqrt_controller(bool use_sqrt_cont) { _use_sqrt_controller = use_sqrt_cont; }

    // 返回 NED 地球坐标系到姿态控制器目标姿态的 321-本体欧拉角，单位为厘度
    // **注意** 使用 vector3f*deg(100) 比 deg(vector3f)*100 或 deg(vector3d*100) 更高效，因为它以最少的乘法提供相同的结果。即使看起来像是一个错误，它是故意的。参见 issue 4895。
    Vector3f        get_att_target_euler_cd() const { return _euler_angle_target * degrees(100.0f); }
    const Vector3f& get_att_target_euler_rad() const { return _euler_angle_target; }

    // 返回四旋翼特定姿态控制输入方法使用的机体坐标系到 NED 的目标姿态
    Quaternion get_attitude_target_quat() const { return _attitude_target; }

    // 返回目标（设定点）的角速度 [rad/s]，在目标姿态框架中
    const Vector3f& get_attitude_target_ang_vel() const { return _ang_vel_target; }

    // 返回目标推力矢量和当前推力矢量之间的夹角
    float get_att_error_angle_deg() const { return degrees(_thrust_error_angle); }

    // 设置机体坐标系中的 x 轴角速度，单位为厘度/s
    void rate_bf_roll_target(float rate_cds) { _ang_vel_body.x = radians(rate_cds * 0.01f); }

    // 设置机体坐标系中的 y 轴角速度，单位为厘度/s
    void rate_bf_pitch_target(float rate_cds) { _ang_vel_body.y = radians(rate_cds * 0.01f); }

    // 设置机体坐标系中的 z 轴角速度，单位为厘度/s
    void rate_bf_yaw_target(float rate_cds) { _ang_vel_body.z = radians(rate_cds * 0.01f); }

    // 设置机体坐标系中的 x 轴系统识别角速度，单位为度/s
    void rate_bf_roll_sysid(float rate) { _sysid_ang_vel_body.x = rate; }

    // 设置机体坐标系中的 y 轴系统识别角速度，单位为度/s
    void rate_bf_pitch_sysid(float rate) { _sysid_ang_vel_body.y = rate; }

    // 设置机体坐标系中的 z 轴系统识别角速度，单位为度/s
    void rate_bf_yaw_sysid(float rate) { _sysid_ang_vel_body.z = rate; }

    // 设置 x 轴系统识别执行器
    void actuator_roll_sysid(float command) { _actuator_sysid.x = command; }

    // 设置 y 轴系统识别执行器
    void actuator_pitch_sysid(float command) { _actuator_sysid.y = command; }

    // 设置 z 轴系统识别执行器
    void actuator_yaw_sysid(float command) { _actuator_sysid.z = command; }

    // 返回导致在 4 个时间步后输出最大值的横滚角速度步进大小（弧度/s）
    float max_rate_step_bf_roll();

    // 返回导致在 4 个时间步后输出最大值的俯仰角速度步进大小（弧度/s）
    float max_rate_step_bf_pitch();

    // 返回导致在 4 个时间步后输出最大值的偏航角速度步进大小（弧度/s）
    float max_rate_step_bf_yaw();
    // 返回横滚角度步进大小（弧度），导致在 4 个时间步后输出最大值
    float max_angle_step_bf_roll() { return max_rate_step_bf_roll() / _p_angle_roll.kP(); }

    // 返回俯仰角度步进大小（弧度），导致在 4 个时间步后输出最大值
    float max_angle_step_bf_pitch() { return max_rate_step_bf_pitch() / _p_angle_pitch.kP(); }

    // 返回偏航角度步进大小（弧度），导致在 4 个时间步后输出最大值
    float max_angle_step_bf_yaw() { return max_rate_step_bf_yaw() / _p_angle_yaw.kP(); }

    // 返回用于角速度控制器的角速度矢量，单位为弧度
    Vector3f rate_bf_targets() const { return _ang_vel_body + _sysid_ang_vel_body; }

    // 返回角速度控制器的目标角速度，单位为弧度/s，在目标姿态框架中
    const Vector3f& get_rate_ef_targets() const { return _euler_rate_target; }

    // 启用或禁用机体坐标系下的前馈控制
    void bf_feedforward(bool enable_or_disable) { _rate_bf_ff_enabled.set(enable_or_disable); }

    // 启用或禁用机体坐标系下的前馈控制，并保存
    void bf_feedforward_save(bool enable_or_disable) { _rate_bf_ff_enabled.set_and_save(enable_or_disable); }

    // 返回机体坐标系下前馈控制设置
    bool get_bf_feedforward() { return _rate_bf_ff_enabled; }

    // 启用或禁用机体坐标系下的加速度限制
    void accel_limiting(bool enable_or_disable);

    // 更新 Alt_Hold 角度最大值
    virtual void update_althold_lean_angle_max(float throttle_in) = 0;

    // 设置输出油门
    virtual void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) = 0;

    // 获取传递给姿态控制器的油门（即传递给 set_throttle_out 的油门）
    float get_throttle_in() const { return _throttle_in; }

    // 返回应用于倾斜补偿的倾斜角增益
    float angle_boost() const { return _angle_boost; }

    // 返回优先考虑高度保持而不是倾斜角的飞行员输入的倾斜角限制，单位为厘度
    virtual float get_althold_lean_angle_max_cd() const;

    // 返回配置的倾斜角限制，单位为厘度
    float lean_angle_max_cd() const { return _aparm.angle_max; }

    // 返回倾斜角度，单位为度
    float lean_angle_deg() const { return degrees(_thrust_angle); }

    // 根据角度误差计算速度修正。角速度具有加速度和减速度限制，包括使用 smoothing_gain 进行基本的 jerk 限制
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float desired_ang_vel, float max_ang_vel, float dt);
    static float input_shaping_angle(float error_angle, float input_tc, float accel_max, float target_ang_vel, float dt) { return input_shaping_angle(error_angle, input_tc, accel_max, target_ang_vel, 0.0f, 0.0f, dt); }

    // 根据速率时间常数对速度请求进行形状调整。角加速度和减速度受限
    static float input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max, float dt, float input_tc);

    // 根据 AC_AttitudeControl 设置，计算角速度控制器中的预期角速度修正。
    // 该函数可用于预测与角度请求相关的延迟。
    void input_shaping_rate_predictor(const Vector2f& error_angle, Vector2f& target_ang_vel, float dt) const;

    // 将机体坐标系中的加速度限制转换为欧拉轴
    void ang_vel_limit(Vector3f& euler_rad, float ang_vel_roll_max, float ang_vel_pitch_max, float ang_vel_yaw_max) const;

    // 将机体坐标系中的加速度限制转换为欧拉轴
    Vector3f euler_accel_limit(const Vector3f& euler_rad, const Vector3f& euler_accel);

    // 计算跟随目标姿态的机体坐标系角速度
    void attitude_controller_run_quat();

    // thrust_heading_rotation_angles - 计算两个有序的旋转，将 attitude_body 四元数移动到 attitude_target 四元数。
    // 根据偏航轴的最大误差限制
    void thrust_heading_rotation_angles(Quaternion& attitude_target, const Quaternion& attitude_body, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const;

    // thrust_vector_rotation_angles - 计算两个有序的旋转，将 attitude_body 四元数移动到 attitude_target 四元数。
    // 第一次旋转纠正推力矢量，第二次旋转纠正偏航矢量。
    void thrust_vector_rotation_angles(const Quaternion& attitude_target, const Quaternion& attitude_body, Quaternion& thrust_vector_correction, Vector3f& attitude_error, float& thrust_angle, float& thrust_error_angle) const;

    // 参数合法性检查。应在起飞前调用一次
    virtual void parameter_sanity_check() { }

    // 如果油门混合值位于最低值，则返回 true
    virtual bool is_throttle_mix_min() const { return true; }

    // 控制滚转、俯仰、偏航和油门混合
    virtual void  set_throttle_mix_min() { }
    virtual void  set_throttle_mix_man() { }
    virtual void  set_throttle_mix_max(float ratio) { }
    virtual void  set_throttle_mix_value(float value) { }
    virtual float get_throttle_mix(void) const { return 0; }

    // 启用/禁用直升机的飞翼传递
    virtual void use_flybar_passthrough(bool passthrough, bool tail_passthrough) { }

    // 控制是否在直升机的机体坐标系到电机输出阶段使用漏电 i 项
    virtual void use_leaky_i(bool leaky_i) { }

    // 设置悬停滚动修剪参数的缩放系数，由车辆代码根据车辆状态使用
    virtual void set_hover_roll_trim_scalar(float scalar) { }

    // 返回要添加到滚动角度以进行悬停集体学习的厘度角度。用于直升机以抵消悬停时尾旋翼推力。AC_Attitude_Heli 会重载此函数以返回角度。
    virtual float get_roll_trim_cd() { return 0; }

    // 直通滚动和俯仰角度率，机体坐标系下偏航率目标
    virtual void passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds) { }

    // 提供有关此刻是否应解锁的反馈信息：
    bool pre_arm_checks(const char* param_prefix, char* failure_msg, const uint8_t failure_msg_len);

    // 启用支持倒置飞行的后端的倒置飞行
    virtual void set_inverted_flight(bool inverted) { }

    // 获取用于振荡检测的滚动、俯仰和偏航的斜率速率值，用于 Lua 脚本
    void get_rpy_srate(float& roll_srate, float& pitch_srate, float& yaw_srate);

    // 设置滚动和俯仰角速度整定时间常数
    void set_roll_pitch_rate_tc(float input_tc) { _rate_rp_tc = input_tc; }

    // 设置偏航角速度整定时间常数
    void set_yaw_rate_tc(float input_tc) { _rate_y_tc = input_tc; }

    // 设置单一环角度 P 比例尺度倍增器。这将替代之前应用的任何缩放
    void set_angle_P_scale(const Vector3f& angle_P_scale) { _angle_P_scale = angle_P_scale; }

    // 设置单一环角度 P 比例尺度倍增器，将乘以先前从此环路中应用的任何缩放。这允许应用于不同目的的不同缩放因子
    void set_angle_P_scale_mult(const Vector3f& angle_P_scale) { _angle_P_scale *= angle_P_scale; }

    // 获取在上一环路中使用的角度 P 比例尺度的值
    const Vector3f& get_last_angle_P_scale(void) const { return _angle_P_scale_used; }

    // 设置单一环 PD 比例尺度倍增器，将乘以先前从此环路中应用的任何缩放。这允许应用于不同目的的不同缩放因子
    void set_PD_scale_mult(const Vector3f& pd_scale) { _pd_scale *= pd_scale; }

    // 获取用于日志记录的上一环路中使用的 PD 比例尺度值
    const Vector3f& get_PD_scale_logging(void) const { return _pd_scale_used; }

    // 用户可设置的参数
    static const struct AP_Param::GroupInfo var_info[];

    static constexpr Vector3f VECTORF_111 { 1.0f, 1.0f, 1.0f };

protected:
    // 从 attitude_error_rot_vec_rad 更新 rate_target_ang_vel
    Vector3f update_ang_vel_target_from_att_error(const Vector3f& attitude_error_rot_vec_rad);

    // 返回要添加到滚转角度的弧度。用于直升机以抵消悬停时尾旋翼推力。由 AC_Attitude_Heli 重载以返回角度。
    virtual float get_roll_trim_rad() { return 0; }

    // 返回偏航缓慢变化速率的限制，单位是弧度/秒
    float get_slew_yaw_max_rads() const { return radians(get_slew_yaw_max_degs()); }

    // 在Loiter、RTL、Auto飞行模式下，偏航目标可更新的最大速率
    AP_Float _slew_yaw;

    // 地球坐标系下滚动、俯仰和偏航轴的最大角速度（以度/秒为单位）
    AP_Float _ang_vel_roll_max;
    AP_Float _ang_vel_pitch_max;
    AP_Float _ang_vel_yaw_max;

    // 地球坐标系下滚动轴的最大旋转加速度
    AP_Float _accel_roll_max;

    // 地球坐标系下俯仰轴的最大旋转加速度
    AP_Float _accel_pitch_max;

    // 地球坐标系下偏航轴的最大旋转加速度
    AP_Float _accel_yaw_max;

    // 启用/禁用机体坐标系下角速度前馈
    AP_Int8 _rate_bf_ff_enabled;

    // 启用/禁用角度提升
    AP_Int8 _angle_boost_enabled;

    // 角度控制 P 参数
    AC_P _p_angle_roll;
    AC_P _p_angle_pitch;
    AC_P _p_angle_yaw;

    // 角度限制时间常数（用于维持高度）
    AP_Float _angle_limit_tc;

    // 角速度控制输入平滑时间常数
    AP_Float _input_tc;

    // 采样周期间隔，单位为秒
    float _dt;
    // 这表示NED坐标系中的321内旋转，用于姿态控制器中的目标（设定点）姿态，单位为弧度。
    Vector3f _euler_angle_target;

    // 这表示目标（设定点）姿态中的角速度，作为321内固有欧拉角导数，单位为弧度每秒，用于姿态控制器。
    Vector3f _euler_rate_target;

    // 这表示NED坐标系中的四元数旋转，用于姿态控制器中的目标（设定点）姿态。
    Quaternion _attitude_target;

    // 这表示目标（设定点）姿态的角速度，作为角速度向量，单位为目标姿态框架中的弧度每秒。
    Vector3f _ang_vel_target;

    // 这是机体坐标系中的角速度，用于角速度控制器中的输入。
    Vector3f _ang_vel_body;

    // 这是添加到系统识别模式输出角度控制器的角速度，以弧度每秒为单位。
    // 在使用后立即重置为零。
    Vector3f _sysid_ang_vel_body;

    // 这是添加到PID输出的无单位值，由系统识别模式使用。
    // 在使用后立即重置为零。
    Vector3f _actuator_sysid;

    // 这表示机体坐标系中的四元数姿态误差，用于惯性坐标系复位处理。
    Quaternion _attitude_ang_error;

    // 目标推力矢量与当前推力矢量之间的角度。
    float _thrust_angle;

    // 目标推力矢量与当前推力矢量之间的角度误差。
    float _thrust_error_angle;

    // 作为输入提供给姿态控制器的油门。不包括角度提升。
    float _throttle_in = 0.0f;

    // 用于倾斜补偿的油门增加量。
    // 仅用于记录。
    float _angle_boost;

    // 指定姿态控制器是否应该在姿态修正中使用平方根控制器。
    // 在Autotune期间使用，以确保P项在不受平方根控制器加速度限制的情况下进行调整。
    bool _use_sqrt_controller;

    // 用于限制油门饱和时倾斜角度的过滤Alt_Hold参数。
    float _althold_lean_angle_max = 0.0f;

    // 期望的throttle_low_comp值，实际throttle_low_comp在1-2秒内逐渐变化到这个值。
    float _throttle_rpy_mix_desired;

    // 油门混合，介于0到1之间，以及大于悬停油门的比值。
    float _throttle_rpy_mix;

    // 偏航前馈百分比，允许在进行极端滚动和俯仰校正时使偏航执行器输出为零。
    float _feedforward_scalar = 1.0f;

    // 角速度控制输入平滑时间常数
    float _rate_rp_tc;
    float _rate_y_tc;

    // 滚动、俯仰和偏航的角度P缩放向量
    Vector3f _angle_P_scale { 1, 1, 1 };

    // 用于最后一个循环的角度P缩放，用于记录和四旋翼机角度P缩放。
    Vector3f _angle_P_scale_used;

    // 滚动、俯仰和偏航的PD缩放向量
    Vector3f _pd_scale { 1, 1, 1 };

    // 用于最后一个循环的PD缩放，用于记录
    Vector3f _pd_scale_used;

    // 外部库的引用
    const AP_AHRS_View&   _ahrs;
    const AP_MultiCopter& _aparm;
    AP_Motors&            _motors;

    static AC_AttitudeControl* _singleton;

protected:
    /*
      控制监测状态
    */
    struct {
        float rms_roll_P;  // 滚动P项的均方根值
        float rms_roll_D;  // 滚动D项的均方根值
        float rms_pitch_P; // 俯仰P项的均方根值
        float rms_pitch_D; // 俯仰D项的均方根值
        float rms_yaw;     // 偏航的均方根值
    } _control_monitor;

    // 在ControlMonitor中更新状态
    void control_monitor_filter_pid(float value, float& rms_P);
    void control_monitor_update(void);

    // 是否处于倒置飞行模式
    bool _inverted_flight;

public:
    // 记录CTRL消息
    void control_monitor_log(void) const;

    // 返回每个轴的当前均方根控制器滤波器
    float control_monitor_rms_output_roll(void) const;
    float control_monitor_rms_output_roll_P(void) const;
    float control_monitor_rms_output_roll_D(void) const;
    float control_monitor_rms_output_pitch_P(void) const;
    float control_monitor_rms_output_pitch_D(void) const;
    float control_monitor_rms_output_pitch(void) const;
    float control_monitor_rms_output_yaw(void) const;

    // 角度和/或角速度目标的结构
    enum class HeadingMode {
        Angle_Only,     // 仅角度
        Angle_And_Rate, // 角度和角速度
        Rate_Only       // 仅角速度
    };
    struct HeadingCommand {
        float       yaw_angle_cd; // 偏航角度（以厘度表示）
        float       yaw_rate_cds; // 偏航角速度（以厘度每秒表示）
        HeadingMode heading_mode; // 偏航模式
    };
    void input_thrust_vector_heading(const Vector3f& thrust_vector, HeadingCommand heading);
};
