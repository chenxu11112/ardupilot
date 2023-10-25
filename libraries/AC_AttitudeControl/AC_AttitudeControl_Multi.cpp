#include "AC_AttitudeControl_Multi.h"
#include <AC_PID/AC_PID.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),
    // @Param: RAT_RLL_P
    // @DisplayName: 横滚轴速度控制器P增益
    // @Description: 横滚轴速度控制器P增益。根据期望的横滚速率与实际横滚速率之间的差异进行修正。
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: 标准用户

    // @Param: RAT_RLL_I
    // @DisplayName: 横滚轴速度控制器I增益
    // @Description: 横滚轴速度控制器I增益。用于纠正期望的横滚速率与实际横滚速率之间的长期差异。
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: 标准用户

    // @Param: RAT_RLL_IMAX
    // @DisplayName: 横滚轴速度控制器I增益最大值
    // @Description: 横滚轴速度控制器I增益最大值。用于限制I项的输出。
    // @Range: 0 1
    // @Increment: 0.01
    // @User: 标准用户

    // @Param: RAT_RLL_D
    // @DisplayName: 横滚轴速度控制器D增益
    // @Description: 横滚轴速度控制器D增益。用于补偿期望横滚速率与实际横滚速率之间的短期变化。
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: 标准用户

    // @Param: RAT_RLL_FF
    // @DisplayName: 横滚轴速度控制器前馈
    // @Description: 横滚轴速度控制器前馈。
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: 标准用户

    // @Param: RAT_RLL_FLTT
    // @DisplayName: 横滚轴速度控制器目标频率（Hz）
    // @Description: 横滚轴速度控制器的目标频率（Hz）。
    // @Range: 5到100
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_RLL_FLTE
    // @DisplayName: 横滚轴速度控制器误差频率（Hz）
    // @Description: 横滚轴速度控制器的误差频率（Hz）。
    // @Range: 0到100
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_RLL_FLTD
    // @DisplayName: 横滚轴速度控制器导数频率（Hz）
    // @Description: 横滚轴速度控制器导数频率（Hz）。
    // @Range: 5到100
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_RLL_SMAX
    // @DisplayName: 横滚速率极限
    // @Description: 设置由比例和微分增益组合产生的斜率的上限。如果速度反馈产生的控制动作的振幅超过此值，D+P增益将减小以尊重此限制。这限制了由过大增益引起的高频振荡的幅度。限制应设置为不超过执行器最大斜率的25%，以允许负载效应。注意：增益不会减小到小于标称值的10%。值为零将禁用此功能。
    // @Range: 0到200
    // @Increment: 0.5
    // @User: 高级用户

    // @Param: RAT_RLL_PDMX
    // @DisplayName: 横滚轴速度控制器PD和最大和最小值
    // @Description: 横滚轴速度控制器PD项的和的最大/最小值。
    // @Range: 0到1
    // @Increment: 0.01
    // @User: 高级用户

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: 俯仰轴速度控制器P增益
    // @Description: 俯仰轴速度控制器P增益。根据期望的俯仰速率与实际俯仰速率之间的差异进行修正。
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: 标准用户

    // @Param: RAT_PIT_I
    // @DisplayName: 俯仰轴速度控制器I增益
    // @Description: 俯仰轴速度控制器I增益。用于纠正期望的俯仰速率与实际俯仰速率之间的长期差异。
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: 标准用户

    // @Param: RAT_PIT_IMAX
    // @DisplayName: 俯仰轴速度控制器I增益最大值
    // @Description: 俯仰轴速度控制器I增益最大值。用于限制I项的输出。
    // @Range: 0 1
    // @Increment: 0.01
    // @User: 标准用户

    // @Param: RAT_PIT_D
    // @DisplayName: 俯仰轴速度控制器D增益
    // @Description: 俯仰轴速度控制器D增益。用于补偿期望俯仰速率与实际俯仰速率之间的短期变化。
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: 标准用户

    // @Param: RAT_PIT_FF
    // @DisplayName: 俯仰轴速度控制器前馈
    // @Description: 俯仰轴速度控制器前馈。
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: 标准用户

    // @Param: RAT_PIT_FLTT
    // @DisplayName: 俯仰轴速度控制器目标频率（Hz）
    // @Description: 俯仰轴速度控制器的目标频率（Hz）。
    // @Range: 5到100
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_PIT_FLTE
    // @DisplayName: 俯仰轴速度控制器误差频率（Hz）
    // @Description: 俯仰轴速度控制器的误差频率（Hz）。
    // @Range: 0到100
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_PIT_FLTD
    // @DisplayName: 俯仰轴速度控制器导数频率（Hz）
    // @Description: 俯仰轴速度控制器导数频率（Hz）。
    // @Range: 5到100
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_PIT_SMAX
    // @DisplayName: 俯仰速率极限
    // @Description: 设置由比例和微分增益组合产生的斜率的上限。如果速度反馈产生的控制动作的振幅超过此值，D+P增益将减小以尊重此限制。这限制了由过大增益引起的高频振荡的幅度。限制应设置为不超过执行器最大斜率的25%，以允许负载效应。注意：增益不会减小到小于标称值的10%。值为零将禁用此功能。
    // @Range: 0到200
    // @Increment: 0.5
    // @User: 高级用户

    // @Param: RAT_PIT_PDMX
    // @DisplayName: 俯仰轴速度控制器PD和最大和最小值
    // @Description: 俯仰轴速度控制器PD项的和的最大/最小值。
    // @Range: 0到1
    // @Increment: 0.01
    // @User: 高级用户

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: 偏航轴速度控制器P增益
    // @Description: 偏航轴速度控制器P增益。根据期望的偏航速率与实际偏航速率之间的差异进行修正。
    // @Range: 0.10到2.50
    // @Increment: 0.005
    // @User: 标准用户

    // @Param: RAT_YAW_I
    // @DisplayName: 偏航轴速度控制器I增益
    // @Description: 偏航轴速度控制器I增益。用于纠正期望的偏航速率与实际偏航速率之间的长期差异。
    // @Range: 0.010到1.0
    // @Increment: 0.01
    // @User: 标准用户

    // @Param: RAT_YAW_IMAX
    // @DisplayName: 偏航轴速度控制器I增益最大值
    // @Description: 偏航轴速度控制器I增益最大值。用于限制I项的输出。
    // @Range: 0到1
    // @Increment: 0.01
    // @User: 标准用户

    // @Param: RAT_YAW_D
    // @DisplayName: 偏航轴速度控制器D增益
    // @Description: 偏航轴速度控制器D增益。用于补偿期望偏航速率与实际偏航速率之间的短期变化。
    // @Range: 0.000到0.02
    // @Increment: 0.001
    // @User: 标准用户

    // @Param: RAT_YAW_FF
    // @DisplayName: 偏航轴速度控制器前馈
    // @Description: 偏航轴速度控制器前馈。
    // @Range: 0到0.5
    // @Increment: 0.001
    // @User: 标准用户

    // @Param: RAT_YAW_FLTT
    // @DisplayName: 偏航轴速度控制器目标频率（Hz）
    // @Description: 偏航轴速度控制器的目标频率（Hz）。
    // @Range: 1到50
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_YAW_FLTE
    // @DisplayName: 偏航轴速度控制器误差频率（Hz）
    // @Description: 偏航轴速度控制器的误差频率（Hz）。
    // @Range: 0到20
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_YAW_FLTD
    // @DisplayName: 偏航轴速度控制器导数频率（Hz）
    // @Description: 偏航轴速度控制器导数频率（Hz）。
    // @Range: 5到50
    // @Increment: 1
    // @Units: Hz
    // @User: 标准用户

    // @Param: RAT_YAW_SMAX
    // @DisplayName: 偏航速率极限
    // @Description: 设置由比例和微分增益组合产生的斜率的上限。如果速度反馈产生的控制动作的振幅超过此值，D+P增益将减小以尊重此限制。这限制了由过大增益引起的高频振荡的幅度。限制应设置为不超过执行器最大斜率的25%，以允许负载效应。注意：增益不会减小到小于标称值的10%。值为零将禁用此功能。
    // @Range: 0到200
    // @Increment: 0.5
    // @User: 高级用户

    // @Param: RAT_YAW_PDMX
    // @DisplayName: 偏航轴速度控制器PD和最大和最小值
    // @Description: 偏航轴速度控制器PD项的和的最大/最小值。
    // @Range: 0到1
    // @Increment: 0.01
    // @User: 高级用户

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: 油门混合最小值
    // @Description: 在降落时使用的油门与姿态控制之间的优先级比较（较高的值意味着我们更注重姿态控制而不是油门控制）
    // @Range: 0.1到0.25
    // @User: 高级用户
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: 油门混合最大值
    // @Description: 在飞行时使用的油门与姿态控制之间的优先级比较（较高的值意味着我们更注重姿态控制而不是油门控制）
    // @Range: 0.5到0.9
    // @User: 高级用户
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: 油门混合手动模式
    // @Description: 手动飞行时使用的油门与姿态控制之间的优先级比较（较高的值意味着我们更注重姿态控制而不是油门控制）
    // @Range: 0.1到0.9
    // @User: 高级用户
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: THR_G_BOOST
    // @DisplayName: 油门增益增强
    // @Description: 油门增益增强比率。值为0表示不应用增益增强，值为1表示应用完全的增益增强。描述应用于俯仰和滚转上的角度P和PD的比率增加。
    // @Range: 0到1
    // @User: 高级用户
    AP_GROUPINFO("THR_G_BOOST", 7, AC_AttitudeControl_Multi, _throttle_gain_boost, 0.0f),

    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View& ahrs, const AP_MultiCopter& aparm, AP_MotorsMulticopter& motors)
    : AC_AttitudeControl(ahrs, aparm, motors)
    , _motors_multi(motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 更新高度保持模式的最大倾斜角
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // 计算基于油门的最大倾斜角
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // 防止除零错误
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    // 计算高度保持模式下的最大倾斜角
    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));

    // 通过低通滤波器更新最大倾斜角
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    // 将输入油门值保存到成员变量 _throttle_in
    _throttle_in = throttle_in;

    // 更新高度保持模式下的最大倾斜角
    update_althold_lean_angle_max(throttle_in);

    // 设置电机油门滤波截止频率
    _motors.set_throttle_filter_cutoff(filter_cutoff);

    if (apply_angle_boost) {
        // 应用角度增益（Angle Boost）
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // 为了记录目的清除角度增益
        _angle_boost = 0.0f;
    }

    // 设置电机油门输出
    _motors.set_throttle(throttle_in);

    // 设置电机油门平均值的最大值
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    // 限制输入的比率在 0.0 到 1.0 之间
    ratio = constrain_float(ratio, 0.0f, 1.0f);

    // 根据比率计算期望的油门混合值
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// 返回一个包括对滚转/俯仰角度补偿的油门值
// 油门值应在 0 ~ 1 之间
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        // 如果角度增益被禁用，重置角度增益并返回原始油门值
        _angle_boost = 0;
        return throttle_in;
    }

    // 计算当前倾斜角度下的余弦值
    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();

    // 计算反转因子，倾斜角度在 60 度以下时为 1，倾斜角度在 60 到 90 度之间时从 1 递减到 0
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);

    // 计算目标余弦值，这是期望的油门补偿目标
    float cos_tilt_target = cosf(_thrust_angle);

    // 计算角度增益因子，确保它在 0.1 到 1 之间
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    // 计算经过补偿后的油门值
    float throttle_out = throttle_in * inverted_factor * boost_factor;

    // 计算角度增益，这是补偿前后的油门变化量
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);

    // 返回经过补偿后的油门值
    return throttle_out;
}

// 返回一个包括对滚转/俯仰角度和油门混合最大值的油门值
// 油门值应在 0 ~ 1 之间
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    // 将输入的油门值限制在 0.0 到 1.0 之间
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);

    // 计算考虑油门混合最大值的油门值
    // 使用 MAX 函数来取最大值，考虑了油门混合最大值的影响
    // 油门混合最大值 _throttle_rpy_mix 控制油门混合的权重
    float throttle_out = MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);

    // 返回计算后的油门值
    return throttle_out;
}

// update_throttle_gain_boost - 在高油门变化时增强角度P/PD
void AC_AttitudeControl_Multi::update_throttle_gain_boost()
{
    // 如果油门的变化速率高于阈值 AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH
    if (_motors.get_throttle_slew_rate() > AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH) {
        // 增强 PD 增益
        const float pd_boost = constrain_float(_throttle_gain_boost + 1.0f, 1.0, 2.0);
        set_PD_scale_mult(Vector3f(pd_boost, pd_boost, 1.0f));

        // 增强角度P增益
        const float angle_p_boost = constrain_float((_throttle_gain_boost + 1.0f) * (_throttle_gain_boost + 1.0f), 1.0, 4.0);
        set_angle_P_scale_mult(Vector3f(angle_p_boost, angle_p_boost, 1.0f));
    }
}

// update_throttle_rpy_mix - 逐渐将 _throttle_rpy_mix 调整到所需值
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // 逐渐将 _throttle_rpy_mix 调整到 _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // 快速增加（例如，从 0.1 增加到 0.9 需要 0.4 秒）
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // 缓慢减少（从 0.9 减少到 0.1 需要 1.6 秒）
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);

        // 如果混合仍然高于正在使用的混合，立即重置
        const float throttle_hover = _motors.get_throttle_hover();
        const float throttle_in    = _motors.get_throttle();
        const float throttle_out   = MAX(_motors.get_throttle_out(), throttle_in);
        float       mix_used;
        // 由于 throttle_out >= throttle_in，在这一点上不需要检查 throttle_in < throttle_hover
        if (throttle_out < throttle_hover) {
            mix_used = (throttle_out - throttle_in) / (throttle_hover - throttle_in);
        } else {
            mix_used = throttle_out / throttle_hover;
        }

        _throttle_rpy_mix = MIN(_throttle_rpy_mix, MAX(mix_used, _throttle_rpy_mix_desired));
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::rate_controller_run()
{
    // 在高油门突变时，每个周期都会提高 angle_p/pd
    update_throttle_gain_boost();

    // 将油门与姿态混合逐渐调整到所需值（从这里调用，因为这方便在每次迭代中调用）
    update_throttle_rpy_mix();

    // 更新角速度（体轴）以考虑系统识别角速度
    _ang_vel_body += _sysid_ang_vel_body;

    // 获取最新的陀螺仪数据
    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    // 使用角速度 PID 控制器计算横滚、俯仰和偏航的控制输出
    _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x, _dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, _dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, _dt, _motors.limit.yaw, _pd_scale.z) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff() * _feedforward_scalar);

    // 重置系统识别角速度和系统识别的控制输出
    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    // 保存使用的 PD 比例因子并重置 PD 比例因子
    _pd_scale_used = _pd_scale;
    _pd_scale      = VECTORF_111;

    // 更新控制监视器
    control_monitor_update();
}

// 参数合法性检查。通常在起飞前调用一次。
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // 对油门混合参数进行合法性检查
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // 参数描述建议 thr-mix-man 不要超过 0.9，但我们允许最多为 4.0，
        // 这对于动力很大、悬停油门很低的直升机非常有用
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // 参数描述建议 thr-mix-max 不要超过 0.9，但我们允许最多为 5.0，
        // 这对于动力很大、悬停油门很低的直升机非常有用
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        // 如果最小值大于最大值，则恢复默认值
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

