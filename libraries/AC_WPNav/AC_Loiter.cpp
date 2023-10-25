#include "AC_Loiter.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#define LOITER_SPEED_DEFAULT             1250.0f // 默认悬停速度（cm/s）
#define LOITER_SPEED_MIN                 20.0f   // 最小悬停速度（cm/s）
#define LOITER_ACCEL_MAX_DEFAULT         500.0f  // 默认悬停模式中的加速度
#define LOITER_BRAKE_ACCEL_DEFAULT       250.0f  // 悬停模式刹车时的加速度
#define LOITER_BRAKE_JERK_DEFAULT        500.0f  // 悬停模式刹车时的加速度变化率
#define LOITER_BRAKE_START_DELAY_DEFAULT 1.0f    // 悬停模式刹车启动前的延迟时间（秒）
#define LOITER_VEL_CORRECTION_MAX        200.0f  // 用于校正悬停位置误差的最大速度
#define LOITER_POS_CORRECTION_MAX        200.0f  // 悬停位置误差的最大允许值
#define LOITER_ACTIVE_TIMEOUT_MS         200     // 如果悬停控制器在过去的200毫秒内被调用，则被视为活跃

// 参数信息数组
const AP_Param::GroupInfo AC_Loiter::var_info[] = {

    // 悬停最大期望倾斜角度
    // 飞行器悬停最大期望倾斜角度。设置为零则为PSC_ANGLE_MAX/ANGLE_MAX的2/3。
    // 飞行器最大倾斜角度仍受PSC_ANGLE_MAX/ANGLE_MAX限制。
    // 单位：度
    // 范围：0 到 45
    // 增量：1
    // 用户级别：高级
    AP_GROUPINFO("ANG_MAX", 1, AC_Loiter, _angle_max, 0.0f),

    // 悬停水平最大速度
    // 定义飞行器在悬停模式下水平行进的最大速度（以厘米/秒为单位）
    // 单位：cm/s
    // 范围：20 到 3500
    // 增量：50
    // 用户级别：标准
    AP_GROUPINFO("SPEED", 2, AC_Loiter, _speed_cms, LOITER_SPEED_DEFAULT),

    // 悬停最大修正加速度
    // 悬停模式下的最大修正加速度，以cm/s^2为单位。较高的值会使飞行器更积极地修正位置误差。
    // 单位：cm/s/s
    // 范围：100 到 981
    // 增量：1
    // 用户级别：高级
    AP_GROUPINFO("ACC_MAX", 3, AC_Loiter, _accel_cmss, LOITER_ACCEL_MAX_DEFAULT),

    // 悬停刹车加速度
    // 悬停模式刹车时的加速度，以cm/s/s为单位。较高的值会在遥控杆处于中间位置时更快地停止飞行器。
    // 单位：cm/s/s
    // 范围：25 到 250
    // 增量：1
    // 用户级别：高级
    AP_GROUPINFO("BRK_ACCEL", 4, AC_Loiter, _brake_accel_cmss, LOITER_BRAKE_ACCEL_DEFAULT),

    // 悬停刹车变化率
    // 悬停模式刹车时的加速度变化率，以cm/s/s/s为单位。较高的值将在刹车操作期间更快地减速，如果驾驶员在刹车操作中移动遥控杆，将更快地取消刹车。
    // 单位：cm/s/s/s
    // 范围：500 到 5000
    // 增量：1
    // 用户级别：高级
    AP_GROUPINFO("BRK_JERK", 5, AC_Loiter, _brake_jerk_max_cmsss, LOITER_BRAKE_JERK_DEFAULT),

    // 悬停刹车启动延迟时间（秒）
    // 悬停刹车启动前的延迟时间（秒）
    // 单位：秒
    // 范围：0 到 2
    // 增量：0.1
    // 用户级别：高级
    AP_GROUPINFO("BRK_DELAY", 6, AC_Loiter, _brake_delay, LOITER_BRAKE_START_DELAY_DEFAULT),

    AP_GROUPEND
};

// 默认构造函数。
// 请注意，向量/矩阵构造函数已隐式置零其值。
AC_Loiter::AC_Loiter(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control)
    : _inav(inav)
    , _ahrs(ahrs)
    , _pos_control(pos_control)
    , _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// 将目标初始化为距离ekf原点的位置（以厘米为单位）
void AC_Loiter::init_target(const Vector2f& position)
{
    sanity_check_params();

    // 初始化位置控制器的速度和加速度
    _pos_control.set_correction_speed_accel_xy(LOITER_VEL_CORRECTION_MAX, _accel_cmss);
    _pos_control.set_pos_error_max_xy_cm(LOITER_POS_CORRECTION_MAX);

    // 初始化位置控制器
    _pos_control.init_xy_controller_stopping_point();

    // 将期望加速度和角度初始化为零，以保持在当前位置
    _predicted_accel.zero();
    _desired_accel.zero();
    _predicted_euler_angle.zero();
    _brake_accel = 0.0f;

    // 设置目标位置
    _pos_control.set_pos_target_xy_cm(position.x, position.y);
}

/// 从当前位置和速度初始化位置和前馈速度
void AC_Loiter::init_target()
{
    sanity_check_params();

    // 初始化位置控制器的速度和加速度
    _pos_control.set_correction_speed_accel_xy(LOITER_VEL_CORRECTION_MAX, _accel_cmss);
    _pos_control.set_pos_error_max_xy_cm(LOITER_POS_CORRECTION_MAX);

    // 初始化位置控制器并将目标加速度平滑地趋近于零
    _pos_control.relax_velocity_controller_xy();

    // 从位置控制器初始化预测的加速度和角度
    _predicted_accel.x       = _pos_control.get_accel_target_cmss().x;
    _predicted_accel.y       = _pos_control.get_accel_target_cmss().y;
    _predicted_euler_angle.x = radians(_pos_control.get_roll_cd() * 0.01f);
    _predicted_euler_angle.y = radians(_pos_control.get_pitch_cd() * 0.01f);
    _brake_accel             = 0.0f;
}

/// 减小用于着陆的响应
void AC_Loiter::soften_for_landing()
{
    _pos_control.soften_for_landing_xy();
}

/// 以厘度为单位设置飞行员期望的加速度
// dt 应该是自上次调用此函数以来的时间（以秒为单位）
void AC_Loiter::set_pilot_desired_acceleration(float euler_roll_angle_cd, float euler_pitch_angle_cd)
{
    // 获取时间间隔 dt，通常是从上次调用该函数到现在的时间差（秒）
    const float dt = _attitude_control.get_dt();

    // 将以厘度为单位的横滚和俯仰角度转换为弧度
    const float euler_roll_angle = radians(euler_roll_angle_cd * 0.01f);
    const float euler_pitch_angle = radians(euler_pitch_angle_cd * 0.01f);

    // 假设飞行器在垂直方向不加速时，将飞行员的期望姿态转换为加速度向量
    const Vector3f desired_euler {euler_roll_angle, euler_pitch_angle, _ahrs.yaw};
    const Vector3f desired_accel = _pos_control.lean_angles_to_accel(desired_euler);

    // 将计算得到的加速度分量存储在 _desired_accel 中
    _desired_accel.x = desired_accel.x;
    _desired_accel.y = desired_accel.y;

    // 计算我们认为应该在的姿态与我们期望的姿态之间的差异
    Vector2f angle_error(wrap_PI(euler_roll_angle - _predicted_euler_angle.x), wrap_PI(euler_pitch_angle - _predicted_euler_angle.y));

    // 根据期望姿态和预测姿态计算角速度
    _attitude_control.input_shaping_rate_predictor(angle_error, _predicted_euler_rate, dt);

    // 基于预测的角速度更新预测姿态
    _predicted_euler_angle += _predicted_euler_rate * dt;

    // 假设飞行器在垂直方向不加速时，将预测的姿态转换为加速度向量
    const Vector3f predicted_euler {_predicted_euler_angle.x, _predicted_euler_angle.y, _ahrs.yaw};
    const Vector3f predicted_accel = _pos_control.lean_angles_to_accel(predicted_euler);

    // 将计算得到的加速度分量存储在 _predicted_accel 中
    _predicted_accel.x = predicted_accel.x;
    _predicted_accel.y = predicted_accel.y;
}

/// 获取基于水平位置和速度的停止点向量
void AC_Loiter::get_stopping_point_xy(Vector2f& stopping_point) const
{
    Vector2p stop;
    _pos_control.get_stopping_point_xy_cm(stop);
    stopping_point = stop.tofloat();
}

/// 获取悬停时的最大倾斜角
float AC_Loiter::get_angle_max_cd() const
{
    // 如果 _angle_max 不是正数，执行以下操作：
    if (!is_positive(_angle_max)) {
        // 返回 _attitude_control.lean_angle_max_cd() 和 _pos_control.get_lean_angle_max_cd() 的较小值
        // 然后将其乘以 (2.0f / 3.0f) 并返回
        return MIN(_attitude_control.lean_angle_max_cd(), _pos_control.get_lean_angle_max_cd()) * (2.0f / 3.0f);
    }
    // 如果 _angle_max 是正数，执行以下操作：
    // 返回 _angle_max 乘以 100.0f 和 _pos_control.get_lean_angle_max_cd() 中的较小值
    return MIN(_angle_max * 100.0f, _pos_control.get_lean_angle_max_cd());
}

/// 运行悬停控制器
void AC_Loiter::update(bool avoidance_on)
{
    calc_desired_velocity(avoidance_on);
    _pos_control.update_xy_controller();
}

// 参数检查
void AC_Loiter::sanity_check_params()
{
    _speed_cms.set(MAX(_speed_cms, LOITER_SPEED_MIN));
    _accel_cmss.set(MIN(_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max_cd() * 0.01f))));
}

/// 更新期望速度（即前馈）以考虑飞手请求的加速度和虚拟的风阻
/// 更新的速度直接发送到位置控制器
void AC_Loiter::calc_desired_velocity(bool avoidance_on)
{
    float ekfGndSpdLimit, ahrsControlScaleXY;
    AP::ahrs().getControlLimits(ekfGndSpdLimit, ahrsControlScaleXY);

    const float dt = _pos_control.get_dt();

    // 计算悬停速度限制，即LOITER_SPEED参数设置的值与由EKF观察光流限制调整后的值的较小值
    float gnd_speed_limit_cms = MIN(_speed_cms, ekfGndSpdLimit * 100.0f);
    gnd_speed_limit_cms       = MAX(gnd_speed_limit_cms, LOITER_SPEED_MIN);

    float pilot_acceleration_max = angle_to_accel(get_angle_max_cd() * 0.01) * 100;

    // 检查dt是否为负值
    if (is_negative(dt)) {
        return;
    }

    // 从位置控制器获取悬停的期望速度
    const Vector3f& desired_vel_3d = _pos_control.get_vel_desired_cms();
    Vector2f        desired_vel { desired_vel_3d.x, desired_vel_3d.y };

    // 使用我们的预测的加速度更新期望速度
    desired_vel.x += _predicted_accel.x * dt;
    desired_vel.y += _predicted_accel.y * dt;

    Vector2f loiter_accel_brake;
    float    desired_speed = desired_vel.length();
    if (!is_zero(desired_speed)) {
        Vector2f desired_vel_norm = desired_vel / desired_speed;

        // 基于期望速度计算阻力加速度
        float drag_decel = pilot_acceleration_max * desired_speed / gnd_speed_limit_cms;

        // 如果摇杆居中，计算刹车加速度
        float loiter_brake_accel = 0.0f;
        if (_desired_accel.is_zero()) {
            if ((AP_HAL::millis() - _brake_timer) > _brake_delay * 1000.0f) {
                float brake_gain   = _pos_control.get_vel_xy_pid().kP() * 0.5f;
                loiter_brake_accel = constrain_float(sqrt_controller(desired_speed, brake_gain, _brake_jerk_max_cmsss, dt), 0.0f, _brake_accel_cmss);
            }
        } else {
            loiter_brake_accel = 0.0f;
            _brake_timer       = AP_HAL::millis();
        }
        _brake_accel += constrain_float(loiter_brake_accel - _brake_accel, -_brake_jerk_max_cmsss * dt, _brake_jerk_max_cmsss * dt);
        loiter_accel_brake = desired_vel_norm * _brake_accel;

        // 使用阻力和刹车加速度更新期望速度
        desired_speed = MAX(desired_speed - (drag_decel + _brake_accel) * dt, 0.0f);
        desired_vel   = desired_vel_norm * desired_speed;
    }

    // 添加刹车加速度到期望加速度
    _desired_accel -= loiter_accel_brake;

    // 应用EKF限制到期望速度 - 此限制由EKF计算，根据需要调整以确保尊重某些传感器的限制（例如光流传感）
    float horizSpdDem = desired_vel.length();
    if (horizSpdDem > gnd_speed_limit_cms) {
        desired_vel.x = desired_vel.x * gnd_speed_limit_cms / horizSpdDem;
        desired_vel.y = desired_vel.y * gnd_speed_limit_cms / horizSpdDem;
    }
#if !APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    if (avoidance_on) {
        // 限制速度以防止围栏违规
        // TODO: 我们还需要限制_desired_accel
        AC_Avoid* _avoid = AP::ac_avoid();
        if (_avoid != nullptr) {
            Vector3f avoidance_vel_3d { desired_vel.x, desired_vel.y, 0.0f };
            _avoid->adjust_velocity(avoidance_vel_3d, _pos_control.get_pos_xy_p().kP(), _accel_cmss, _pos_control.get_pos_z_p().kP(), _pos_control.get_max_accel_z_cmss(), dt);
            desired_vel = Vector2f { avoidance_vel_3d.x, avoidance_vel_3d.y };
        }
    }
#endif // !APM_BUILD_ArduPlane

    // 获取悬停期望速度，从位置控制器中存储的位置获取
    Vector2p target_pos = _pos_control.get_pos_target_cm().xy();

    // 使用我们的预测速度更新目标位置
    target_pos += (desired_vel * dt).topostype();

    // 将调整后的前馈加速度和速度发送回位置控制器
    _pos_control.set_pos_vel_accel_xy(target_pos, desired_vel, _desired_accel);
}
