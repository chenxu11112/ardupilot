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

#include "AP_MotorsMatrix.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

// 初始化电机矩阵
//
// 该函数用于初始化电机矩阵。根据请求的机架类别和类型，设置电机并启用快速通道或即时 PWM。
// 如果机架类别是脚本机架，不执行任何其他操作，因为脚本需要调用专用的初始化函数。
//
// 参数：
// - frame_class：机架类别，指示机架的类型或配置。
// - frame_type：机架类型，指示机架的详细配置。
//
void AP_MotorsMatrix::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 记录请求的机架类别和类型
    _active_frame_class = frame_class;
    _active_frame_type  = frame_type;

    if (frame_class == MOTOR_FRAME_SCRIPTING_MATRIX) {
        // 如果是脚本机架类别，不执行任何其他操作，因为脚本需要调用专用的初始化函数
        return;
    }

    // 设置电机
    setup_motors(frame_class, frame_type);

    // 启用快速通道或即时 PWM
    set_update_rate(_speed_hz);
}

#if AP_SCRIPTING_ENABLED
// 为 Lua 脚本专用的初始化
//
// 该函数用于为 Lua 脚本初始化电机矩阵。它确保正确数量的电机已添加，并设置 MAV 类型以及通道正常化因子。如果初始化失败，将返回 false。
//
// 参数：
// - expected_num_motors：期望的电机数量，以确定是否正确添加了所需数量的电机。
//
// 返回值：
// - 如果成功初始化，则返回 true；否则返回 false。
//
bool AP_MotorsMatrix::init(uint8_t expected_num_motors)
{
    if (_active_frame_class != MOTOR_FRAME_SCRIPTING_MATRIX) {
        // 不是正确的类别
        return false;
    }

    // 确保正确数量的电机已添加
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    normalise_rpy_factors();

    set_update_rate(_speed_hz);

    return true;
}

// AP_MotorsMatrix::set_throttle_factor - 从脚本设置油门系数
// 用于从脚本中设置特定电机的油门系数
// 参数：
// - motor_num：电机编号
// - throttle_factor：要设置的油门系数
// 返回值：
// - 成功设置则返回 true，否则返回 false
bool AP_MotorsMatrix::set_throttle_factor(int8_t motor_num, float throttle_factor)
{
    // 如果不是脚本帧类别，返回 false
    if (_active_frame_class != MOTOR_FRAME_SCRIPTING_MATRIX) {
        return false;
    }

    // 如果电机已经初始化或者给定的电机未启用，返回 false
    if (initialised_ok() || !motor_enabled[motor_num]) {
        return false;
    }

    // 设置电机的油门系数
    _throttle_factor[motor_num] = throttle_factor;
    return true;
}

#endif // AP_SCRIPTING_ENABLED

// AP_MotorsMatrix::set_update_rate - 设置电机更新频率
// 用于设置电机的更新频率，以赫兹为单位。
//
// 参数：
// - speed_hz：要设置的更新频率，以赫兹为单位。
//
void AP_MotorsMatrix::set_update_rate(uint16_t speed_hz)
{
    // 记录请求的频率
    _speed_hz = speed_hz;

    // 创建一个掩码以表示哪些电机已启用
    uint32_t mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }

    // 将更新频率应用于已启用的电机
    rc_set_freq(mask, _speed_hz);
}

// AP_MotorsMatrix::set_frame_class_and_type - 设置机架类别和类型（例如四旋翼、六旋翼、直升机）
// 用于设置机架的类别（frame_class）和类型（frame_type）。
//
// 参数：
// - frame_class：机架类别，例如四旋翼、六旋翼、直升机等。
// - frame_type：机架类型，例如 X 型、十字型等。
//
void AP_MotorsMatrix::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 如果已经解锁或无更改，立即退出
    if (armed() || (frame_class == _active_frame_class && _active_frame_type == frame_type)) {
        return;
    }
    _active_frame_class = frame_class;
    _active_frame_type  = frame_type;

    // 初始化新的机架类别和类型
    init(frame_class, frame_type);
}

// AP_MotorsMatrix::output_to_motors - 输出电机控制信号
// 根据当前的 SpoolState（电机状态）和推力需求，将电机控制信号发送到电机。
//
void AP_MotorsMatrix::output_to_motors()
{
    int8_t i;

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // 关闭电机，输出为零
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }
        case SpoolState::GROUND_IDLE:
            // 在解锁但未起飞时，将输出发送到电机
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
                }
            }
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // 根据推力需求设置电机输出
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]));
                }
            }
            break;
    }

    // 将输出转换为 PWM 信号并发送到每个电机
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }
}

// AP_MotorsMatrix::get_motor_mask - 获取正在使用电机的位掩码
// 返回一个位掩码，表示哪些输出正在用于电机控制（1 表示正在使用）
// 这可用于确保其他 PWM 输出（例如伺服）不会发生冲突。
//
// 返回值：电机位掩码，1 表示正在使用，0 表示未使用
//
uint32_t AP_MotorsMatrix::get_motor_mask()
{
    uint32_t motor_mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_mask |= 1U << i;
        }
    }
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // 将父类的掩码添加到当前掩码中
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// AP_MotorsMatrix::boost_ratio - 根据 _thrust_boost_ratio 返回在 boost_value 和 normal_value 之间缩放的值
// 当 _thrust_boost_ratio 为 1 时，返回值为 boost_value
// 当 _thrust_boost_ratio 为 0 时，返回值为 normal_value
//
// 参数：
// - boost_value：用于缩放的增强值
// - normal_value：用于缩放的普通值
//
// 返回值：缩放后的值
//
float AP_MotorsMatrix::boost_ratio(float boost_value, float normal_value) const
{
    return _thrust_boost_ratio * boost_value + (1.0 - _thrust_boost_ratio) * normal_value;
}
// AP_MotorsMatrix::output_armed_stabilizing - 发送命令到电机
// 包括新的缩放稳定补丁
//
void AP_MotorsMatrix::output_armed_stabilizing()
{
    // 应用电压和气压补偿
    const float compensation_gain = thr_lin.get_compensation_gain(); // 电池电压和高度补偿

    // 俯仰推力输入值，范围为+/- 1.0
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;

    // 横滚推力输入值，范围为+/- 1.0
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;

    // 偏航推力输入值，范围为+/- 1.0
    float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;

    // 油门推力输入值，范围为0.0 - 1.0
    float throttle_thrust = get_throttle() * compensation_gain;

    // 油门平均最大值，范围为0.0 - 1.0
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    // 油门最大值，范围为0.0 - 1.0，如果启用了推力增强，则不限制最大推力
    const float throttle_thrust_max = boost_ratio(1.0, _throttle_thrust_max * compensation_gain);

    // 检查油门是否在零和当前限制的油门之间
    if (throttle_thrust <= 0.0f) {
        throttle_thrust      = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust      = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // 确保 throttle_avg_max 位于输入油门和最大油门之间
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // 提供最大横滚、俯仰和偏航范围的油门
    // 计算可提供最大控制范围的允许的平均推力的最高值
    float throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);

    // 计算可以容纳最大偏航控制的推力，即：
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - 这将在最高电机上方和最低电机下方提供最大可能的余量
    //      2. 以下之间的更高值：
    //            a) 飞手的油门输入
    //            b) 飞手输入油门和悬停油门之间的点 _throttle_rpy_mix
    //      情况＃2确保我们永远不会将油门提高到悬停油门以上，除非飞手已命令这样做。
    //      情况＃2b 允许我们提高油门超过飞手命令的油门，但不会使直升机上升。
    //      我们会选择＃1（用于最佳的偏航控制）如果这意味着降低油门到电机（即，我们会选择降低油门，*因为*它提供了更好的偏航控制）
    //      只有当油门相当低时，我们才会选择＃2（飞手和悬停油门的混合物）。我们倾向于降低油门而不是提高油门，因为飞手已经命令这样做。
    //
    // 在电机丢失的情况下，我们会从我们的计算中去除最高电机输出，并允许该电机超过1.0
    // 为确保控制和最大纠正性能，Hex 和 Octo 有一些应该使用的最佳设置
    // Y6               : MOT_YAW_HEADROOM = 350, ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.375, ATC_RAT_PIT_IMAX = 0.375, ATC_RAT_YAW_IMAX = 0.375
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.75,  ATC_RAT_PIT_IMAX = 0.75,  ATC_RAT_YAW_IMAX = 0.375
    // 可用的下限下限可能会导致电机丢失时的姿态偏移。六边形飞机只有一些边缘，必须小心处理
    // Hex              : MOT_YAW_HEADROOM = 0,   ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.25,  ATC_RAT_PIT_IMAX = 0.25,  ATC_RAT_YAW_IMAX = 0.25
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.5,   ATC_RAT_PIT_IMAX = 0.5,   ATC_RAT_YAW_IMAX = 0.25
    // 四旋翼飞机不能使用电机丢失处理，因为它没有足够的自由度。
    //
    // 计算可以适应推力范围的偏航量
    // 这总是等于或小于飞行员或速度控制器的要求偏航量
    float yaw_allowed = 1.0f; // 我们可以适应的偏航量
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // 计算横滚和俯仰的推力输出
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];

            // 检查可以在该通道上使用的最大偏航控制
            // 如果启用了推力增强，则排除任何丢失的电机
            if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index)) {
                const float thrust_rp_best_throttle = throttle_thrust_best_rpy + _thrust_rpyt_out[i];
                float       motor_room;
                if (is_positive(yaw_thrust * _yaw_factor[i])) {
                    // 上限余量
                    motor_room = 1.0 - thrust_rp_best_throttle;
                } else {
                    // 下限余量
                    motor_room = thrust_rp_best_throttle;
                }
                const float motor_yaw_allowed = MAX(motor_room, 0.0) / fabsf(_yaw_factor[i]);
                yaw_allowed                   = MIN(yaw_allowed, motor_yaw_allowed);
            }
        }
    }

    // 计算可以使用的最大偏航控制
    // todo: 使 _yaw_headroom 0 到 1
    float yaw_allowed_min = (float)_yaw_headroom * 0.001f;

    // 如果启用了推力增强，将偏航余量增加到50%
    yaw_allowed_min = boost_ratio(0.5, yaw_allowed_min);

    // 让偏航访问最小的余量
    yaw_allowed = MAX(yaw_allowed, yaw_allowed_min);

    // 包括通过 _thrust_boost_ratio 缩放的丢失电机，以使该电机在计算中平滑过渡
    if (_thrust_boost && motor_enabled[_motor_lost_index]) {
        // 检查可以在该通道上使用的最大偏航控制
        // 如果启用了推力增强，则排除任何丢失的电机
        if (!is_zero(_yaw_factor[_motor_lost_index])) {
            const float thrust_rp_best_throttle = throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index];
            float       motor_room;
            if (is_positive(yaw_thrust * _yaw_factor[_motor_lost_index])) {
                motor_room = 1.0 - thrust_rp_best_throttle;
            } else {
                motor_room = thrust_rp_best_throttle;
            }
            const float motor_yaw_allowed = MAX(motor_room, 0.0) / fabsf(_yaw_factor[_motor_lost_index]);
            yaw_allowed                   = boost_ratio(yaw_allowed, MIN(yaw_allowed, motor_yaw_allowed));
        }
    }

    if (fabsf(yaw_thrust) > yaw_allowed) {
        // 无法使用所有指定的偏航量
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw  = true;
    }

    // 将偏航控制添加到推力输出
    float rpy_low  = 1.0f;  // 最低推力值
    float rpy_high = -1.0f; // 最高推力值
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // 记录最低横滚 + 俯仰 + 偏航命令
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // 记录最高横滚 + 俯仰 + 偏航命令
            // 如果启用了推力增强，则排除任何丢失的电机
            if (_thrust_rpyt_out[i] > rpy_high && (!_thrust_boost || i != _motor_lost_index)) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }
    // 包括通过 _thrust_boost_ratio 缩放的丢失电机，以使该电机在计算中平滑过渡
    if (_thrust_boost) {
        // 记录最高横滚 + 俯仰 + 偏航命令
        if (_thrust_rpyt_out[_motor_lost_index] > rpy_high && motor_enabled[_motor_lost_index]) {
            rpy_high = boost_ratio(rpy_high, _thrust_rpyt_out[_motor_lost_index]);
        }
    }

    // 计算所需的缩放以使组合的推力输出适应输出范围
    float rpy_scale = 1.0f;
    if (rpy_high - rpy_low > 1.0f) {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }
    if (throttle_avg_max + rpy_low < 0) {
        rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low);
    }

    // 计算电机可以接近所需油门的程度
    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    float thr_adj            = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f) {
        // 横滚、俯仰和偏航都使用全范围
        limit.roll  = true;
        limit.pitch = true;
        limit.yaw   = true;
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else if (thr_adj < 0.0f) {
        // 无法将油门降至所需值
        // todo: 添加下限标志并确保在高度控制器中正确处理它
        thr_adj = 0.0f;
    } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high)) {
        // 无法将油门提高到所需值
        thr_adj              = 1.0f - (throttle_thrust_best_rpy + rpy_high);
        limit.throttle_upper = true;
    }

    // 为每个电机添加缩放的横滚、俯仰、约束的偏航和油门
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = (throttle_thrust_best_plus_adj * _throttle_factor[i]) + (rpy_scale * _thrust_rpyt_out[i]);
        }
    }

    // 确定调制器的推力补偿
    // 补偿系数永远不可能为零
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // 检查失败的电机
    check_for_failed_motor(throttle_thrust_best_plus_adj);
}

// 检查电机故障
//   应立即在 output_armed_stabilizing 后运行
//   第一个参数是：
//      a) throttle_thrust_best_rpy：提供最大横滚、俯仰和偏航范围但不上升的油门级别（从0到1）
//      b) thr_adj：飞行员期望油门与 throttle_thrust_best_rpy 之间的差值
//   记录经过滤波和缩放的电机输出值到 _thrust_rpyt_out_filt 数组
//   如果电机平衡，则将 thrust_balanced 设置为 true，如果检测到电机故障，则设置为 false
//   设置 _motor_lost_index 为失败电机的索引
void AP_MotorsMatrix::check_for_failed_motor(float throttle_thrust_best_plus_adj)
{
    // 记录经过滤波和缩放的推力输出，以供电机丢失监控用
    float alpha = _dt / (_dt + 0.5f);
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
        }
    }

    float   rpyt_high     = 0.0f;
    float   rpyt_sum      = 0.0f;
    uint8_t number_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            number_motors++;
            rpyt_sum += _thrust_rpyt_out_filt[i];
            // 记录最高经过滤的推力命令
            if (_thrust_rpyt_out_filt[i] > rpyt_high) {
                rpyt_high = _thrust_rpyt_out_filt[i];
                // 在推力增强激活时保持电机丢失索引不变
                if (!_thrust_boost) {
                    _motor_lost_index = i;
                }
            }
        }
    }

    float thrust_balance = 1.0f;
    if (rpyt_sum > 0.1f) {
        thrust_balance = rpyt_high * number_motors / rpyt_sum;
    }
    // 确保在电机数量小于6的多旋翼飞机上不激活推力平衡
    if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced) {
        _thrust_balanced = false;
    }
    if (thrust_balance <= 1.25f && !_thrust_balanced) {
        _thrust_balanced = true;
    }

    // 检查推力增强是否使用的油门超过了 _throttle_thrust_max
    if ((_throttle_thrust_max * thr_lin.get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9f) && _thrust_balanced) {
        _thrust_boost = false;
    }
}

// output_test_seq - 以指定的 PWM 值旋转电机
//  motor_seq 是电机的顺序号，从 1 到框架上的电机数量
//  pwm 值是要输出的实际 PWM 值，通常在 1000 ~ 2000 范围内
void AP_MotorsMatrix::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // 循环遍历所有可能的顺序，旋转与该描述相匹配的任何电机
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // 打开此电机
            rc_write(i, pwm);
        }
    }
}

// output_test_num - 旋转连接到指定输出通道的电机
//  （应仅在测试期间执行）
//  如果电机输出通道被重新映射，则使用映射的通道。
//  如果设置了电机输出，返回 true；否则返回 false。
//  pwm 值是要输出的实际 PWM 值，通常在 1000 ~ 2000 范围内
bool AP_MotorsMatrix::output_test_num(uint8_t output_channel, int16_t pwm)
{
    if (!armed()) {
        return false;
    }

    // 通道是否在支持的范围内？
    if (output_channel > AP_MOTORS_MAX_NUM_MOTORS - 1) {
        return false;
    }

    // 电机是否已启用？
    if (!motor_enabled[output_channel]) {
        return false;
    }

    rc_write(output_channel, pwm); // 输出
    return true;
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor)
{
    if (initialised_ok()) {
        // do not allow motors to be set if the current frame type has init correctly
        return;
    }

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {

        // enable motor
        motor_enabled[motor_num] = true;

        // set roll, pitch, yaw and throttle factors
        _roll_factor[motor_num]     = roll_fac;
        _pitch_factor[motor_num]    = pitch_fac;
        _yaw_factor[motor_num]      = yaw_fac;
        _throttle_factor[motor_num] = throttle_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num]    = false;
        _roll_factor[motor_num]     = 0.0f;
        _pitch_factor[motor_num]    = 0.0f;
        _yaw_factor[motor_num]      = 0.0f;
        _throttle_factor[motor_num] = 0.0f;
    }
}

void AP_MotorsMatrix::add_motors(const struct MotorDef* motors, uint8_t num_motors)
{
    for (uint8_t i = 0; i < num_motors; i++) {
        const auto& motor = motors[i];
        add_motor(i, motor.angle_degrees, motor.yaw_factor, motor.testing_order);
    }
}
void AP_MotorsMatrix::add_motors_raw(const struct MotorDefRaw* motors, uint8_t num_motors)
{
    for (uint8_t i = 0; i < num_motors; i++) {
        const auto& m = motors[i];
        add_motor_raw(i, m.roll_fac, m.pitch_fac, m.yaw_fac, m.testing_order);
    }
}
#if AP_MOTORS_FRAME_QUAD_ENABLED
bool AP_MotorsMatrix::setup_quad_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "QUAD";
    _mav_type           = MAV_TYPE_QUADROTOR;
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_PLUS: {
            _frame_type_string = "PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_X: {
            _frame_type_string = "X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
# if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
        case MOTOR_FRAME_TYPE_NYT_PLUS: {
            _frame_type_string = "NYT_PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 90, 0, 2 },
                { -90, 0, 4 },
                { 0, 0, 1 },
                { 180, 0, 3 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_NYT_X: {
            _frame_type_string = "NYT_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, 0, 1 },
                { -135, 0, 3 },
                { -45, 0, 4 },
                { 135, 0, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
# endif // APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
        case MOTOR_FRAME_TYPE_BF_X: {
            // betaflight quad X order
            // see: https://fpvfrenzy.com/betaflight-motor-order/
            _frame_type_string = "BF_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_BF_X_REV: {
            // betaflight quad X order, reversed motors
            _frame_type_string = "X_REV";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_DJI_X: {
            // DJI quad X order
            // see https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
            _frame_type_string = "DJI_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_CW_X: {
            // "clockwise X" motor order. Motors are ordered clockwise from front right
            // matching test order
            _frame_type_string = "CW_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_V: {
            _frame_type_string = "V";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, 0.7981f, 1 },
                { -135, 1.0000f, 3 },
                { -45, -0.7981f, 4 },
                { 135, -1.0000f, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_H: {
            // H frame set-up - same as X but motors spin in opposite directiSons
            _frame_type_string = "H";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_VTAIL: {
            /*
                Tested with: Lynxmotion Hunter Vtail 400
                - inverted rear outward blowing motors (at a 40 degree angle)
                - should also work with non-inverted rear outward blowing motors
                - no roll in rear motors
                - no yaw in front motors
                - should fly like some mix between a tricopter and X Quadcopter

                Roll control comes only from the front motors, Yaw control only from the rear motors.
                Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

                Note: if we want the front motors to help with yaw,
                    motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                    motors 3's yaw factor should be changed to -sin(radians(40))
            */
            _frame_type_string = "VTAIL";
            add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
            add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
            add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
            add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
            break;
        }
        case MOTOR_FRAME_TYPE_ATAIL:
            /*
                The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
                - The Yaw factors are reversed, because the rear motors are facing different directions

                With V-Shaped VTails, the props make a V-Shape when spinning, but with
                A-Shaped VTails, the props make an A-Shape when spinning.
                - Rear thrust on a V-Shaped V-Tail Quad is outward
                - Rear thrust on an A-Shaped V-Tail Quad is inward

                Still functions the same as the V-Shaped VTail mixing below:
                - Yaw control is entirely in the rear motors
                - Roll is is entirely in the front motors
            */
            _frame_type_string = "ATAIL";
            add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
            add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
            add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
            add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
            break;
        case MOTOR_FRAME_TYPE_PLUSREV: {
            // plus with reversed motor directions
            _frame_type_string = "PLUSREV";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_Y4:
            _frame_type_string = "Y4";
            // Y4 motor definition with right front CCW, left front CW
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { -1.0f, 1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 1.0f, 1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        default:
            // quad frame class does not support this frame type
            return false;
    }
    return true;
}
#endif // AP_MOTORS_FRAME_QUAD_ENABLED
#if AP_MOTORS_FRAME_HEXA_ENABLED
bool AP_MotorsMatrix::setup_hexa_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "HEXA";
    _mav_type           = MAV_TYPE_HEXAROTOR;
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_PLUS: {
            _frame_type_string = "PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { 60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_X: {
            _frame_type_string = "X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_H: {
            // H is same as X except middle motors are closer to center
            _frame_type_string = "H";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { -1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { 1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_DJI_X: {
            _frame_type_string = "DJI_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_CW_X: {
            _frame_type_string = "CW_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        default:
            // hexa frame class does not support this frame type
            return false;
    } // hexa
    return true;
}
#endif ////AP_MOTORS_FRAME_HEXA_ENABLED
#if AP_MOTORS_FRAME_OCTA_ENABLED
bool AP_MotorsMatrix::setup_octa_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "OCTA";
    _mav_type           = MAV_TYPE_OCTOROTOR;
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_PLUS: {
            _frame_type_string = "PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
            };

            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_X: {
            _frame_type_string = "X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { 67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { 157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { 112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_V: {
            _frame_type_string = "V";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { 0.83f, 0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { -0.67f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { 0.67f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { -0.50f, -1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 1.00f, 1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { -0.83f, 0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { -1.00f, 1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 0.50f, -1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_H: {
            _frame_type_string = "H";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { -1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { -1.0f, 0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { 1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { 1.0f, 0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { -1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_I: {
            _frame_type_string = "I";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { 0.333f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { -0.333f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { 0.333f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { -0.333f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { -1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { 1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_DJI_X: {
            _frame_type_string = "DJI_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8 },
                { -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
                { -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { 157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_CW_X: {
            _frame_type_string = "CW_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
                { -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        default:
            // octa frame class does not support this frame type
            return false;
    } // octa frame type
    return true;
}
#endif // AP_MOTORS_FRAME_OCTA_ENABLED
#if AP_MOTORS_FRAME_OCTAQUAD_ENABLED
bool AP_MotorsMatrix::setup_octaquad_matrix(motor_frame_type frame_type)
{
    _mav_type           = MAV_TYPE_OCTOROTOR;
    _frame_class_string = "OCTAQUAD";
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_PLUS: {
            _frame_type_string = "PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_X: {
            _frame_type_string = "X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_V: {
            _frame_type_string = "V";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, 0.7981f, 1 },
                { -45, -0.7981f, 7 },
                { -135, 1.0000f, 5 },
                { 135, -1.0000f, 3 },
                { -45, 0.7981f, 8 },
                { 45, -0.7981f, 2 },
                { 135, 1.0000f, 4 },
                { -135, -1.0000f, 6 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_H: {
            // H frame set-up - same as X but motors spin in opposite directions
            _frame_type_string = "H";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_CW_X: {
            _frame_type_string = "CW_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        // BF/X cinelifters using two 4-in-1 ESCs are quite common
        // see: https://fpvfrenzy.com/betaflight-motor-order/
        case MOTOR_FRAME_TYPE_BF_X: {
            _frame_type_string = "BF_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_BF_X_REV: {
            // betaflight octa quad X order, reversed motors
            _frame_type_string = "X_REV";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
                { 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        default:
            // octaquad frame class does not support this frame type
            return false;
    } // octaquad
    return true;
}
#endif // AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#if AP_MOTORS_FRAME_DODECAHEXA_ENABLED
bool AP_MotorsMatrix::setup_dodecahexa_matrix(motor_frame_type frame_type)
{
    _mav_type           = MAV_TYPE_DODECAROTOR;
    _frame_class_string = "DODECAHEXA";
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_PLUS: {
            _frame_type_string = "PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },    // forward-top
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },     // forward-bottom
                { 60, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },    // forward-right-top
                { 60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },   // forward-right-bottom
                { 120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },  // back-right-top
                { 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },   // back-right-bottom
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },   // back-top
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },  // back-bottom
                { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 9 }, // back-left-top
                { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 10 }, // back-left-bottom
                { -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 11 },  // forward-left-top
                { -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12 }, // forward-left-bottom
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_X: {
            _frame_type_string = "X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },   // forward-right-top
                { 30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },    // forward-right-bottom
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },    // right-top
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },   // right-bottom
                { 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },  // back-right-top
                { 150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },   // back-right-bottom
                { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 7 },  // back-left-top
                { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 }, // back-left-bottom
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 9 },  // left-top
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 10 },  // left-bottom
                { -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 11 },  // forward-left-top
                { -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12 }, // forward-left-bottom
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        default:
            // dodeca-hexa frame class does not support this frame type
            return false;
    } // dodecahexa
    return true;
}
#endif // AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#if AP_MOTORS_FRAME_Y6_ENABLED
bool AP_MotorsMatrix::setup_y6_matrix(motor_frame_type frame_type)
{
    _mav_type           = MAV_TYPE_HEXAROTOR;
    _frame_class_string = "Y6";
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_Y6B: {
            // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
            _frame_type_string = "Y6B";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { -1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { -1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { 0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3 },
                { 0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
                { 1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { 1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_Y6F: {
            // Y6 motor layout for FireFlyY6
            _frame_type_string = "Y6F";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { 0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { -1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { 0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { -1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 1.0f, 0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
        default: {
            _frame_type_string = "default";
            static const AP_MotorsMatrix::MotorDefRaw motors[] {
                { -1.0f, 0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
                { 1.0f, 0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 5 },
                { 1.0f, 0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
                { 0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { -1.0f, 0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 1 },
                { 0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            };
            add_motors_raw(motors, ARRAY_SIZE(motors));
            break;
        }
    } // y6
    return true;
}
#endif // AP_MOTORS_FRAME_Y6_ENABLED
#if AP_MOTORS_FRAME_DECA_ENABLED
bool AP_MotorsMatrix::setup_deca_matrix(motor_frame_type frame_type)
{
    _mav_type           = MAV_TYPE_DECAROTOR;
    _frame_class_string = "DECA";
    switch (frame_type) {
        case MOTOR_FRAME_TYPE_PLUS: {
            _frame_type_string = "PLUS";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 36, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 108, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
                { -108, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8 },
                { -72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 9 },
                { -36, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 10 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        case MOTOR_FRAME_TYPE_X:
        case MOTOR_FRAME_TYPE_CW_X: {
            _frame_type_string = "X/CW_X";
            static const AP_MotorsMatrix::MotorDef motors[] {
                { 18, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
                { 54, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
                { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
                { 126, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
                { 162, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
                { -162, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
                { -126, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
                { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 8 },
                { -54, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 9 },
                { -18, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 10 },
            };
            add_motors(motors, ARRAY_SIZE(motors));
            break;
        }
        default:
            // deca frame class does not support this frame type
            return false;
    } // deca
    return true;
}
#endif // AP_MOTORS_FRAME_DECA_ENABLED

void AP_MotorsMatrix::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }
    set_initialised_ok(false);
    bool success = true;

    switch (frame_class) {
#if AP_MOTORS_FRAME_QUAD_ENABLED
        case MOTOR_FRAME_QUAD:
            success = setup_quad_matrix(frame_type);
            break; // quad
#endif             // AP_MOTORS_FRAME_QUAD_ENABLED
#if AP_MOTORS_FRAME_HEXA_ENABLED
        case MOTOR_FRAME_HEXA:
            success = setup_hexa_matrix(frame_type);
            break;
#endif // AP_MOTORS_FRAME_HEXA_ENABLED
#if AP_MOTORS_FRAME_OCTA_ENABLED
        case MOTOR_FRAME_OCTA:
            success = setup_octa_matrix(frame_type);
            break;
#endif // AP_MOTORS_FRAME_OCTA_ENABLED
#if AP_MOTORS_FRAME_OCTAQUAD_ENABLED
        case MOTOR_FRAME_OCTAQUAD:
            success = setup_octaquad_matrix(frame_type);
            break;
#endif // AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#if AP_MOTORS_FRAME_DODECAHEXA_ENABLED
        case MOTOR_FRAME_DODECAHEXA:
            success = setup_dodecahexa_matrix(frame_type);
            break;
#endif // AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#if AP_MOTORS_FRAME_Y6_ENABLED
        case MOTOR_FRAME_Y6:
            success = setup_y6_matrix(frame_type);
            break;
#endif // AP_MOTORS_FRAME_Y6_ENABLED
#if AP_MOTORS_FRAME_DECA_ENABLED
        case MOTOR_FRAME_DECA:
            success = setup_deca_matrix(frame_type);
            break;
#endif // AP_MOTORS_FRAME_DECA_ENABLED
        default:
            // matrix doesn't support the configured class
            success   = false;
            _mav_type = MAV_TYPE_GENERIC;
            break;
    } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    if (!success) {
        _frame_class_string = "UNSUPPORTED";
    }
    set_initialised_ok(success);
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
// normalizes throttle factors so max value is 1 and no value is less than 0
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac     = 0.0f;
    float pitch_fac    = 0.0f;
    float yaw_fac      = 0.0f;
    float throttle_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            roll_fac     = MAX(roll_fac, fabsf(_roll_factor[i]));
            pitch_fac    = MAX(pitch_fac, fabsf(_pitch_factor[i]));
            yaw_fac      = MAX(yaw_fac, fabsf(_yaw_factor[i]));
            throttle_fac = MAX(throttle_fac, MAX(0.0f, _throttle_factor[i]));
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
            }
            if (!is_zero(throttle_fac)) {
                _throttle_factor[i] = MAX(0.0f, _throttle_factor[i] / throttle_fac);
            }
        }
    }
}

/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}

/*
  disable the use of motor torque to control yaw. Used when an
  external mechanism such as vectoring is used for yaw control
*/
void AP_MotorsMatrix::disable_yaw_torque(void)
{
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _yaw_factor[i] = 0;
    }
}

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
// examples can pull values direct
float AP_MotorsMatrix::get_thrust_rpyt_out(uint8_t i) const
{
    if (i < AP_MOTORS_MAX_NUM_MOTORS) {
        return _thrust_rpyt_out[i];
    }
    return 0.0;
}

bool AP_MotorsMatrix::get_factors(uint8_t i, float& roll, float& pitch, float& yaw, float& throttle, uint8_t& testing_order) const
{
    if ((i < AP_MOTORS_MAX_NUM_MOTORS) && motor_enabled[i]) {
        roll          = _roll_factor[i];
        pitch         = _pitch_factor[i];
        yaw           = _yaw_factor[i];
        throttle      = _throttle_factor[i];
        testing_order = _test_order[i];
        return true;
    }
    return false;
}
#endif

// singleton instance
AP_MotorsMatrix* AP_MotorsMatrix::_singleton;
