#include "Copter.h"

#if MODE_POSHOLD_ENABLED == ENABLED

/*
 * PosHold飞行模式的初始化和运行调用
 *     PosHold试图通过混合飞行员输入和定点控制器来改进普通的定点飞行模式
 */

# define POSHOLD_SPEED_0                       10 // 低于此速度时，始终可以安全切换到定点飞行模式

// 400Hz循环更新速率
# define POSHOLD_BRAKE_TIME_ESTIMATE_MAX       (600 * 4) // 在切换到定点飞行模式之前，刹车将应用的最大周期数
# define POSHOLD_BRAKE_TO_LOITER_TIMER         (150 * 4) // 从刹车模式切换到定点飞行模式的周期数。必须低于POSHOLD_LOITER_STAB_TIMER
# define POSHOLD_WIND_COMP_START_TIMER         (150 * 4) // 在启用定点飞行模式后，开始风补偿更新的周期数
# define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER (50 * 4)  // 设置从100到200，混合定点飞行模式和手动控制命令的百分之一秒数，以实现平滑过渡。
# define POSHOLD_SMOOTH_RATE_FACTOR            0.0125f   // 应用于飞行员的滚转/俯仰输入的滤波器，当刹车率也较低时，较低的数值会导致滚转/俯仰返回零的速度较慢。
# define POSHOLD_WIND_COMP_TIMER_10HZ          40        // 用于减小风补偿到10Hz的计数器值
# define LOOP_RATE_FACTOR                      4         // 用于适应PosHold参数到循环速率

# define TC_WIND_COMP                          0.0025f // poshold_update_wind_comp_estimate()的风补偿的时间常数

// 与主循环速率无关的定义
# define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE    1800    // 最大角度要求（以百分之一度为单位），在此之后将应用平滑杆释放效果
# define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX  10      // 风补偿估算将仅在速度在或低于此速度时运行（单位：cm/s）
# define POSHOLD_WIND_COMP_LEAN_PCT_MAX        0.6666f // 风补偿不超过2/3的最大角度，以确保飞行员始终可以覆盖

// 定义ModePosHold类的init方法，此方法旨在初始化PosHold（位置保持）模式。
// 如果ignore_checks为true，某些检查可能会被忽略。
bool ModePosHold::init(bool ignore_checks)
{
    // 设置垂直方向上的最大速度和加速度，这些参数通常基于飞行员的配置。
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // 如果垂直位置控制器当前未激活，初始化垂直位置控制器。
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // 初始化飞行员的滚转和俯仰角，这里是设为0，可能是重置飞行控制的起始参考点。
    pilot_roll  = 0.0f;
    pilot_pitch = 0.0f;

    // 计算刹车增益，这影响了无人机在PosHold模式下减速的敏捷性。
    // 这是基于poshold_brake_rate的配置，并通过一些计算来设置。
    brake.gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) * 0.01f;

    // 检查无人机是否已着陆
    if (copter.ap.land_complete) {
        // 如果已着陆，则滚转和俯仰控制模式设置为“悬停”（LOITER），
        // 无人机将在当前位置保持稳定。
        roll_mode  = RPMode::LOITER;
        pitch_mode = RPMode::LOITER;
    } else {
        // 如果未着陆，设置控制模式为“飞行员覆盖”（PILOT_OVERRIDE），
        // 允许飞行员的输入有更直接的影响，防止自动控制造成突然的动作。
        roll_mode  = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
    }

    // 重置飞行员的期望加速度输入，并初始化悬停导航的目标点，
    // 以准备无人机在悬停模式下保持位置。
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // 初始化风补偿估计，这是每次进入PosHold模式时都需要做的，
    // 以考虑风的影响。
    init_wind_comp_estimate();

    // 返回true表示初始化成功完成。
    return true;
}

// poshold_run - 运行PosHold控制器
// 应该以100Hz或更高的频率调用
void ModePosHold::run()
{
    // 定义一些控制器和飞行员控制的混合系数
    float           controller_to_pilot_roll_mix;  // 控制器和飞行员控制的混合系数，0表示完全由上一次控制器控制，1表示完全由飞行员控制
    float           controller_to_pilot_pitch_mix; // 控制器和飞行员控制的混合系数，0表示完全由上一次控制器控制，1表示完全由飞行员控制
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();

    // 设置垂直速度和加速度限制
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // 应用简单模式的转换到飞行员输入
    update_simple_mode();

    // 将飞行员输入转换为倾斜角度
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // 获取飞行员期望的偏航速率
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // 获取飞行员期望的爬升率（用于定高模式和起飞）
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate       = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // 如果可能降落，松开定点模式的目标
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // PosHold状态机确定
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate);

    // 状态机
    switch (poshold_state) {

            // 状态1：电机已停止
        case AltHold_MotorStopped:
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->reset_yaw_target_and_rate(false);
            pos_control->relax_z_controller(0.0f); // 强制油门输出衰减到零
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();

            // 将poshold状态设置为飞行员覆盖
            roll_mode  = RPMode::PILOT_OVERRIDE;
            pitch_mode = RPMode::PILOT_OVERRIDE;

            // 初始化风补偿估计
            init_wind_comp_estimate();
            break;

        // 状态2：着陆，地面空闲
        case AltHold_Landed_Ground_Idle:
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();
            attitude_control->reset_yaw_target_and_rate();
            init_wind_comp_estimate();
            FALLTHROUGH;

        // 状态3：着陆前起飞
        case AltHold_Landed_Pre_Takeoff:
            attitude_control->reset_rate_controller_I_terms_smoothly();
            pos_control->relax_z_controller(0.0f); // 强制油门输出衰减到零

            // 将poshold状态设置为飞行员覆盖
            roll_mode  = RPMode::PILOT_OVERRIDE;
            pitch_mode = RPMode::PILOT_OVERRIDE;
            break;

        // 状态4：起飞
        case AltHold_Takeoff:
            // 初始化起飞
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            }

            // 获取避障调整的爬升率
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // 设置位置控制器目标，根据飞行员输入进行调整
            takeoff.do_pilot_takeoff(target_climb_rate);

            // 初始化并更新定点控制，尽管飞行员正在控制倾斜角
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();

            // 将poshold状态设置为飞行员覆盖
            roll_mode  = RPMode::PILOT_OVERRIDE;
            pitch_mode = RPMode::PILOT_OVERRIDE;
            break;

        // 状态5：飞行中
        case AltHold_Flying:
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            // 获取避障调整的爬升率
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // 基于表面测量更新垂直偏移
            copter.surface_tracking.update_surface_offset();

            // 将期望的爬升率发送给位置控制器
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            break;
    }

    // PosHold具体行为以计算期望的横滚和俯仰角度
    // 将惯性导航地坐标系的速度转换为机体坐标系
    float vel_fw    = vel.x * ahrs.cos_yaw() + vel.y * ahrs.sin_yaw();
    float vel_right = -vel.x * ahrs.sin_yaw() + vel.y * ahrs.cos_yaw();

    // 如果不在LOITER模式下，检索与当前偏航相关的最新风补偿倾斜角
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles(wind_comp_roll, wind_comp_pitch);
    }

    // 横滚状态机
    // 每个状态（也称为模式）负责：
    // 1. 处理飞行员输入
    // 2. 计算发送给姿态控制器的最终横滚输出
    // 3. 检查状态（模式）是否应更改，如果是，执行所需的初始化操作
    switch (roll_mode) {

        case RPMode::PILOT_OVERRIDE:
            // 使用最新的遥控输入更新飞行员期望的横滚角度
            // 这会滤除输入，使其不会比制动速率更快地返回零
            update_pilot_lean_angle(pilot_roll, target_roll);

            // 如果没有飞行员输入，切换到制动模式以供下一次迭代使用
            if (is_zero(target_roll) && (fabsf(pilot_roll) < 2 * g.poshold_brake_rate)) {
                // 初始化制动模式
                roll_mode               = RPMode::BRAKE;                   // 设置制动横滚模式
                brake.roll              = 0.0f;                            // 初始化制动角度为零
                brake.angle_max_roll    = 0.0f;                            // 重置制动角度最大值，以便检测车辆何时开始变平
                brake.timeout_roll      = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // 制动将应用的周期数，在制动模式下更新
                brake.time_updated_roll = false;                           // 标志用于重新估计制动时间
            }

            // 最终的横滚角应为飞行员输入加上风补偿
            roll = pilot_roll + wind_comp_roll;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // 计算制动横滚角度以抵消速度
            update_brake_angle_from_velocity(brake.roll, vel_right);

            // 更新制动时间估算
            if (!brake.time_updated_roll) {
                // 检查制动角度是否在增加
                if (fabsf(brake.roll) >= brake.angle_max_roll) {
                    brake.angle_max_roll = fabsf(brake.roll);
                } else {
                    // 制动角度开始减小，因此重新估算制动时间
                    brake.timeout_roll      = 1 + (uint16_t)(LOOP_RATE_FACTOR * 15L * (int32_t)(fabsf(brake.roll)) / (10L * (int32_t)g.poshold_brake_rate)); // 制动时间的调整因子
                    brake.time_updated_roll = true;
                }
            }

            // 如果速度非常低，将制动时间减少到0.5秒
            if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (brake.timeout_roll > 50 * LOOP_RATE_FACTOR)) {
                brake.timeout_roll = 50 * LOOP_RATE_FACTOR;
            }

            // 减少制动计时器
            if (brake.timeout_roll > 0) {
                brake.timeout_roll--;
            } else {
                // 表示准备转入Loiter模式
                // 只有当roll_mode和pitch_mode都更改为RPMode::BRAKE_READY_TO_LOITER时，才会实际进入Loiter
                // 触发进入Loiter的逻辑在roll和pitch模式切换语句下处理
                roll_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // 最终的横滚角是制动角度加上风补偿角度
            roll = brake.roll + wind_comp_roll;

            // 检查飞行员输入
            if (!is_zero(target_roll)) {
                // 初始化切换到飞行员覆盖模式
                roll_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // 这些模式是合并的横滚-俯仰模式，将在下面处理
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // 使用最新的遥控输入更新飞行员期望的横滚角度
            // 这会滤除输入，使其不会比制动速率更快地返回零
            update_pilot_lean_angle(pilot_roll, target_roll);

            // 倒计时控制器到飞行员的计时器
            if (controller_to_pilot_timer_roll > 0) {
                controller_to_pilot_timer_roll--;
            } else {
                // 当计时器耗尽时，切换到下一次迭代的完全飞行员覆盖
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // 计算控制器到飞行员的混合比例
            controller_to_pilot_roll_mix = (float)controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // 混合最终的Loiter横滚角和飞行员期望横滚角
            roll = mix_controls(controller_to_pilot_roll_mix, controller_final_roll, pilot_roll + wind_comp_roll);
            break;
    }

    // 俯仰状态机
    // 每个状态（也称为模式）负责：
    // 1. 处理飞行员输入
    // 2. 计算发送给姿态控制器的最终俯仰输出
    // 3. 检查状态（模式）是否应更改，如果是，执行所需的初始化操作
    switch (pitch_mode) {

        case RPMode::PILOT_OVERRIDE:
            // 使用最新的遥控输入更新飞行员期望的俯仰角度
            // 这会滤除输入，使其不会比制动速率更快地返回零
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // 如果没有飞行员输入，切换到制动模式以供下一次迭代使用
            if (is_zero(target_pitch) && (fabsf(pilot_pitch) < 2 * g.poshold_brake_rate)) {
                // 初始化制动模式
                pitch_mode               = RPMode::BRAKE;                   // 设置制动俯仰模式
                brake.pitch              = 0.0f;                            // 初始化制动角度为零
                brake.angle_max_pitch    = 0.0f;                            // 重置制动角度最大值，以便检测车辆何时开始变平
                brake.timeout_pitch      = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // 制动将应用的周期数，在制动模式下更新
                brake.time_updated_pitch = false;                           // 标志用于重新估计制动时间
            }

            // 最终的俯仰角应为飞行员输入加上风补偿
            pitch = pilot_pitch + wind_comp_pitch;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // 计算制动俯仰角度以抵消速度
            update_brake_angle_from_velocity(brake.pitch, -vel_fw);

            // 更新制动时间估算
            if (!brake.time_updated_pitch) {
                // 检查制动角度是否在增加
                if (fabsf(brake.pitch) >= brake.angle_max_pitch) {
                    brake.angle_max_pitch = fabsf(brake.pitch);
                } else {
                    // 制动角度开始减小，因此重新估算制动时间
                    brake.timeout_pitch      = 1 + (uint16_t)(LOOP_RATE_FACTOR * 15L * (int32_t)(fabsf(brake.pitch)) / (10L * (int32_t)g.poshold_brake_rate)); // 制动时间的调整因子
                    brake.time_updated_pitch = true;
                }
            }

            // 如果速度非常低，将制动时间减少到0.5秒
            if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (brake.timeout_pitch > 50 * LOOP_RATE_FACTOR)) {
                brake.timeout_pitch = 50 * LOOP_RATE_FACTOR;
            }

            // 减少制动计时器
            if (brake.timeout_pitch > 0) {
                brake.timeout_pitch--;
            } else {
                // 表示准备转入Loiter模式
                // 只有当pitch_mode和pitch_mode都更改为RPMode::BRAKE_READY_TO_LOITER时，才会实际进入Loiter
                // 触发进入Loiter的逻辑在pitch和pitch模式切换语句下处理
                pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // 最终的俯仰角是制动角度加上风补偿角度
            pitch = brake.pitch + wind_comp_pitch;

            // 检查飞行员输入
            if (!is_zero(target_pitch)) {
                // 初始化切换到飞行员覆盖模式
                pitch_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // 这些模式是合并的俯仰-俯仰模式，将在下面处理
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // 使用最新的遥控输入更新飞行员期望的俯仰角度
            // 这会滤除输入，使其不会比制动速率更快地返回零
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // 倒计时控制器到飞行员的计时器
            if (controller_to_pilot_timer_pitch > 0) {
                controller_to_pilot_timer_pitch--;
            } else {
                // 当计时器耗尽时，切换到下一次迭代的完全飞行员覆盖
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // 计算控制器到飞行员的混合比例
            controller_to_pilot_pitch_mix = (float)controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // 混合最终的Loiter俯仰角和飞行员期望俯仰角
            pitch = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch, pilot_pitch + wind_comp_pitch);
            break;
    }

    // 共享的Roll和Pitch状态（RPMode::BRAKE_TO_LOITER和RPMode::LOITER）

    // 当Roll和Pitch都准备就绪时，切换到LOITER模式
    if (roll_mode == RPMode::BRAKE_READY_TO_LOITER && pitch_mode == RPMode::BRAKE_READY_TO_LOITER) {
        roll_mode             = RPMode::BRAKE_TO_LOITER;
        pitch_mode            = RPMode::BRAKE_TO_LOITER;
        brake.to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
        // 初始化Loiter控制器
        loiter_nav->init_target(inertial_nav.get_position_xy_cm());
        // 设置延迟开始风补偿估算更新
        wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
    }

    // 在BRAKE_TO_LOITER或LOITER模式下，roll_mode用作组合的Roll+Pitch模式
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // 强制Pitch模式与Roll模式相同，以保持一致性（实际上在这些状态下不使用Pitch模式）
        pitch_mode = roll_mode;

        // 处理组合的Roll+Pitch模式
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER: {
                // 减少制动到Loiter的计时器
                if (brake.to_loiter_timer > 0) {
                    brake.to_loiter_timer--;
                } else {
                    // 在下一次迭代中进入完全Loiter
                    roll_mode  = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // 制动和Loiter控制的混合。 0 = 完全制动控制，1 = 完全Loiter控制
                const float brake_to_loiter_mix = (float)brake.to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                // 计算抵消速度的制动Roll和Pitch角度
                update_brake_angle_from_velocity(brake.roll, vel_right);
                update_brake_angle_from_velocity(brake.pitch, -vel_fw);

                // 运行Loiter控制器
                loiter_nav->update(false);

                // 通过混合Loiter和制动控制来计算最终的Roll和Pitch输出
                roll  = mix_controls(brake_to_loiter_mix, brake.roll + wind_comp_roll, loiter_nav->get_roll());
                pitch = mix_controls(brake_to_loiter_mix, brake.pitch + wind_comp_pitch, loiter_nav->get_pitch());

                // 检查飞行员输入
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // 如果有Roll输入，切换到Roll的飞行员覆盖模式
                    if (!is_zero(target_roll)) {
                        // 初始化切换到飞行员覆盖模式
                        roll_controller_to_pilot_override();
                        // 将Pitch模式切换到制动（但随时准备回到Loiter）
                        // 这里无需重置brake.pitch，因为自上次计算brake.pitch以来未更新wind_comp
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // 如果有Pitch输入，切换到Pitch的飞行员覆盖模式
                    if (!is_zero(target_pitch)) {
                        // 初始化切换到飞行员覆盖模式
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll)) {
                            // 切换Roll模式到制动（但随时准备回到Loiter）
                            // 这里无需重置brake.roll，因为自上次计算brake.roll以来未更新wind_comp
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                        }
                    }
                }
                break;
            }
            case RPMode::LOITER:
                // 运行Loiter控制器
                loiter_nav->update(false);

                // 基于Loiter控制器输出设置Roll角度
                roll  = loiter_nav->get_roll();
                pitch = loiter_nav->get_pitch();

                // 更新风补偿估算
                update_wind_comp_estimate();

                // 检查飞行员输入
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // 如果有Roll输入，切换到Roll的飞行员覆盖模式
                    if (!is_zero(target_roll)) {
                        // 初始化切换到飞行员覆盖模式
                        roll_controller_to_pilot_override();
                        // 切换Pitch模式到制动（但随时准备回到Loiter）
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // 重置brake.pitch，因为现在的wind_comp不同，应该提供整个先前Loiter角度的补偿
                        brake.pitch = 0.0f;
                    }
                    // 如果有Pitch输入，切换到Pitch的飞行员覆盖模式
                    if (!is_zero(target_pitch)) {
                        // 初始化切换到飞行员覆盖模式
                        pitch_controller_to_pilot_override();
                        // 如果没有覆盖Roll，将Roll模式切换到制动（但随时准备回到Loiter）
                        if (is_zero(target_roll)) {
                            roll_mode  = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll = 0.0f;
                        }
                        // 如果没有覆盖Roll，将Roll模式切换到制动（但随时准备回到Loiter）
                    }
                }
                break;

            default:
                // 对于未组合的Roll和Pitch模式不执行任何操作
                break;
        }
    }

    // 限制目标俯仰角和横滚角
    float angle_max = copter.aparm.angle_max;
    roll            = constrain_float(roll, -angle_max, angle_max);
    pitch           = constrain_float(pitch, -angle_max, angle_max);

    // 调用姿态控制器，传入横滚角、俯仰角以及目标偏航角速率
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);

    // 运行垂直位置控制器并设置输出油门
    pos_control->update_z_controller();
}

// poshold_update_pilot_lean_angle - 使用最新接收的原始输入更新飞行员的滤波倾斜角度
void ModePosHold::update_pilot_lean_angle(float& lean_angle_filtered, float& lean_angle_raw)
{
    // 如果原始输入较大或立即将飞行器的倾斜角度反向，则将滤波后的角度设置为新的原始角度
    if ((lean_angle_filtered > 0 && lean_angle_raw < 0) || (lean_angle_filtered < 0 && lean_angle_raw > 0) || (fabsf(lean_angle_raw) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw 必须将 lean_angle_filtered 向零拉动，平滑减小
        if (lean_angle_filtered > 0) {
            // 以5%或刹车速率中较快的速度减小滤波后的倾斜角度
            lean_angle_filtered -= MAX(lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1.0f, g.poshold_brake_rate / (float)LOOP_RATE_FACTOR));
            // 不允许滤波后的角度低于飞行员的输入倾斜角度
            // 上面的代码将滤波后的角度向下拉，下面的代码充当防护
            lean_angle_filtered = MAX(lean_angle_filtered, lean_angle_raw);
        } else {
            lean_angle_filtered += MAX(-lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1.0f, g.poshold_brake_rate / (float)LOOP_RATE_FACTOR));
            lean_angle_filtered = MIN(lean_angle_filtered, lean_angle_raw);
        }
    }
}

// mix_controls - 根据混合比例混合两个控制信号
//  mix_ratio为1表示完全使用first_control，0表示完全使用second_control，0.5表示均匀混合
float ModePosHold::mix_controls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);                     // 确保混合比例在0到1之间
    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control; // 使用混合比例将两个控制信号混合
}

// update_brake_angle_from_velocity - 根据飞行器的速度和刹车增益更新刹车角度
//  刹车角度会受到wpnav.poshold_brake_rate的调节，并受限于wpnav.poshold_braking_angle_max
//  假定速度与倾斜角度方向一致，因此对于俯仰运动，应提供向后的速度（即负的前进速度）
void ModePosHold::update_brake_angle_from_velocity(float& brake_angle, float velocity)
{
    float lean_angle;
    float brake_rate = g.poshold_brake_rate;

    brake_rate /= (float)LOOP_RATE_FACTOR; // 将刹车速率除以循环速率因子
    if (brake_rate <= 1.0f) {
        brake_rate = 1.0f; // 如果刹车速率小于等于1，将其设置为1
    }

    // 计算仅基于速度的倾斜角度
    if (velocity >= 0) {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (velocity + 60.0f));
    } else {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (-velocity + 60.0f));
    }

    // 不允许倾斜角度与刹车角度相差太大
    brake_angle = constrain_float(lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // 限制最终的刹车角度
    brake_angle = constrain_float(brake_angle, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}

// 将风补偿估算初始化为零
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ef.zero();    // 将风补偿估算初始化为零向量
    wind_comp_timer = 0;    // 将风补偿计时器初始化为零
    wind_comp_roll  = 0.0f; // 将风补偿滚动角初始化为零
    wind_comp_pitch = 0.0f; // 将风补偿俯仰角初始化为零
}

// update_wind_comp_estimate - 更新风补偿估算
//  当悬停模式启用时，应以最大循环速率调用此函数
void ModePosHold::update_wind_comp_estimate()
{
    // 检查风估算是否延迟启动
    if (wind_comp_start_timer > 0) {
        wind_comp_start_timer--;
        return;
    }

    // 检查水平速度是否较低
    if (inertial_nav.get_speed_xy_cms() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // 获取位置控制器的加速度目标
    const Vector3f& accel_target = pos_control->get_accel_target_cmss();

    // 在地球坐标系下更新风补偿的倾斜角度
    if (is_zero(wind_comp_ef.x)) {
        // 如果风补偿未初始化，则立即将其设置为位置控制器在北向的期望加速度
        wind_comp_ef.x = accel_target.x;
    } else {
        // 低通滤波位置控制器的倾斜角度输出
        wind_comp_ef.x = (1.0f - TC_WIND_COMP) * wind_comp_ef.x + TC_WIND_COMP * accel_target.x;
    }
    if (is_zero(wind_comp_ef.y)) {
        // 如果风补偿未初始化，则立即将其设置为位置控制器在北向的期望加速度
        wind_comp_ef.y = accel_target.y;
    } else {
        // 低通滤波位置控制器的倾斜角度输出
        wind_comp_ef.y = (1.0f - TC_WIND_COMP) * wind_comp_ef.y + TC_WIND_COMP * accel_target.y;
    }

    // 限制加速度
    const float accel_lim_cmss   = tanf(radians(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max * 0.01f)) * 981.0f;
    const float wind_comp_ef_len = wind_comp_ef.length();
    if (!is_zero(accel_lim_cmss) && (wind_comp_ef_len > accel_lim_cmss)) {
        wind_comp_ef *= accel_lim_cmss / wind_comp_ef_len;
    }
}

// get_wind_comp_lean_angles - 获取风补偿的倾斜角度，包括滚动和俯仰角度
//  应以最大循环速率调用
void ModePosHold::get_wind_comp_lean_angles(float& roll_angle, float& pitch_angle)
{
    // 降低速率为10hz
    wind_comp_timer++;
    if (wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    wind_comp_timer = 0;

    // 将地球坐标系下的期望加速度转换为机体坐标系中的滚动和俯仰倾斜角度
    roll_angle  = atanf((-wind_comp_ef.x * ahrs.sin_yaw() + wind_comp_ef.y * ahrs.cos_yaw()) / (GRAVITY_MSS * 100)) * (18000.0f / M_PI);
    pitch_angle = atanf(-(wind_comp_ef.x * ahrs.cos_yaw() + wind_comp_ef.y * ahrs.sin_yaw()) / (GRAVITY_MSS * 100)) * (18000.0f / M_PI);
}

// roll_controller_to_pilot_override - 初始化从控制器子模式（刹车或悬停）切换到滚动轴上的飞行员覆盖
void ModePosHold::roll_controller_to_pilot_override()
{
    roll_mode                      = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // 将pilot_roll初始化为0，风补偿将更新以进行补偿，并且在下一次迭代中，poshold_update_pilot_lean_angle函数将不会平滑此过渡，所以0是正确的值
    pilot_roll = 0.0f;
    // 存储最终的控制器输出以与飞行员输入混合
    controller_final_roll = roll;
}

// pitch_controller_to_pilot_override - 初始化从控制器子模式（刹车或悬停）切换到俯仰轴上的飞行员覆盖
void ModePosHold::pitch_controller_to_pilot_override()
{
    pitch_mode                      = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // 将pilot_pitch初始化为0，风补偿将更新以进行补偿，并且update_pilot_lean_angle函数将不会平滑此过渡，所以0是正确的值
    pilot_pitch = 0.0f;
    // 存储最终的悬停控制器输出以与飞行员输入混合
    controller_final_pitch = pitch;
}

#endif
