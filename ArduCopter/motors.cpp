#include "Copter.h"

#define ARM_DELAY          20  // 武装（解锁）延迟，以10赫兹频率计算，相当于2秒
#define DISARM_DELAY       20  // 解除武装（锁定）延迟，以10赫兹频率计算，相当于2秒
#define AUTO_TRIM_DELAY    100 // 自动校准延迟，以10赫兹频率计算，相当于10秒
#define LOST_VEHICLE_DELAY 10  // 载具丢失警报延迟，以10赫兹频率计算，相当于1秒

static uint32_t auto_disarm_begin;

// arm_motors_check - 检查激活或解除激活直升机的飞行器输入
// 每秒调用10次
void Copter::arm_motors_check()
{
    static int16_t arming_counter; // 静态变量，用于跟踪激活计数

    // 检查是否允许使用方向舵进行激活/解除激活
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        return; // 如果禁用了方向舵激活/解除激活，则返回
    }

#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        // 玩具模式下不使用摇杆激活
        return;
    }
#endif

    // 确保油门关闭
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t yaw_in  = channel_yaw->get_control_in();  // 获取方向舵输入值
    int16_t roll_in = channel_roll->get_control_in(); // 获取方向舵输入值

    // 向最右
    if (yaw_in > 4000 && roll_in > 4000) {

        // 增加激活计数，最多比自动校准计数器多1
        if (arming_counter <= AUTO_TRIM_DELAY) {
            arming_counter++;
        }

        // 激活电机并配置为飞行
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // 如果激活失败，则重置激活计数
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // 激活电机并配置为飞行
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && flightmode->mode_number() == Mode::Number::STABILIZE) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim 启动");
            auto_trim_counter = 250;
            auto_trim_started = false;
            // 确保自动解除激活不会立即触发
            auto_disarm_begin = millis();
        }

        // 向最左，且启用了方向舵激活/解除激活
    } else if ((yaw_in < -4000 && roll_in < -4000) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // 增加计数，最多比解除激活延迟多1
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // 解除激活电机
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

        // 方向舵处于中心位置，重置激活计数
    } else {
        arming_counter = 0;
    }
}

// auto_disarm_check - 如果直升机在手动模式下油门低并且静止在地面上至少15秒，则上锁直升机电机
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms         = millis();                                       // 获取当前时间（毫秒）
    uint32_t disarm_delay_ms = 1000 * constrain_int16(g.disarm_delay, 0, 127); // 获取上锁延迟时间（毫秒）

    // 如果电机已上锁，或者已禁用自动上锁，或者处于THROW模式下，立即退出
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // 如果旋翼仍在旋转，不要启动自动上锁
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // 总是允许自动上锁，如果使用了联锁开关或电机已经紧急停止
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
        // 如果使用了油门联锁开关或紧急停止，延迟时间更短，因为电机不旋转，难以确定是否已上锁
#if FRAME_CONFIG != HELI_FRAME
        disarm_delay_ms /= 2;
#endif
    } else {
        // 检查油门是否低，如果是则继续检查上锁条件
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low            = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // 重置计时器
            auto_disarm_begin = tnow_ms;
        }
    }

    // 当计时器超过上锁延迟时间时，执行上锁
    if ((tnow_ms - auto_disarm_begin) >= disarm_delay_ms) {
        arming.disarm(AP_Arming::Method::DISARMDELAY);
        auto_disarm_begin = tnow_ms;
    }
}

// motors_output - 将输出发送到电机库，该库将进行调整并将信号发送给电子速度控制器（ESCs）和伺服电机
void Copter::motors_output()
{
#if ADVANCED_FAILSAFE == ENABLED
    // 高级故障保护：用于允许故障保护模块有意终止载具的情况。
    // 仅在极端情况下使用，以满足特殊规则（例如OBC规则）。
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return; // 如果未通过着陆终止，立即退出
        }
        // 仍需继续执行电机输出以完成着陆
    }
#endif

    // 更新解锁延迟状态，用于确保飞机安全地解锁
    if (ap.in_arming_delay && (!motors->armed() || millis() - arm_time_ms > ARMING_DELAY_SEC * 1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // 计算并输出伺服通道的PWM信号
    SRV_Channels::calc_pwm();

    // 现在锁定通道，以便所有通道输出同时进行
    SRV_Channels::cork();

    // 更新所有辅助通道的输出，用于手动通行（例如手动控制伺服）
    SRV_Channels::output_ch_all();

    // 更新电机联锁状态，确保电机可以安全运行
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        AP::logger().Write_Event(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        AP::logger().Write_Event(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    if (ap.motor_test) {
        // 执行电机测试，用于验证电机性能
        motor_test_output();
    } else {
        // 将输出信号发送到电机，以实现飞行控制
        flightmode->output_to_motors();
    }

    // 推送所有通道输出
    SRV_Channels::push();
}

// lost_vehicle_check - 检查飞手遥杆输入以触发丢失载具警报
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter; // 静态计数器，用于触发声音警报

    // 如果设置了辅助开关用于载具丢失警报，禁用此功能以避免干扰
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // 确保油门关闭、电机未解锁、俯仰和横滚通道遥杆位置在最大值。
    // 注意：rc1=横滚通道、rc2=俯仰通道
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE, "Locate Copter alarm"); // 发送警告消息
            }
        } else {
            soundalarm_counter++; // 增加计数以触发警报
        }
    } else {
        soundalarm_counter = 0; // 重置计数
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}