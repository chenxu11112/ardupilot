#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
 # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif


#define AR_ATTCONTROL_PITCH_THR_P       2.0
#define AR_ATTCONTROL_PITCH_THR_I       0.1
#define AR_ATTCONTROL_PITCH_THR_D       0.02
#define AR_ATTCONTROL_PITCH_THR_IMAX    1.0f
#define AR_ATTCONTROL_PITCH_THR_FILT    10.0f
#define AR_ATTCONTROL_BAL_PITCH_FF      0.4f

#define AR_VELOCITY_THR_P        0.3
#define AR_VELOCITY_THR_I        0.01f
#define AR_VELOCITY_THR_D        0.001f
#define AAR_VELOCITY_THR_IMAX    1.0f
#define AR_VELOCITY_THR_FILT     10.0f
#define AR_VELOCITY_THR_PITCH_FF 0.4f

#define AR_ATTCONTROL_TIMEOUT_MS 200

#define ZERO_ANGLE 0.0f


class AC_AttitudeControl_Multi : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    float get_throttle_out_from_pitch(float pitch_max, bool motor_limit, float dt);


protected:

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll;
    AC_PID                _pid_rate_pitch;
    AC_PID                _pid_rate_yaw;

    // balancebot pitch control
    uint32_t _balance_last_ms = 0;  // system time that get_throttle_out_from_pitch was last called
    float _pitch_limit_low = 0;     // min desired pitch (in radians) used to protect against falling over
    float _pitch_limit_high = 0;    // max desired pitch (in radians) used to protect against falling over
    bool _pitch_limited = false;    // true if pitch was limited on last call to get_throttle_out_from_pitch

    AC_PID                _pitch_to_throttle_pid;// balancebot pitch controller
    AP_Float              _pitch_to_throttle_ff; // balancebot feed forward from current pitch angle
    AC_PID                _vel_to_pitch_pid;// balancebot pitch controller
    AP_Float              _vel_to_pitch_ff; // balancebot feed forward from current pitch angle

    AP_Float              _zero_angle; // 机械零值


    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
};
