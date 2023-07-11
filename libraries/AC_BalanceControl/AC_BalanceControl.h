#pragma once

/// @file    AC_AttitudeControl_Multi.h
/// @brief   ArduCopter attitude control library

#include <AC_PID/AC_PI.h>
#include <AC_PID/AC_PID.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Param/AP_Param.h>
#include <AP_RMUART/AP_RMUART.h>

// default rate controller PID gains

#define AC_BALANCE_BALANCE_P 0.5
#define AC_BALANCE_BALANCE_D 0.1f

#define AC_BALANCE_VELOCITY_P    0.5f
#define AC_BALANCE_VELOCITY_I    0.05f
#define AC_BALANCE_VELOCITY_IMAX 1.0f

#define AC_BALANCE_TURN_P    0.3f
#define AC_BALANCE_TURN_D    0.1f

#define AC_BALANCE_ZERO_ANGLE 0.0f

class AC_BalanceControl {
public:
    AC_BalanceControl(AP_Motors& motors, AP_AHRS_View& ahrs, AP_RMUART& _rmuart);

    // empty destructor to suppress compiler warning
    virtual ~AC_BalanceControl() { }

    float Balance(float Angle, float Gyro);
    float Velocity(int encoder_left, int encoder_right);
    float Turn(float gyro);

    void balance_all_control(void);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    AP_Motors& _motors;
    const AP_AHRS_View& _ahrs;
    AP_RMUART& _rmuart;

    enum moveFlag {
        none = 0,
        moveFront = 1,
        moveBack = 2,
        moveRight = 3,
        moveLeft = 4,
    };

protected:
    AP_Float _balance_bal_p;
    AP_Float _balance_bal_d;

    AP_Float _balance_velocity_p;
    AP_Float _balance_velocity_i;
    AP_Float _balance_velocity_imax;

    AP_Float _balance_turn_p;
    AP_Float _balance_turn_d;

    AP_Float _zero_angle;      // 机械零值

    float Encoder_bias_filter; // 一阶低通滤波器

    uint8_t _moveflag;

    float motor_Left,motor_Right;
    float control_balance, control_velocity, control_turn;

    

};
