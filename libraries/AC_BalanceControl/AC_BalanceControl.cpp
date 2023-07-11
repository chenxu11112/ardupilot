#include "AC_BalanceControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AC_BalanceControl::var_info[] = {
    AP_GROUPINFO("BAL_BAL_P", 0, AC_BalanceControl, _balance_bal_p, AC_BALANCE_BALANCE_P),

    AP_GROUPINFO("BAL_BAL_D", 1, AC_BalanceControl, _balance_bal_d, AC_BALANCE_BALANCE_D),

    AP_GROUPINFO("BAL_VEL_P", 2, AC_BalanceControl, _balance_velocity_p, AC_BALANCE_VELOCITY_P),

    AP_GROUPINFO("BAL_VEL_I", 3, AC_BalanceControl, _balance_velocity_i, AC_BALANCE_VELOCITY_I),

    AP_GROUPINFO("BAL_VEL_IMAX", 4, AC_BalanceControl, _balance_velocity_imax, AC_BALANCE_VELOCITY_IMAX),

    AP_GROUPINFO("BAL_TURN_P", 5, AC_BalanceControl, _balance_bal_p, AC_BALANCE_BALANCE_P),

    AP_GROUPINFO("BAL_TURN_D", 6, AC_BalanceControl, _balance_bal_d, AC_BALANCE_BALANCE_D),

    AP_GROUPINFO("BAL_ZERO", 7, AC_BalanceControl, _zero_angle, ZERO_ANGLE),

    AP_GROUPEND
};

AC_BalanceControl::AC_BalanceControl(AP_Motors& motors, AP_AHRS_View& ahrs, AP_RMUART& rmuart)
    : _motors(motors)
    , _ahrs(ahrs)
    , _rmuart(rmuart)
{
    AP_Param::setup_object_defaults(this, var_info);

    Encoder_bias_filter = 0.84f;
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle:angle；Gyro：angular velocity
Output  : balance：Vertical control PWM
函数功能：直立PD控制
入口参数：Angle:角度；Gyro：角速度
返回  值：balance：直立控制PWM
**************************************************************************/
float AC_BalanceControl::Balance(float Angle, float Gyro)
{
    float Angle_bias, Gyro_bias;
    float balance;
    Angle_bias = _zero_angle - Angle;                                   // 求出平衡的角度中值 和机械相关
    Gyro_bias = 0 - Gyro;
    balance = _balance_bal_p * Angle_bias + Gyro_bias * _balance_bal_d; // 计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    return balance;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
// 修改前进后退速度，请修改Target_Velocity，比如，改成60就比较慢了
float AC_BalanceControl::Velocity(int encoder_left, int encoder_right)
{
    static float velocity, Encoder_Least, Encoder_bias, Movement;
    static float Encoder_Integral, Target_Velocity = 0.1;

    //================遥控前进后退部分====================//
    if (_moveflag == moveFlag::moveFront) {
        Movement = Target_Velocity;  // 收到前进信号
    } else if (_moveflag == moveFlag::moveBack) {
        Movement = -Target_Velocity; // 收到后退信号
    } else {
        Movement = 0;
    }

    //================速度PI控制器=====================//
    Encoder_Least = 0 - (encoder_left + encoder_right);        // 获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和）

    Encoder_bias *= Encoder_bias_filter;                       // 一阶低通滤波器
    Encoder_bias += Encoder_Least * (1 - Encoder_bias_filter); // 一阶低通滤波器，减缓速度变化

    Encoder_Integral += Encoder_bias;                          // 积分出位移 积分时间：10ms
    Encoder_Integral = Encoder_Integral + Movement;            // 接收遥控器数据，控制前进后退

    if (Encoder_Integral > AC_BALANCE_VELOCITY_IMAX)
        Encoder_Integral = AC_BALANCE_VELOCITY_IMAX;                                        // 积分限幅
    if (Encoder_Integral < -AC_BALANCE_VELOCITY_IMAX)
        Encoder_Integral = -AC_BALANCE_VELOCITY_IMAX;                                       // 积分限幅

    velocity = Encoder_bias * _balance_velocity_p + Encoder_Integral * _balance_velocity_i; // 速度控制
    // if (Turn_Off(Angle_Balance, Voltage) == 1 || Flag_Stop == 1)
    //     Encoder_Integral = 0;                                                           // 电机关闭后清除积分
    return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
float AC_BalanceControl::Turn(float gyro)
{
    static float Turn_Target, turn, Turn_Amplitude = 0.2;
    float Kp = _balance_turn_p, Kd; // 修改转向速度，请修改Turn_Amplitude即可

    //===================遥控左右旋转部分=================//
    if (_moveflag == moveFlag::moveLeft)
        Turn_Target = -Turn_Amplitude;
    else if (_moveflag == moveFlag::moveRight)
        Turn_Target = Turn_Amplitude;
    else
        Turn_Target = 0;

    if (_moveflag == moveFlag::moveFront || _moveflag == moveFlag::moveBack)
        Kd = _balance_turn_d;
    else
        Kd = 0; // 转向的时候取消陀螺仪的纠正 有点模糊PID的思想

    //===================转向PD控制器=================//
    turn = Turn_Target * Kp + gyro * Kd; // 结合Z轴陀螺仪进行PD控制

    return turn;                         // 转向环PWM右转为正，左转为负
}

void AC_BalanceControl::balance_all_control(void)
{
    float control_balance, control_velocity, control_turn;
    float wheel_left, wheel_right;
    float angle_y, gyroy, gyro_z;

    _rmuart.getWheelSpeed(wheel_left, wheel_right);

    angle_y = _ahrs.pitch;
    gyroy = _ahrs.get_gyro_latest()[1];
    gyro_z = _ahrs.get_gyro_latest()[2];

    control_balance = Balance(angle_y, gyroy);            // 平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
    control_velocity = Velocity(wheel_left, wheel_right); // 速度环PID控制	记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
    control_turn = Turn(gyro_z);                          // 转向环PID控制

    // PWM值正数使小车前进，负数使小车后退
    Motor_Left = control_balance + control_velocity + control_turn;  // 计算左轮电机最终PWM
    Motor_Right = control_balance + control_velocity - control_turn; // 计算右轮电机最终PWM
}