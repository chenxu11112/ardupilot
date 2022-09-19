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

/*
 *       AP_MotorsMatrix_with_Tilt.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMatrix_with_Tilt.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define SERVO_OUTPUT_RANGE 4500

// init
void AP_MotorsMatrix_with_Tilt::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    // _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_motor1) || SRV_Channels::function_assigned(SRV_Channel::k_motor1);

    // throttle defaults to motor output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_1);

    // throttle defaults to motor output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_2);

    // throttle defaults to motor output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor3, CH_3);

    // throttle defaults to motor output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor4, CH_4);

    // throttle defaults to motor output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor5, CH_5);

    // throttle defaults to motor output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor6, CH_6);

    // throttle defaults to motor output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor7, CH_7);

    // throttle defaults to motor output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor8, CH_8);

    SRV_Channels::set_output_min_max(SRV_Channel::k_motor5, 600, 2400);
    SRV_Channels::set_output_min_max(SRV_Channel::k_motor6, 600, 2400);
    SRV_Channels::set_output_min_max(SRV_Channel::k_motor7, 600, 2400);
    SRV_Channels::set_output_min_max(SRV_Channel::k_motor8, 600, 2400);

    _mav_type = MAV_TYPE_OCTOROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_OCTA);
}

/// Constructor
AP_MotorsMatrix_with_Tilt::AP_MotorsMatrix_with_Tilt(uint16_t loop_rate, uint16_t speed_hz) : AP_MotorsMulticopter(loop_rate, speed_hz)
{
    // set_update_rate(speed_hz);
    set_update_rate(900);
}

// set update rate to motors - a value in hertz
void AP_MotorsMatrix_with_Tilt::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor5, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor6, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor7, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor8, speed_hz);
}

void AP_MotorsMatrix_with_Tilt::output_to_motors()
{
    if (!initialised_ok())
    {
        return;
    }

    switch (_spool_state)
    {
    case SpoolState::SHUT_DOWN:
        _actuator[0] = 0.0f;
        _actuator[1] = 0.0f;
        _actuator[2] = 0.0f;
        _actuator[3] = 0.0f;

        _tilt[0] = 0.0f;
        _tilt[1] = 0.0f;
        _tilt[2] = 0.0f;
        _tilt[3] = 0.0f;

        _external_min_throttle = 0.0;
        break;
    case SpoolState::GROUND_IDLE:
        set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
        set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
        set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
        set_actuator_with_slew(_actuator[3], actuator_spin_up_to_ground_idle());

        _tilt[0] = 0.0f;
        _tilt[1] = 0.0f;
        _tilt[2] = 0.0f;
        _tilt[3] = 0.0f;

        _external_min_throttle = 0.0;
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        set_actuator_with_slew(_actuator[0], thrust_to_actuator(_thrust[0]));
        set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust[1]));
        set_actuator_with_slew(_actuator[2], thrust_to_actuator(_thrust[2]));
        set_actuator_with_slew(_actuator[3], thrust_to_actuator(_thrust[3]));
        break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, output_to_pwm(_actuator[1]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, output_to_pwm(_actuator[2]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, output_to_pwm(_actuator[3]));

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor5, output_to_tilt(_tilt[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor6, output_to_tilt(_tilt[1]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor7, output_to_tilt(_tilt[2]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor8, output_to_tilt(_tilt[3]));
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsMatrix_with_Tilt::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan))
    {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan))
    {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// convert actuator output (0~1) range to pwm range
int16_t AP_MotorsMatrix_with_Tilt::output_to_tilt(float actuator)
{
    float pwm_output;
    if (_spool_state == SpoolState::SHUT_DOWN)
    {
        // in shutdown mode, use PWM 0 or minimum PWM
        pwm_output = 1500;
    }
    else
    {
        // in all other spool modes, covert to desired PWM
        pwm_output = 1500 + 1000 * actuator;
    }

    return pwm_output;
}

const float sqrt2 = sqrtf(2);
const float armz = 0.1f;

#define fx (force_body[0])
#define fz (force_body[2])
#define mx roll_thrust
#define my pitch_thrust
#define mz yaw_thrust
#include <stdio.h>
#include <stdarg.h>
// calculate outputs to the motors
void AP_MotorsMatrix_with_Tilt::output_armed_stabilizing()
{
    uint8_t i;

    float roll_thrust;     // roll thrust input value, +/- 1.0
    float pitch_thrust;    // pitch thrust input value, +/- 1.0
    float yaw_thrust;      // yaw thrust input value, +/- 1.0
    float throttle_thrust; // throttle thrust input value, 0.0 - 1.0
    float thrust_max;      // highest motor value
    float thrust_min;      // lowest motor value
    float thr_adj = 0.0f;  // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float forward_thrust;  // forward thrust input value, +/- 1.0

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;
    forward_thrust = get_forward() * throttle_thrust;

    const float max_boost_throttle = _throttle_avg_max * compensation_gain;

    // never boost above max, derived from throttle mix params
    const float min_throttle_out = MIN(_external_min_throttle, max_boost_throttle);
    const float max_throttle_out = _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above min and below current limited throttle
    if (throttle_thrust <= min_throttle_out)
    {
        throttle_thrust = min_throttle_out;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= max_throttle_out)
    {
        throttle_thrust = max_throttle_out;
        limit.throttle_upper = true;
    }

    if (roll_thrust >= 1.0)
    {
        // cannot split motor outputs by more than 1
        roll_thrust = 1;
        limit.roll = true;
    }

    if (pitch_thrust >= 1.0)
    {
        // cannot split motor outputs by more than 1
        pitch_thrust = 1;
        limit.pitch = true;
    }

    // printf("throttle_thrust=%f\r\n", throttle_thrust);

    //   mx*((4*sinrad)/9 - (cosrad*sqrt2)/9) - fz*(cosrad + armz*sinrad*sqrt2) - fx*(sinrad - armz*cosrad*sqrt2) + mz*((4*cosrad)/9 + (sinrad*sqrt2)/9) + my*sqrt2
    //                                                          mx*((4*cosrad)/9 + (sinrad*sqrt2)/9) - cosrad*fx - mz*((4*sinrad)/9 - (cosrad*sqrt2)/9) + fz*sinrad
    //   mx*((4*sinrad)/9 + (cosrad*sqrt2)/9) - fz*(cosrad - armz*sinrad*sqrt2) - fx*(sinrad + armz*cosrad*sqrt2) + mz*((4*cosrad)/9 - (sinrad*sqrt2)/9) - my*sqrt2
    //                                                          cosrad*fx - mx*((4*cosrad)/9 - (sinrad*sqrt2)/9) + mz*((4*sinrad)/9 + (cosrad*sqrt2)/9) - fz*sinrad
    //   my*sqrt2 - fz*(cosrad + armz*sinrad*sqrt2) - mx*((4*sinrad)/9 - (cosrad*sqrt2)/9) - mz*((4*cosrad)/9 + (sinrad*sqrt2)/9) - fx*(sinrad - armz*cosrad*sqrt2)
    //                                                          cosrad*fx + mx*((4*cosrad)/9 + (sinrad*sqrt2)/9) - mz*((4*sinrad)/9 - (cosrad*sqrt2)/9) - fz*sinrad
    // - fx*(sinrad + armz*cosrad*sqrt2) - fz*(cosrad - armz*sinrad*sqrt2) - mx*((4*sinrad)/9 + (cosrad*sqrt2)/9) - mz*((4*cosrad)/9 - (sinrad*sqrt2)/9) - my*sqrt2
    //                                                          mz*((4*sinrad)/9 + (cosrad*sqrt2)/9) - mx*((4*cosrad)/9 - (sinrad*sqrt2)/9) - cosrad*fx + fz*sinrad

    extern float aim_pitch_deg;
    float sinrad = sinf(radians(aim_pitch_deg));
    float cosrad = cosf(radians(aim_pitch_deg));

    extern float ahrs_pitch_deg;

    Quaternion quat;
    quat.from_axis_angle(Vector3f{0, 1, 0}, radians(-ahrs_pitch_deg));

    Vector3f force_nav{forward_thrust, 0, -throttle_thrust};
    Vector3f force_body = quat * force_nav;

    float mx_factor = 0.5f;
    float mz_factor = 0.5f;

    _virutal_thrust[0] = mx_factor * mx * ((4 * sinrad) / 9 - (cosrad * sqrt2) / 9) - fz * (cosrad + armz * sinrad * sqrt2) - fx * (sinrad - armz * cosrad * sqrt2) + mz * ((4 * cosrad) / 9 + (sinrad * sqrt2) / 9) + my * sqrt2;
    _virutal_thrust[1] = mx_factor * mx * ((4 * cosrad) / 9 + (sinrad * sqrt2) / 9) - cosrad * fx - mz * ((4 * sinrad) / 9 - (cosrad * sqrt2) / 9) * mz_factor + fz * sinrad;
    _virutal_thrust[2] = mx_factor * mx * ((4 * sinrad) / 9 + (cosrad * sqrt2) / 9) - fz * (cosrad - armz * sinrad * sqrt2) - fx * (sinrad + armz * cosrad * sqrt2) + mz * ((4 * cosrad) / 9 - (sinrad * sqrt2) / 9) - my * sqrt2;
    _virutal_thrust[3] = cosrad * fx - mx_factor * mx * ((4 * cosrad) / 9 - (sinrad * sqrt2) / 9) + mz * ((4 * sinrad) / 9 + (cosrad * sqrt2) / 9) * mz_factor - fz * sinrad;
    _virutal_thrust[4] = my * sqrt2 - fz * (cosrad + armz * sinrad * sqrt2) - mx_factor * mx * ((4 * sinrad) / 9 - (cosrad * sqrt2) / 9) - mz * ((4 * cosrad) / 9 + (sinrad * sqrt2) / 9) - fx * (sinrad - armz * cosrad * sqrt2);
    _virutal_thrust[5] = cosrad * fx + mx_factor * mx * ((4 * cosrad) / 9 + (sinrad * sqrt2) / 9) - mz * ((4 * sinrad) / 9 - (cosrad * sqrt2) / 9) * mz_factor - fz * sinrad;
    _virutal_thrust[6] = -fx * (sinrad + armz * cosrad * sqrt2) - fz * (cosrad - armz * sinrad * sqrt2) - mx_factor * mx * ((4 * sinrad) / 9 + (cosrad * sqrt2) / 9) - mz * ((4 * cosrad) / 9 - (sinrad * sqrt2) / 9) - my * sqrt2;
    _virutal_thrust[7] = mz * ((4 * sinrad) / 9 + (cosrad * sqrt2) / 9) * mz_factor - mx_factor * mx * ((4 * cosrad) / 9 - (sinrad * sqrt2) / 9) - cosrad * fx + fz * sinrad;

    for (i = 0; i < 4; i++)
    {
        _thrust[i] = sqrtf(_virutal_thrust[2 * i + 1] * _virutal_thrust[2 * i + 1] + _virutal_thrust[2 * i] * _virutal_thrust[2 * i]);
        if (throttle_thrust > 0.15f)
        {
            if (i == 0 || i == 3)
            {
                _tilt[i] = (degrees(atan2f(_virutal_thrust[2 * i + 1], _virutal_thrust[2 * i]))) / 180.0f;
            }
            else if (i == 1 || i == 2)
            {
                _tilt[i] = (degrees(atan2f(_virutal_thrust[2 * i + 1], _virutal_thrust[2 * i]))) / 180.0f;
            }
        }
        else
        {
            _tilt[i] = 0.0f;
        }
    }

    // calculate left and right throttle outputs

    thrust_min = 1.0f;
    thrust_max = -1.0f;
    for (i = 0; i < 4; i++)
    {
        thrust_max = MAX(_thrust[i], thrust_max);
        thrust_min = MIN(_thrust[i], thrust_min);
    }

    if (thrust_max > 1.0f)
    {
        // if max thrust is more than one reduce average throttle
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
    }
    else if (thrust_min < 0.0)
    {
        // if min thrust is less than 0 increase average throttle
        // but never above max boost
        thr_adj = -thrust_min;
        if ((throttle_thrust + thr_adj) > max_boost_throttle)
        {
            thr_adj = MAX(max_boost_throttle - throttle_thrust, 0.0);
            // in this case we throw away some roll output, it will be uneven
            // constraining the lower motor more than the upper
            // this unbalances torque, but motor torque should have significantly less control power than tilts / control surfaces
            // so its worth keeping the higher roll control power at a minor cost to yaw
            limit.roll = true;
        }
        limit.throttle_lower = true;
    }

    // Add adjustment to reduce average throttle
    for (i = 0; i < 4; i++)
    {
        _thrust[i] = constrain_float(_thrust[i] + thr_adj, 0.0f, 1.0f);
    }
    // compensation_gain can never be zero
    // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
    _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix_with_Tilt::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq)
    {
    case 1:
        // right throttle
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
        break;
    case 2:
        // right tilt servo
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
        break;
    case 3:
        // left throttle
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
        break;
    case 4:
        // left tilt servo
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
        break;
    case 5:
        // right throttle
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor5, 500 + (pwm - 1000) * 2);
        break;
    case 6:
        // right tilt servo
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor6, 500 + (pwm - 1000) * 2);
        break;
    case 7:
        // left throttle
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor7, 500 + (pwm - 1000) * 2);
        break;
    case 8:
        // left tilt servo
        SRV_Channels::set_output_pwm(SRV_Channel::k_motor8, 500 + (pwm - 1000) * 2);
        break;
    default:
        // do nothing
        break;
    }
}
