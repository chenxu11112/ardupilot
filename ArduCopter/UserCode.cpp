#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
float aim_pitch_deg;
float delta_pitch_deg_s;
float ahrs_pitch_deg;

const float norm_pitch=4.5;
const float tilt_pitch=1.5;

const float norm_rate_pitch=0.15;
const float tilt_rate_pitch=0.05;

#define default_angle 60.0f
#define default_angle_speed 0.04f

#define secend_degree 80.0f
#define secend_angle_speed 0.02f

#define third_degree 90.0f
#define third_angle_speed 0.01f

#define use_adjust_pid

void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uint16_t value = hal.rcin->read(CH_6);
    if (value < 1600 && value >1400)
    {
        if (aim_pitch_deg < default_angle)
        {
            aim_pitch_deg += default_angle_speed;
            delta_pitch_deg_s = default_angle_speed;
        }
#ifdef secend_degree
        else if (aim_pitch_deg < secend_degree)
        {
            aim_pitch_deg += secend_angle_speed;
            delta_pitch_deg_s = secend_angle_speed;
        }
#endif
#ifdef third_degree
        else if (aim_pitch_deg < third_degree)
        {
            aim_pitch_deg += third_angle_speed;
            delta_pitch_deg_s = third_angle_speed;
        }
#endif
        else
        {
            delta_pitch_deg_s = 0.0f;
        }
    }
    else if (value < 2050 && value >1850)
    {
#ifdef third_degree
        if (aim_pitch_deg > secend_degree)
        {
            aim_pitch_deg += -third_angle_speed;
            delta_pitch_deg_s = -third_angle_speed;
        }
#endif
#ifdef secend_degree
        else if (aim_pitch_deg > default_angle)
        {
            aim_pitch_deg += -secend_angle_speed;
            delta_pitch_deg_s = -secend_angle_speed;
        }else
#endif
        if (aim_pitch_deg > 0.0f)
        {
            aim_pitch_deg += -default_angle_speed;
            delta_pitch_deg_s = -default_angle_speed;
        }
        else
        {
            delta_pitch_deg_s = 0.0f;
        }
    }
    else
    {
        delta_pitch_deg_s = 0.0f;
    }

    ahrs_pitch_deg = degrees(ahrs.get_pitch());

#ifdef use_adjust_pid
    float k1 = (tilt_pitch - norm_pitch) / (90.0f - 0.0f);
    float pitch_kp = norm_pitch + k1 * (aim_pitch_deg - 0.0f);
    copter.attitude_control->get_angle_pitch_p().kP(pitch_kp);

    float k2 = (tilt_rate_pitch - norm_rate_pitch) / (90.0f - 0.0f);
    float pitch_rate_kp = norm_rate_pitch + k2 * (aim_pitch_deg - 0.0f);
    copter.attitude_control->get_rate_pitch_pid().kP(pitch_rate_kp);
#endif

    // printf("pitch_rate_kp=%f, pitch_kp=%f\r\n", pitch_rate_kp, pitch_kp);
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    gcs().send_text(MAV_SEVERITY_NOTICE, "aim_pitch_deg=%.3f", aim_pitch_deg);

#ifdef use_adjust_pid
    gcs().send_text(MAV_SEVERITY_NOTICE, "rate_p=%.3f", (float)copter.attitude_control->get_rate_pitch_pid().kP());
    gcs().send_text(MAV_SEVERITY_NOTICE, "rate_p=%.3f", (float)copter.attitude_control->get_angle_pitch_p().kP());
#endif
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
