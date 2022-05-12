#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
float aim_roll;
float delta_roll;
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    uint16_t value = hal.rcin->read(CH_6);
    if (value == 1500)
    {
        if (aim_roll < 70.0f)
        {
            aim_roll += 0.1f;
            delta_roll = 0.1f;
        }
        else if (aim_roll < 80.0f)
        {
            aim_roll += 0.05f;
            delta_roll = 0.05f;
        }
        else if (aim_roll < 90.0f)
        {
            aim_roll += 0.02f;
            delta_roll = 0.02f;
        }
        else
        {
            delta_roll = 0.0f;
        }
    }
    else if (value == 2000)
    {
        if (aim_roll > 20.0f)
        {
            aim_roll -= 0.1f;
            delta_roll = -0.1f;
        }
        else if (aim_roll > 10.0f)
        {
            aim_roll -= 0.05f;
            delta_roll = -0.05f;
        }
        else if (aim_roll > 0.0f)
        {
            aim_roll -= 0.02f;
            delta_roll = -0.02f;
        }
        else
        {
            delta_roll = 0.0f;
        }
    }
    else
    {
        delta_roll = 0.0f;
    }

    printf("pitch=%f\r\n", aim_roll);
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
