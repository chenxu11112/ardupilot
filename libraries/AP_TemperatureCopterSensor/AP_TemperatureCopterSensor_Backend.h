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
#pragma once

#include "AP_TemperatureCopterSensor.h"

#if AP_TEMPERATURE_COPTER_SENSOR_ENABLED
#include <AP_HAL/Semaphores.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

class AP_TemperatureCopterSensor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_TemperatureCopterSensor_Backend(AP_TemperatureCopterSensor &front, AP_TemperatureCopterSensor::TemperatureSensor_State &state, AP_TemperatureCopterSensor_Params &params);

    // we declare a virtual destructor so that BattMonitor driver can
    // override with a custom destructor if need be
    virtual ~AP_TemperatureCopterSensor_Backend(void) {}

    // initialise
    virtual void init() {};

    // update the latest temperature
    virtual void update() = 0;

    // do we have a valid temperature reading?
    virtual bool healthy(void) const;

    // logging functions
    void Log_Write_TEMP() const;

protected:

    void set_temperature(const float temperature);

    AP_TemperatureCopterSensor                            &_front;    // reference to front-end
    AP_TemperatureCopterSensor::TemperatureSensor_State   &_state;    // reference to this instance's state (held in the front-end)
    AP_TemperatureCopterSensor_Params                     &_params;   // reference to this instance's parameters (held in the front-end)
};

#endif // AP_TEMPERATURE_COPTER_SENSOR_ENABLED
