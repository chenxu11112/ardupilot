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

#include "AP_TemperatureCopterSensor_Analog.h"

#if AP_TEMPERATURE_COPTER_SENSOR_ANALOG_ENABLED
#include <utility>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>


const AP_Param::GroupInfo AP_TemperatureCopterSensor_Analog::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the analog input pin that should be used for voltage monitoring.
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_PIN", 1, AP_TemperatureCopterSensor_Analog, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("_MULT", 2, AP_TemperatureCopterSensor_Analog, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Voltage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("_OFFSET", 3, AP_TemperatureCopterSensor_Analog, _volt_offset, 0),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};


extern const AP_HAL::HAL &hal;


AP_TemperatureCopterSensor_Analog::AP_TemperatureCopterSensor_Analog(AP_TemperatureCopterSensor &mon, 
                                AP_TemperatureCopterSensor::TemperatureSensor_State &mon_state,
                                AP_TemperatureCopterSensor_Params &params):
    AP_TemperatureCopterSensor_Backend(mon, mon_state, params)

{
    AP_Param::setup_object_defaults(this, var_info);

    _state.var_info = var_info;
    
    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
}


// read - read the voltage and current
void AP_TemperatureCopterSensor_Analog::update()
{
    const uint32_t tnow = AP_HAL::micros();

    _state.temperature = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    // this copes with changing the pin at runtime
    _state.last_time_ms = tnow;
}

#endif // AP_TEMPERATURE_COPTER_SENSOR_ANALOG_ENABLED
