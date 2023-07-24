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

#include "AP_TemperatureSensor_Analog.h"

#if AP_TEMPERATURE_SENSOR_ANALOG_ENABLED

#include <utility>
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

extern const AP_HAL::HAL &hal;

#define KELVIN_ZEROS_TEMPATURE 273.15f 

const AP_Param::GroupInfo AP_TemperatureSensor_Analog::var_info[] = {

    // @Param: VOLT_PIN
    AP_GROUPINFO("PIN", 1, AP_TemperatureSensor_Analog, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("MULT", 2, AP_TemperatureSensor_Analog, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Voltage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("OFFSET", 3, AP_TemperatureSensor_Analog, _volt_offset, 0),
    
    AP_GROUPINFO("Rp", 4, AP_TemperatureSensor_Analog, _Rp, 10000),

    AP_GROUPINFO("T2", 5, AP_TemperatureSensor_Analog, _T2, 25),

    AP_GROUPINFO("B", 6, AP_TemperatureSensor_Analog, _B, 3950),

    AP_GROUPINFO("Vcc", 7, AP_TemperatureSensor_Analog, _Vcc, 3.3),

    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

AP_TemperatureSensor_Analog::AP_TemperatureSensor_Analog(AP_TemperatureSensor &mon, 
                                AP_TemperatureSensor::TemperatureSensor_State &mon_state,
                                AP_TemperatureSensor_Params &params):
    AP_TemperatureSensor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);

    _state.var_info = var_info;

    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);

}

// read - read the voltage and current
void AP_TemperatureSensor_Analog::update()
{
    if(_volt_pin == -1)
    {
        return;
    }

    float voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    float rt = (_Vcc - voltage) / (voltage / _Rp);

    _state.temperature = 1 / ((_T2 + KELVIN_ZEROS_TEMPATURE) + logf(rt / _Rp) / _B) - KELVIN_ZEROS_TEMPATURE;

    hal.console->printf("_state.temperature=%f\r",_state.temperature);
}

#endif // AP_TEMPERATURE_COPTER_SENSOR_ANALOG_ENABLED
