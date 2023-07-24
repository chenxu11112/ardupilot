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
 * I2C driver for Measurement Specialties MEAS TSYS01 digital temperature sensor
 */

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#if AP_TEMPERATURE_SENSOR_ANALOG_ENABLED

#define AP_BATT_VOLT_PIN            -1
#define AP_BATT_VOLTDIVIDER_DEFAULT -1

class AP_TemperatureSensor_Analog : public AP_TemperatureSensor_Backend {
public:
    AP_TemperatureSensor_Analog(AP_TemperatureSensor &mon, 
                                AP_TemperatureSensor::TemperatureSensor_State &mon_state,
                                AP_TemperatureSensor_Params &params);

    void init(void) override {};

    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    
    // Parameters
    AP_Float _Rp; 
    AP_Float _T2;    
    AP_Float _B;
    AP_Float _Vcc; 

    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _volt_offset;              /// offset voltage that is subtracted from voltage pin before conversion
    AP_Int8  _volt_pin;                 /// board pin used to measure battery voltage

};


#endif // AP_TEMPERATURE_SENSOR_TSYS01_ENABLED
