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

#include "AP_TemperatureCopterSensor_Params.h"
#include "AP_TemperatureCopterSensor.h"

#if AP_TEMPERATURE_COPTER_SENSOR_ENABLED

#ifndef AP_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT
#define AP_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT 0
#endif

#ifndef AP_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT
#define AP_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT 0
#endif

#ifndef AP_TEMPERATURE_SENSOR_SOURCE_ID_DEFAULT
#define AP_TEMPERATURE_SENSOR_SOURCE_ID_DEFAULT -1
#endif

const AP_Param::GroupInfo AP_TemperatureCopterSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Temperature Sensor Type
    // @Description: Enables temperature sensors
    // @Values: 0:Disabled, 1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_TemperatureCopterSensor_Params, type, (float)Type::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: SRC
    // @DisplayName: Sensor Source
    // @Description: Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.
    // @Values: 0: None, 1:ESC, 2:Motor(not implemented yet), 3:Battery Index, 4:Battery ID/SerialNumber
    // @User: Standard
    AP_GROUPINFO("SRC", 4, AP_TemperatureCopterSensor_Params, source, (float)Source::None),



    AP_GROUPEND
};

AP_TemperatureCopterSensor_Params::AP_TemperatureCopterSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_TEMPERATURE_COPTER_SENSOR_ENABLED
