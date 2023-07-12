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
   OpenMV library
*/

#define AP_SERIALMANAGER_RMUART_BAUD       115200
#define AP_SERIALMANAGER_RMUART_BUFSIZE_RX 64
#define AP_SERIALMANAGER_RMUART_BUFSIZE_TX 64

#include "AP_RMUART.h"
#include "stdio.h"
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_RMUART::AP_RMUART(void)
{
    _port = NULL;
    _rx_step = 0;
}

// init - perform require initialisation including detecting which protocol to
// use
void AP_RMUART::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_RMUART, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_RMUART_BAUD,
                     AP_SERIALMANAGER_RMUART_BUFSIZE_RX,
                     AP_SERIALMANAGER_RMUART_BUFSIZE_TX);
    }

    memset(apm_2_stm32.bits, 0, sizeof(struct apm_2_stm32_struct));
    memset(stm32_2_apm.bits, 0, sizeof(struct stm32_2_apm_struct));

    apm_2_stm32.apm_2_stm32_t.header[0] = 0xAA;
    apm_2_stm32.apm_2_stm32_t.header[1] = 0xAF;
    apm_2_stm32.apm_2_stm32_t.len = sizeof(struct apm_2_stm32_struct);
}

void AP_RMUART::update()
{
    if (_port == NULL)
        return;

    Send();
    Receive();
}

float wheel1, wheel2;

void AP_RMUART::Receive(void)
{
    if (_port == NULL)
        return;

    uint8_t numc = _port->available();
    uint8_t data = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < numc; i++) {
        data = _port->read();

        switch (_rx_step) {
        case 0:
            if (data == 0xAA) {
                receive_buff[count++] = data;
                _rx_step = 1;
            }
            break;

        case 1:
            if (data == 0xAC) {
                receive_buff[count++] = data;
                _rx_step = 2;
            } else
                _rx_step = 0;
            break;

        case 2:
            if (data == sizeof(struct stm32_2_apm_struct)) {
                receive_buff[count++] = data;
                _rx_step = 3;
            } else
                _rx_step = 0;
            break;

        case 3:
            receive_buff[count++] = data;
            if (count >= sizeof(struct stm32_2_apm_struct)) {
                _rx_step = 0;
                count = 0;

                memcpy(stm32_2_apm.bits, receive_buff, sizeof(struct stm32_2_apm_struct));

                // printf("wheel_left_speed=%d\n", ardupilot_rx.ardupilot_s.wheel_left_speed);
                // gcs().send_text(MAV_SEVERITY_INFO,"wheel_left_speed=%d\n", ardupilot_rx.ardupilot_s.wheel_left_speed );
                // getWheelSpeed(wheel1, wheel2);
            }
            break;

        default:
            _rx_step = 0;
            count = 0;
            break;
        }
    }
}

void AP_RMUART::Send(void)
{
    if (_port == NULL)
        return;

    apm_2_stm32.apm_2_stm32_t.len = sizeof(struct apm_2_stm32_struct);

    _port->write(apm_2_stm32.bits, sizeof(struct apm_2_stm32_struct));
}