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

    for (uint8_t i = 0; i < BALANCEBOT_MOTOR_NUM; i++) {
        _rmuart.rmuart_s.motor[i] = 1000;
    }
    for (uint8_t i = 0; i < BALANCEBOT_SERVO_NUM; i++) {
        _rmuart.rmuart_s.servo[i] = 1500;
    }
}

void AP_RMUART::update()
{
    if (_port == NULL)
        return;

    Send();
    Receive();
}

void AP_RMUART::Receive(void)
{
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
            if (data == sizeof(struct ardupilot_struct)) {
                receive_buff[count++] = data;
                _rx_step = 3;
            } else
                _rx_step = 0;
            break;

        case 3:
            receive_buff[count++] = data;
            if (count >= sizeof(struct ardupilot_struct)) {
                _rx_step = 0;
                count = 0;

                memcpy(ardupilot_rx.bits, receive_buff, sizeof(struct ardupilot_struct));

                gcs().send_text(MAV_SEVERITY_NOTICE, "wheel1=%d, wheel2=%d", ardupilot_rx.ardupilot_s.wheel_speed[0], ardupilot_rx.ardupilot_s.wheel_speed[1]);
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
    _rmuart.rmuart_s.header[0] = 0xAA;
    _rmuart.rmuart_s.header[1] = 0xAF;
    _rmuart.rmuart_s.timestamp_ms = AP_HAL::millis();

    SRV_Channel* out_chan;

    for (uint8_t i = 0; i < BALANCEBOT_MOTOR_NUM; i++) {
        out_chan = SRV_Channels::get_channel_for(SRV_Channel::Aux_servo_function_t(SRV_Channel::k_speedMotorRightWheel + i));
        if (out_chan == nullptr) {
            continue;
        }
        _rmuart.rmuart_s.motor[i] = out_chan->get_output_pwm();
    }

    for (uint8_t i = 0; i < BALANCEBOT_SERVO_NUM; i++) {
        out_chan = SRV_Channels::get_channel_for(SRV_Channel::Aux_servo_function_t(SRV_Channel::k_tiltMotorRightJoint + i));
        if (out_chan == nullptr) {
            continue;
        }
        _rmuart.rmuart_s.servo[i] = out_chan->get_output_pwm();
    }

    _rmuart.rmuart_s.len = sizeof(struct rmuart_struct);

    _port->write(_rmuart.bits, sizeof(struct rmuart_struct));
}