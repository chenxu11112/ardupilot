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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define BALANCEBOT_MOTOR_NUM 2
#define BALANCEBOT_SERVO_NUM 2
class AP_RMUART {
public:
    AP_RMUART();

    /* Do not allow copies */
    AP_RMUART(const AP_RMUART& other) = delete;
    AP_RMUART& operator=(const AP_RMUART&) = delete;

    // init - perform require initialisation including detecting which protocol
    // to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    void update(void);

    void Receive(void);

    void Send(void);

    struct PACKED rmuart_struct {
        uint8_t header[2];
        uint8_t len;
        uint32_t timestamp_ms;
        uint16_t motor[BALANCEBOT_MOTOR_NUM];
        uint16_t servo[BALANCEBOT_SERVO_NUM];
    };

    union rmuart_t {
        struct rmuart_struct rmuart_s;
        uint8_t bits[sizeof(struct rmuart_struct)];
    };

    struct PACKED ardupilot_struct {
        uint8_t header[2];
        uint8_t len;
        int16_t wheel_speed[BALANCEBOT_MOTOR_NUM];
    };
    union ardupilot_t {
        struct ardupilot_struct ardupilot_s;

        uint8_t bits[sizeof(struct ardupilot_struct)];
    };

private:
    AP_HAL::UARTDriver* _port; // UART used to send data to receiver

    rmuart_t _rmuart;
    ardupilot_t ardupilot_rx;

    uint8_t  _rx_step;

    uint8_t receive_buff[20];
};