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

#define MAX_BALANCE_MAX_SPEED   10000.0f // 电机最大转速
#define MAX_BALANCE_MAX_CURRENT 10000.0f // 电机最大电流

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

    // 从电机转速映射到【-1~1】
    void getWheelSpeed(float& wheelleft_f, float& wheelright_f)
    {
        wheelleft_f = (float)(stm32_2_apm.stm32_2_apm_t.wheel_left_int) / MAX_BALANCE_MAX_SPEED;
        wheelright_f = (float)(stm32_2_apm.stm32_2_apm_t.wheel_right_int) / MAX_BALANCE_MAX_SPEED;
    }

    // 从【-1~1】映射到电机转速
    void setWheelSpeed(float& wheelleft_f, float& wheelright_f)
    {
        apm_2_stm32.apm_2_stm32_t.wheel_left_int = (int16_t)(wheelleft_f * MAX_BALANCE_MAX_CURRENT);
        apm_2_stm32.apm_2_stm32_t.wheel_right_int = (int16_t)(wheelright_f * MAX_BALANCE_MAX_CURRENT);
    }

    struct PACKED apm_2_stm32_struct {
        uint8_t header[2];
        uint8_t len;
        int16_t wheel_left_int;
        int16_t wheel_right_int;
    };
    union apm_2_stm32_union {
        struct apm_2_stm32_struct apm_2_stm32_t;
        uint8_t bits[sizeof(struct apm_2_stm32_struct)];
    };

    struct PACKED stm32_2_apm_struct {
        uint8_t header[2];
        uint8_t len;
        int16_t wheel_left_int;
        int16_t wheel_right_int;
    };
    union stm32_2_apm_union {
        struct stm32_2_apm_struct stm32_2_apm_t;

        uint8_t bits[sizeof(struct stm32_2_apm_struct)];
    };

private:
    AP_HAL::UARTDriver* _port; // UART used to send data to receiver

    apm_2_stm32_union apm_2_stm32;
    stm32_2_apm_union stm32_2_apm;

    uint8_t _rx_step;

    uint8_t receive_buff[20];
};