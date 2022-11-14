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
 Original C Code by Marvelmind (https://github.com/MarvelmindRobotics/marvelmind.c)
 Adapted into Ardupilot by Karthik Desai, Amilcar Lucas
 April 2017
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include "AP_Beacon_LinkPG.h"
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_LinkPG::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_LinkPG::update(void)
{
    if (uart == nullptr)
    {
        return;
    }

    // read any available characters
    int16_t nbytes = uart->available();

    while (nbytes-- > 0)
    {
        uint8_t c = uart->read();

        switch (parse_state)
        {
        default:
        case ParseState_WaitingForModBusID:
            if (c == 0x01)
            {
                linebuf_len = 0;
                parse_state = ParseState_WaitingForModBusFunc;
                linebuf[linebuf_len++] = c;
            }
            break;

        case ParseState_WaitingForModBusFunc:
            if (c == 0x03)
            {
                parse_state = ParseState_WaitingForLen;
                linebuf[linebuf_len++] = c;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForLen:
            parse_msg_len = c;
            linebuf[linebuf_len++] = c;

            if (parse_msg_len > AP_BEACON_LINKPG_BUF_SIZE)
            {
                // invalid message length
                parse_state = ParseState_WaitingForModBusID;
            }
            else
            {
                parse_state = ParseState_WaitingForFlag1;
            }
            break;

        case ParseState_WaitingForFlag1:

            if (c == 0xAC)
            {
                parse_state = ParseState_WaitingForFlag2;

                linebuf[linebuf_len++] = c;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForFlag2:

            if (c == 0xDA)
            {
                linebuf[linebuf_len++] = c;

                parse_state = ParseState_WaitingForInfoFlag1;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForInfoFlag1:

            if (c == 0x00)
            {
                linebuf[linebuf_len++] = c;

                parse_state = ParseState_WaitingForInfoFlag2;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForInfoFlag2:
            if (c == 0x03)
            {
                linebuf[linebuf_len++] = c;

                parse_state = ParseState_WaitingForContents;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForContents:
            linebuf[linebuf_len++] = c;
            if (linebuf_len == (parse_msg_len + 3 + 2))
            {
                // process buffer

                // printf("parse_buffer\r\n");

                parse_buffer();
                // reset state for next message
                parse_state = ParseState_WaitingForModBusID;
            }
            break;
        }
    }
}

// parse buffer
void AP_Beacon_LinkPG::parse_buffer()
{
    // for (int i = 0; i < 47; i++)
    // {
    //     printf("%x ", linebuf[i]);
    // }
    // printf("\r\n");

    // check crc
    uint16_t crc = calc_crc_modbus(linebuf, linebuf_len - 2); // CRC16

    uint8_t crc1 = ((uint8_t)(((uint64_t)(crc)) & 0xff));
    uint8_t crc2 = ((uint8_t)((((uint64_t)(crc)) >> 8) & 0xff));

    // printf("crc=0x%x,0x%x\r\n", crc1, crc2);

    if ((crc1 != linebuf[45]) || (crc2 != linebuf[46]))
    {
        return;
    }
    int16_t vehicle_x = (uint16_t)linebuf[39] << 8 | (uint16_t)linebuf[40];
    int16_t vehicle_y = (uint16_t)linebuf[41] << 8 | (uint16_t)linebuf[42];
    int16_t vehicle_z = (uint16_t)linebuf[43] << 8 | (uint16_t)linebuf[44];

    Vector3f veh_pos(Vector3f(vehicle_x * 0.01f, vehicle_y * 0.01f, vehicle_z * 0.01f));

    printf("veh_pos:%f,%f,%f\r\n", veh_pos[0], veh_pos[1], veh_pos[2]);

    set_vehicle_position(veh_pos, 0.1f);

    last_update_ms = AP_HAL::millis();
}
