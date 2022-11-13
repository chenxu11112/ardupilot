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
                parse_state = ParseState_WaitingForModBusFunc;
                linebuf_len = 0;
            }
            break;

        case ParseState_WaitingForModBusFunc:
            if (c == 0x03)
            {
                parse_state = ParseState_WaitingForLen;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForLen:
            parse_msg_len = c;
            linebuf_len++;

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
            linebuf_len++;

            if (c == 0xAC)
            {
                parse_state = ParseState_WaitingForFlag2;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForFlag2:
            linebuf_len++;

            if (c == 0xDA)
            {
                parse_state = ParseState_WaitingForInfoFlag1;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForInfoFlag1:
            linebuf_len++;

            if (c == 0x00)
            {
                parse_state = ParseState_WaitingForInfoFlag2;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForInfoFlag2:
            linebuf_len++;

            if (c == 0x03)
            {
                parse_state = ParseState_WaitingForContents;
            }
            else
            {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForContents:
            linebuf[linebuf_len++] = c;
            if (linebuf_len == parse_msg_len)
            {
                // process buffer
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
    // check crc
    uint16_t crc = crc_crc16_ibm(0, linebuf, linebuf_len - 2); // 2: CRC16

    uint8_t crc1 = ((uint8_t)(((uint64_t)(crc)) & 0xff));
    uint8_t crc2 = ((uint8_t)((((uint64_t)(crc)) >> 8) & 0xff));

    if ((crc1 != linebuf[45]) || (crc2 != linebuf[46]))
        return;

    int16_t vehicle_x = (uint16_t)linebuf[39] << 8 | (uint16_t)linebuf[40];
    int32_t vehicle_y = (uint16_t)linebuf[41] << 8 | (uint16_t)linebuf[42];
    int32_t vehicle_z = (uint16_t)linebuf[43] << 8 | (uint16_t)linebuf[44];

    Vector3f veh_pos(Vector3f(vehicle_x * 0.01f, vehicle_y * 0.01f, vehicle_z * 0.01f));

    set_vehicle_position(veh_pos, 0.1f);

    last_update_ms = AP_HAL::millis();
}
