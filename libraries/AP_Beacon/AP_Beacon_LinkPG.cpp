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

#include "AP_Beacon_LinkPG.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Beacon_LinkPG::var_info[] = {

    // @Param: Beacon 0
    AP_GROUPINFO("b0_X", 1, AP_Beacon_LinkPG, beaconXYZ[0][0], 0), AP_GROUPINFO("b0_Y", 2, AP_Beacon_LinkPG, beaconXYZ[0][1], 0),
    AP_GROUPINFO("b0_Z", 3, AP_Beacon_LinkPG, beaconXYZ[0][2], 0),

    // @Param: Beacon 1
    AP_GROUPINFO("b1_X", 4, AP_Beacon_LinkPG, beaconXYZ[1][0], 0), AP_GROUPINFO("b1_Y", 5, AP_Beacon_LinkPG, beaconXYZ[1][1], 600),
    AP_GROUPINFO("b1_Z", 6, AP_Beacon_LinkPG, beaconXYZ[1][2], 300),

    // @Param: Beacon 2
    AP_GROUPINFO("b2_X", 7, AP_Beacon_LinkPG, beaconXYZ[2][0], 600), AP_GROUPINFO("b2_Y", 8, AP_Beacon_LinkPG, beaconXYZ[2][1], 600),
    AP_GROUPINFO("b2_Z", 9, AP_Beacon_LinkPG, beaconXYZ[2][2], 300),

    // @Param: Beacon 3
    AP_GROUPINFO("b3_X", 10, AP_Beacon_LinkPG, beaconXYZ[3][0], 600), AP_GROUPINFO("b3_Y", 11, AP_Beacon_LinkPG, beaconXYZ[3][1], 0),
    AP_GROUPINFO("b3_Z", 12, AP_Beacon_LinkPG, beaconXYZ[3][2], 0),

    // @Param: Beacon 0 id
    AP_GROUPINFO("b0_id", 13, AP_Beacon_LinkPG, beaconID[0], 0),

    // @Param: Beacon 1 id
    AP_GROUPINFO("b1_id", 14, AP_Beacon_LinkPG, beaconID[1], 5),

    // @Param: Beacon 2 id
    AP_GROUPINFO("b2_id", 15, AP_Beacon_LinkPG, beaconID[2], 3),

    // @Param: Beacon 3 id
    AP_GROUPINFO("b3_id", 16, AP_Beacon_LinkPG, beaconID[3], 2),

    AP_GROUPEND
};

AP_Beacon_LinkPG::AP_Beacon_LinkPG(AP_Beacon& frontend)
    : AP_Beacon_Backend(frontend)
    , vehPosFilter { 4, 4, 4 }
    , beaconDistFilter { 4, 4, 4, 4 }
{
    AP_Param::setup_object_defaults(this, var_info);
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_LinkPG::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

void AP_Beacon_LinkPG::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // read any available characters
    int16_t nbytes = uart->available();
    // hal.console->printf("update bcn,nbytes=%d\n", nbytes);

    while (nbytes-- > 0) {
        uint8_t c = uart->read();

        switch (parse_state) {
        default:
        case ParseState_WaitingForModBusID:
            if (c == 0x01) {
                linebuf_len = 0;
                parse_state = ParseState_WaitingForModBusFunc;
                linebuf[linebuf_len++] = c;
            }
            break;

        case ParseState_WaitingForModBusFunc:
            if (c == 0x03) {
                parse_state = ParseState_WaitingForLen;
                linebuf[linebuf_len++] = c;
            } else {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForLen:
            parse_msg_len = c;
            linebuf[linebuf_len++] = c;

            if (parse_msg_len > AP_BEACON_LINKPG_BUF_SIZE) {
                // invalid message length
                parse_state = ParseState_WaitingForModBusID;
            } else {
                parse_state = ParseState_WaitingForFlag1;
            }
            break;

        case ParseState_WaitingForFlag1:

            if (c == 0xAC) {
                parse_state = ParseState_WaitingForFlag2;

                linebuf[linebuf_len++] = c;
            } else {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForFlag2:

            if (c == 0xDA) {
                linebuf[linebuf_len++] = c;

                parse_state = ParseState_WaitingForInfoFlag1;
            } else {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForInfoFlag1:

            if (c == 0x00) {
                linebuf[linebuf_len++] = c;

                parse_state = ParseState_WaitingForInfoFlag2;
            } else {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForInfoFlag2:
            if (c == 0x03) {
                linebuf[linebuf_len++] = c;

                parse_state = ParseState_WaitingForContents;
            } else {
                parse_state = ParseState_WaitingForModBusID;
            }
            break;

        case ParseState_WaitingForContents:
            linebuf[linebuf_len++] = c;
            if (linebuf_len == (parse_msg_len + 3 + 2)) {
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
    // hal.console->printf("parse_buffer\n");

    // for (int i = 0; i < 47; i++)
    // {
    //     hal.console->printf("%x ", linebuf[i]);
    // }
    // hal.console->printf("\r\n");

    // check crc
    uint16_t crc = calc_crc_modbus(linebuf, linebuf_len - 2); // CRC16

    uint8_t crc1 = ((uint8_t)(((uint64_t)(crc)) & 0xff));
    uint8_t crc2 = ((uint8_t)((((uint64_t)(crc)) >> 8) & 0xff));

    // printf("crc=0x%x,0x%x\r\n", crc1, crc2);

    if ((crc1 != linebuf[45]) || (crc2 != linebuf[46])) {
        // hal.console->printf("crc error\r\n");
        // gcs().send_text(MAV_SEVERITY_WARNING, "crc error\r\n");
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    int16_t veh_pos_temp;
    // linkpg beacon is enu, transform to ned

    for (uint8_t i = 0; i < 3; i++) {
        veh_pos_temp = (int16_t)((uint16_t)linebuf[39 + 2 * i] << 8 | (uint16_t)linebuf[40 + 2 * i]);

        veh_pos_temp = vehPosFilter[i].apply(veh_pos_temp);

        vehPos[i] = 0.01f * (float)veh_pos_temp;

        vehPos[i] = vehposbutter[i].filter(vehPos[i]);
    }

    // enu to ned
    vehPos.rotate(ROTATION_PITCH_180_YAW_270);
    set_vehicle_position(vehPos, 0.15f);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint16_t distance_temp;

    for (uint8_t i = 0; i < AP_BEACON_LINKPG_MAX_NUM; i++) {
        // beaconDistance
        distance_temp = (uint16_t)((uint16_t)linebuf[7 + beaconID[i] * 2] << 8 | (uint16_t)linebuf[8 + beaconID[i] * 2]);

        distance_temp = beaconDistFilter[i].apply(distance_temp);

        beaconDistance[i] = 0.01f * (float)distance_temp;

        beaconDistance[i] = beaconDistbutter[i].filter(beaconDistance[i]);

        set_beacon_distance(i, beaconDistance[i]);

        /////////////////////////////////////////////////
        printf("beaconDistance[%d]=%f\r\n", i, beaconDistance[i]);
    }

    uint32_t millis = AP_HAL::millis();
    uint32_t delta_ms = millis - last_update_ms;
    // hal.console->printf("%d\r\n", last_update_ms);
    printf("%d\r\n", delta_ms);

    last_update_ms = millis;
}


void AP_Beacon_LinkPG::setBeaconXYZ()
{
        Vector3f beacon_pos;
    for (uint8_t i = 0; i < AP_BEACON_LINKPG_MAX_NUM; i++) {
        beacon_pos[0] = 0.01f * (float)(beaconXYZ[i][0]);
        beacon_pos[1] = 0.01f * (float)(beaconXYZ[i][1]);
        beacon_pos[2] = 0.01f * (float)(beaconXYZ[i][2]);

        // beaconXYZ is enu, transform to ned
        beacon_pos.rotate(ROTATION_PITCH_180_YAW_270);
        set_beacon_position(i, beacon_pos);

        printf("beaconXYZ[%d]=%f,%f,%f\r\n", i, beacon_pos.x, beacon_pos.y, beacon_pos.z);
    }
}