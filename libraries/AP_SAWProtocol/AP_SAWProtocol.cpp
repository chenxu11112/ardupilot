#define AP_SERIALMANAGER_SAW_BAUD       115200
#define AP_SERIALMANAGER_SAW_BUFSIZE_RX 64
#define AP_SERIALMANAGER_SAW_BUFSIZE_TX 64

#include "AP_SAWProtocol.h"
#include "stdio.h"
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define DEF_OP_START      0xCC // 启动
#define DEF_OP_STOP       0X00 // 停止

#define DEF_DP_ON         0xdd // 脱钩
#define DEF_DP_OFF        0x11 // 不脱钩

#define DEF_SC_DATA       0x42
#define DEF_LIMIT_CMD     0xee
#define DEF_CURRENT_LIMIT 40

extern const AP_HAL::HAL& hal;

// constructor
AP_SAWProtocol::AP_SAWProtocol(void)
{
    _port    = NULL;
    _rx_step = 0;
}

// init - perform require initialisation including detecting which protocol to
// use
void AP_SAWProtocol::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(
             AP_SerialManager::SerialProtocol_SAW, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_SAW_BAUD,
                     AP_SERIALMANAGER_SAW_BUFSIZE_RX,
                     AP_SERIALMANAGER_SAW_BUFSIZE_TX);
    }

    memset(fcu_to_saw_union.bits, 0, sizeof(struct FCU_to_SAW_struct));
    memset(saw_to_fcu_union.bits, 0, sizeof(struct SAW_to_FCU_struct));

    fcu_to_saw_union.fcu_to_saw_struct.MSG_OP         = 0x03;
    fcu_to_saw_union.fcu_to_saw_struct.OP_IS_START    = DEF_OP_STOP;
    fcu_to_saw_union.fcu_to_saw_struct.MSG_DP         = 0x04;
    fcu_to_saw_union.fcu_to_saw_struct.MSG_DP_IS_ON   = DEF_DP_OFF;
    fcu_to_saw_union.fcu_to_saw_struct.THROTTLE_CMD   = 0x0B;
    fcu_to_saw_union.fcu_to_saw_struct.THROTTLE_Value = 0;
    fcu_to_saw_union.fcu_to_saw_struct.MSG_SC_DATA    = DEF_SC_DATA;
    fcu_to_saw_union.fcu_to_saw_struct.LIMIT_CMD      = DEF_LIMIT_CMD;
    fcu_to_saw_union.fcu_to_saw_struct.CURRENT_LIMI   = DEF_CURRENT_LIMIT;
}

void AP_SAWProtocol::update()
{
    if (_port == NULL)
        return;

    Send();
    // Receive();
}

void AP_SAWProtocol::Receive(void)
{
    if (_port == NULL)
        return;

    // uint8_t numc = _port->available();
    // uint8_t data = 0;
    // uint8_t count = 0;

    // for (uint8_t i = 0; i < numc; i++)
    // {
    //     data = _port->read();

    //     switch (_rx_step)
    //     {
    //     case 0:
    //         if (data == 0xAA)
    //         {
    //             receive_buff[count++] = data;
    //             _rx_step = 1;
    //             count = 0;
    //         }
    //         break;

    //     case 1:
    //         if (data == 0xAC)
    //         {
    //             receive_buff[count++] = data;
    //             _rx_step = 2;
    //         }
    //         else
    //         {
    //             _rx_step = 0;
    //             count = 0;
    //         }
    //         break;

    //     case 2:
    //         if (data == sizeof(struct SAW_to_FCU_struct))
    //         {
    //             receive_buff[count++] = data;
    //             _rx_step = 3;
    //         }
    //         else
    //         {
    //             _rx_step = 0;
    //             count = 0;
    //         }
    //         break;

    //     case 3:
    //         receive_buff[count++] = data;
    //         if (count >= (sizeof(struct SAW_to_FCU_struct) - 1))
    //         {
    //             _rx_step = 4;
    //             count = 0;
    //         }
    //         break;

    //     case 4:
    //         if (data == SAW_to_FCU_struct_TAIL)
    //         {
    //             receive_buff[count++] = data;
    //             memcpy(saw_to_fcu_union.bits, receive_buff, sizeof(struct SAW_to_FCU_struct));

    //             gcs().send_text(MAV_SEVERITY_INFO, "saw_state=%d\n", saw_to_fcu_union.saw_to_fcu_struct.saw_state);
    //         }
    //         _rx_step = 0;
    //         count = 0;

    //         break;

    //     default:
    //         _rx_step = 0;
    //         count = 0;
    //         break;
    //     }
    // }
}

void AP_SAWProtocol::Send(void)
{
    if (_port == NULL)
        return;

    if (hal.rcin->read(CH_7) > 1700) {
        fcu_to_saw_union.fcu_to_saw_struct.MSG_DP_IS_ON = DEF_DP_ON;
    } else {
        fcu_to_saw_union.fcu_to_saw_struct.MSG_DP_IS_ON = DEF_DP_OFF;
    }

    if (hal.rcin->read(CH_9) > 1700) {
        fcu_to_saw_union.fcu_to_saw_struct.OP_IS_START = DEF_OP_START;
    } else {
        fcu_to_saw_union.fcu_to_saw_struct.OP_IS_START = DEF_OP_STOP;
    }

    fcu_to_saw_union.fcu_to_saw_struct.THROTTLE_Value = 8;

    _port->write(fcu_to_saw_union.bits, sizeof(struct FCU_to_SAW_struct));
}
