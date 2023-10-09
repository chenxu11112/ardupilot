#define AP_SERIALMANAGER_SAW_BAUD       115200
#define AP_SERIALMANAGER_SAW_BUFSIZE_RX 64
#define AP_SERIALMANAGER_SAW_BUFSIZE_TX 64

#include "AP_Saw.h"
#include "stdio.h"
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_Dummy.h>

#define FCU_to_SAW_struct_HEADER_1 0XAA
#define FCU_to_SAW_struct_HEADER_2 0XAF
#define FCU_to_SAW_struct_TAIL 0XFA

#define SAW_to_FCU_struct_HEADER_1 0XAA
#define SAW_to_FCU_struct_HEADER_2 0XAC
#define SAW_to_FCU_struct_TAIL 0XCA


extern const AP_HAL::HAL& hal;

// constructor
AP_SAW::AP_SAW(void)
{
    _port = NULL;
    _rx_step = 0;
}


// init - perform require initialisation including detecting which protocol to
// use
void AP_SAW::init(const AP_SerialManager& serial_manager)
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

    fcu_to_saw_union.fcu_to_saw_struct.header[0] = FCU_to_SAW_struct_HEADER_1;
    fcu_to_saw_union.fcu_to_saw_struct.header[1] = FCU_to_SAW_struct_HEADER_2;
    fcu_to_saw_union.fcu_to_saw_struct.tail = FCU_to_SAW_struct_TAIL;
    fcu_to_saw_union.fcu_to_saw_struct.len = sizeof(struct FCU_to_SAW_struct);
}

void AP_SAW::update()
{
    if (_port == NULL)
        return;

    Send();
    Receive();
}

void AP_SAW::Receive(void)
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
                count = 0;
            }
            break;

        case 1:
            if (data == 0xAC) {
                receive_buff[count++] = data;
                _rx_step = 2;
            } else  {
                _rx_step = 0;
                count = 0;
            }
            break;

        case 2:
            if (data == sizeof(struct SAW_to_FCU_struct)) {
                receive_buff[count++] = data;
                _rx_step = 3;
            } else  {
                _rx_step = 0;
                count = 0;
            }
            break;

        case 3:
            receive_buff[count++] = data;
            if (count >= (sizeof(struct SAW_to_FCU_struct) - 1)) {
                _rx_step = 4;
                count = 0;
            }
            break;

        case 4:
            if (data == SAW_to_FCU_struct_TAIL) {
                receive_buff[count++] = data;
                memcpy(saw_to_fcu_union.bits, receive_buff, sizeof(struct SAW_to_FCU_struct));

                gcs().send_text(MAV_SEVERITY_INFO,"saw_state=%d\n", saw_to_fcu_union.saw_to_fcu_struct.saw_state);

            }
            _rx_step = 0;
            count = 0;
        
            break;

        default:
            _rx_step = 0;
            count = 0;
            break;
        }
    }
}

void AP_SAW::Send(void)
{
    if (_port == NULL)
        return;

    fcu_to_saw_union.fcu_to_saw_struct.len = sizeof(struct FCU_to_SAW_struct);

    _port->write(fcu_to_saw_union.bits, sizeof(struct FCU_to_SAW_struct));
}
