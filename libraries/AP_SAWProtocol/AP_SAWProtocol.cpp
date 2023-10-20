

#include "AP_SAWProtocol.h"
#include "stdio.h"
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define AP_SERIALMANAGER_SAW_BAUD       115200
#define AP_SERIALMANAGER_SAW_BUFSIZE_RX 64
#define AP_SERIALMANAGER_SAW_BUFSIZE_TX 64

extern const AP_HAL::HAL& hal;

// constructor
AP_SAWProtocol::AP_SAWProtocol(void)
{
    _port = NULL;
}

// init - perform require initialisation including detecting which protocol to
// use
void AP_SAWProtocol::init(const AP_SerialManager& serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_SAW, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_SAW_BAUD,
                     AP_SERIALMANAGER_SAW_BUFSIZE_RX,
                     AP_SERIALMANAGER_SAW_BUFSIZE_TX);
    }

    memset(&send_HeartBeat, 0, sizeof(send_HeartBeat));
    memset(&send_SwitchStatus, 0, sizeof(send_SwitchStatus));

    memset(&recv_HeartBeat, 0, sizeof(recv_HeartBeat));
    memset(&recv_SawStatus, 0, sizeof(recv_SawStatus));
    memset(&recv_Powerstatus, 0, sizeof(recv_Powerstatus));
}

void AP_SAWProtocol::update()
{
    if (_port == NULL)
        return;

    Send();
    Receive();
}

void AP_SAWProtocol::Receive(void)
{
    if (_port == NULL)
        return;

    uint8_t numc = _port->available();
    uint8_t data;
    for (uint8_t i = 0; i < numc; i++) {
        data = _port->read();
        parse_char(data);
    }
}

void AP_SAWProtocol::parse_char(uint8_t data)
{
    static uint8_t _rx_step     = 0;
    static uint8_t _rx_count    = 0;
    static uint8_t _rx_data_len = 0;

    recv_buffer[_rx_count++] = data;

    switch (_rx_step) {
    case Protocol_Header_1:
        if (data == LOWBYTE(PROTOCOL_HEADER)) {
            _rx_step = Protocol_Header_2;
        } else {
            _rx_count = _rx_step = 0;
        }
        break;

    case Protocol_Header_2:
        if (data == HIGHBYTE(PROTOCOL_HEADER)) {
            _rx_step = Protocol_Equipment;
        } else {
            _rx_count = _rx_step = 0;
        }
        break;

    case Protocol_Equipment:
        _rx_step = Protocol_ID;
        break;

    case Protocol_ID:
        _rx_step = Protocol_LENGTH;
        break;

    case Protocol_LENGTH:
        if (data <= 0) {
            _rx_count = _rx_step = 0;
        } else {
            _rx_data_len = recv_buffer[4] - 5;
            _rx_step     = Protocol_Payload;
        }
        break;

    case Protocol_Payload:
        _rx_data_len--;
        if (_rx_data_len == 0) {
            recv_decode(recv_buffer, _rx_count);
            _rx_count = _rx_step = 0;
        }
        break;

    default:
        _rx_count = _rx_step = 0;
    }
}

void AP_SAWProtocol::recv_decode(uint8_t* data, uint16_t len)
{
    if (crc_fletcher16(recv_buffer, len - 2) != UINT16_VALUE(data[len - 1], data[len - 2])) {
        return;
    }

    if (data[3] == HEARTBEAT_ID) {
        recv_heartbeat = true;
        memcpy(recv_HeartBeat.bits, data, sizeof(recv_HeartBeat));

    } else if (data[3] == SAWSTATUS_ID) {
        memcpy(recv_SawStatus.bits, data, sizeof(recv_SawStatus));

    } else if (data[3] == POWERSTATUS_ID) {
        memcpy(recv_Powerstatus.bits, data, sizeof(recv_Powerstatus));
    }
}

void AP_SAWProtocol::Send(void)
{
    if (_port == NULL)
        return;

    static uint16_t count_tick = 0;

    const uint8_t send_heartbeat_tick    = 8;
    const uint8_t send_switchstatus_tick = 3;
    const uint8_t send_throttle_tick     = 2;

    count_tick++;
    if (count_tick % send_heartbeat_tick == (send_heartbeat_tick - 1)) {
        send_heartbeat();
        checkConnected();
    }

    if (recv_heartbeat == false) {
        return;
    }

    if (count_tick % send_switchstatus_tick == (send_switchstatus_tick - 1)) {
        send_switchstatus();
    }

    if (count_tick % send_throttle_tick == (send_throttle_tick - 1)) {
        send_throttle();
    }
}

void AP_SAWProtocol::send_heartbeat(void)
{
    send_HeartBeat.data.header    = PROTOCOL_HEADER;
    send_HeartBeat.data.equipment = PROTOCOL_EQUIPMENT_ID;
    send_HeartBeat.data.id        = HEARTBEAT_ID;
    send_HeartBeat.data.length    = sizeof(send_HeartBeat);
    send_HeartBeat.data.timestamp = AP_HAL::millis();

    send_HeartBeat.data.crcsum = crc_fletcher16(send_HeartBeat.bits, sizeof(send_HeartBeat) - 2);

    _port->write(send_HeartBeat.bits, sizeof(send_HeartBeat));
}

void AP_SAWProtocol::send_switchstatus(void)
{
    send_SwitchStatus.data.header    = PROTOCOL_HEADER;
    send_SwitchStatus.data.equipment = PROTOCOL_EQUIPMENT_ID;
    send_SwitchStatus.data.id        = SWITCHSTATUS_ID;
    send_SwitchStatus.data.length    = sizeof(send_SwitchStatus);
    send_SwitchStatus.data.hang_open = (hal.rcin->read(CH_7) > 1700) ? true : false;
    send_SwitchStatus.data.saw_open  = (hal.rcin->read(CH_8) > 1700) ? true : false;

    send_SwitchStatus.data.crcsum = crc_fletcher16(send_SwitchStatus.bits, sizeof(send_SwitchStatus) - 2);

    _port->write(send_SwitchStatus.bits, sizeof(send_SwitchStatus));
}

void AP_SAWProtocol::send_throttle(void)
{
    send_Throttle.data.header        = PROTOCOL_HEADER;
    send_Throttle.data.equipment     = PROTOCOL_EQUIPMENT_ID;
    send_Throttle.data.id            = THROTTLE_ID;
    send_Throttle.data.length        = sizeof(send_Throttle);
    send_Throttle.data.throttle_pwm  = 60;
    send_Throttle.data.current_limit = 50;

    send_Throttle.data.crcsum = crc_fletcher16(send_Throttle.bits, sizeof(send_Throttle) - 2);

    _port->write(send_Throttle.bits, sizeof(send_Throttle));
}

void AP_SAWProtocol::checkConnected()
{
    if (last_get_tick != recv_HeartBeat.data.timestamp) {
        if ((recv_HeartBeat.data.timestamp - last_get_tick) < 10000) {
            recv_heartbeat = true;
        } else {
            recv_heartbeat = false;
        }
    } else {
        recv_heartbeat = false;
    }
    last_get_tick = recv_HeartBeat.data.timestamp;
}