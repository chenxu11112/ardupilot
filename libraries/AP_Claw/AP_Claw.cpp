#include "AP_Claw.h"

#include <stdio.h> // for sprintf

#define AP_SERIALMANAGER_CLAW_BAUD 115200
#define AP_SERIALMANAGER_CLAW_BUFSIZE_RX 64
#define AP_SERIALMANAGER_CLAW_BUFSIZE_TX 64

extern const AP_HAL::HAL &hal;

// constructor
AP_Claw::AP_Claw(void)
{
    _port = NULL;
}

// init - perform require initialisation including detecting which protocol to use
void AP_Claw::init(const AP_SerialManager &serial_manager)
{
    // check for DEVO_DPort
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_CLAW, 0)))
    {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise uart
        _port->begin(AP_SERIALMANAGER_CLAW_BAUD, AP_SERIALMANAGER_CLAW_BUFSIZE_RX, AP_SERIALMANAGER_CLAW_BUFSIZE_TX);
    }
}

void AP_Claw::update()
{
    if (_port == NULL)
        return;

    uint16_t chan_value = hal.rcin->read(CH_7);

    if (chan_value > 1700)
    {
        send_switch(1);
    }
    else
    {
        send_switch(0);
    }
}

void AP_Claw::send_switch(uint8_t is_close)
{
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = is_close;

    data_to_send[3] = _cnt - 4;

    uint8_t sum = 0;
    for (uint8_t i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    _port->write(data_to_send, _cnt);
}
