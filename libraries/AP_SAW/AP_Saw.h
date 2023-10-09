#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


class AP_SAW {
public:
    AP_SAW();

    /* Do not allow copies */
    AP_SAW(const AP_SAW& other) = delete;
    AP_SAW& operator=(const AP_SAW&) = delete;

    // init - perform require initialisation including detecting which protocol
    // to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    void update(void);

    void Receive(void);

    void Send(void);

    struct PACKED FCU_to_SAW_struct {
        uint8_t header[2];
        uint8_t len;
        uint8_t is_open;
        uint8_t tail;
    };

    union FCU_to_SAW_Union {
        struct FCU_to_SAW_struct fcu_to_saw_struct;
        uint8_t bits[sizeof(struct FCU_to_SAW_struct)];
    };

    struct PACKED SAW_to_FCU_struct {
        uint8_t header[2];
        uint8_t len;
        uint8_t saw_state;
        int16_t saw_voltage;
        int16_t saw_current;
        uint8_t tail;
    };
    union SAW_to_FCU_Union {
        struct SAW_to_FCU_struct saw_to_fcu_struct;
        uint8_t bits[sizeof(struct SAW_to_FCU_struct)];
    };

private:
    AP_HAL::UARTDriver* _port; // UART used to send data to receiver

    FCU_to_SAW_Union fcu_to_saw_union;
    SAW_to_FCU_Union saw_to_fcu_union;

    uint8_t _rx_step;

    uint8_t receive_buff[20];
};