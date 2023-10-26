#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>


class AP_SAWProtocol {
public:
    AP_SAWProtocol();

    /* Do not allow copies */
    AP_SAWProtocol(const AP_SAWProtocol& other) = delete;
    AP_SAWProtocol& operator=(const AP_SAWProtocol&) = delete;

    // init - perform require initialisation including detecting which protocol
    // to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    void update(void);

    void Receive(void);

    void Send(void);

    struct PACKED FCU_to_SAW_struct {
        uint8_t MSG_OP;
        uint8_t OP_IS_START;
        uint8_t MSG_DP;
        uint8_t MSG_DP_IS_ON;
        uint8_t THROTTLE_CMD;
        uint8_t THROTTLE_Value;
        uint8_t MSG_SC_DATA;
        uint8_t LIMIT_CMD;
        uint8_t CURRENT_LIMI;

        uint8_t zeros[11];
    };

    union FCU_to_SAW_Union {
        struct FCU_to_SAW_struct fcu_to_saw_struct;
        uint8_t bits[sizeof(struct FCU_to_SAW_struct)];
    };

    struct PACKED SAW_to_FCU_struct {
        uint8_t MSG_OP;
        uint8_t OP_IS_START;
        uint8_t MSG_DP;
        uint8_t MSG_DP_IS_ON;
        uint8_t THROTTLE_CMD;
        uint8_t THROTTLE_Value;
        uint8_t MSG_SC_DATA;
        uint8_t LIMIT_CMD;
        uint8_t CURRENT_LIMI;

        uint16_t   Voltage;
        uint16_t   SC_Current[4];        //刀具电流
        uint16_t   ADC_VOL[2];           //电流传感器ADC采样        
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

    uint8_t receive_buff[50];
};