#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define PROTOCOL_EQUIPMENT_ID           0x02
#define PROTOCOL_HEADER                 0xAAAF

#define DEFAULT_PRTOCOL_HEARTBEAT_TICK  50
#define DEFAULT_PRTOCOL_NODESTATUS_TICK 20
#define DEFAULT_PRTOCOL_POWER_TICK      10

enum Protocol_ID {
    HEARTBEAT_ID = 1,
    SAWSTATUS_ID,
    POWERSTATUS_ID,
    SWITCHSTATUS_ID,
    THROTTLE_ID,
};

enum Protocol_Parse_STEP {
    Protocol_Header_1 = 0,
    Protocol_Header_2,
    Protocol_Equipment,
    Protocol_ID,
    Protocol_LENGTH,
    Protocol_Payload,
    Protocol_CRCSUM,
};

struct Protocol_Send_Tick_Struct {
    uint8_t heartbeat;
    uint8_t node_status;
    uint8_t power_status;
};

union PROTOCOL_HEARTBEAT_Union {
    struct PACKED HEARTBEAT_Structure {
        uint16_t header;
        uint8_t  equipment;
        uint8_t  id;
        uint8_t  length;
        uint32_t timestamp;
        uint16_t crcsum;
    } data;

    uint8_t bits[sizeof(HEARTBEAT_Structure)];
};

union PROTOCOL_SAWSTATUS_Union {
    struct PACKED SAWSTATUS_Structure {
        uint16_t header;
        uint8_t  equipment;
        uint8_t  id;
        uint8_t  length;
        uint8_t  fly_status;
        uint16_t crcsum;
    } data;
    uint8_t bits[sizeof(SAWSTATUS_Structure)];
};

union PROTOCOL_POWERSTATUS_Union {
    struct PACKED POWERSTATUS_Structure {
        uint16_t header;
        uint8_t  equipment;
        uint8_t  id;
        uint8_t  length;
        float    voltage;
        float    current[4];
        uint16_t crcsum;
    } data;
    uint8_t bits[sizeof(POWERSTATUS_Structure)];
};

union PROTOCOL_SWITCH_STATUS_Union {
    struct PACKED SWITCH_STATUS_Structure {
        uint16_t header;
        uint8_t  equipment;
        uint8_t  id;
        uint8_t  length;
        uint8_t  hang_open;
        uint8_t  saw_open;
        uint16_t crcsum;
    } data;
    uint8_t bits[sizeof(SWITCH_STATUS_Structure)];
};

union PROTOCOL_THROTTLE_Union {
    struct PACKED THROTTLE_Structure {
        uint16_t header;
        uint8_t  equipment;
        uint8_t  id;
        uint8_t  length;
        uint16_t throttle_pwm;
        uint16_t current_limit;
        uint16_t crcsum;
    } data;
    uint8_t bits[sizeof(THROTTLE_Structure)];
};

class AP_SAWProtocol {
public:
    AP_SAWProtocol();

    /* Do not allow copies */
    AP_SAWProtocol(const AP_SAWProtocol& other)      = delete;
    AP_SAWProtocol& operator=(const AP_SAWProtocol&) = delete;

    // init - perform require initialisation including detecting which protocol
    // to use
    void init(const AP_SerialManager& serial_manager);

    // update flight control mode. The control mode is vehicle type specific
    void update(void);

    void Receive(void);

    void Send(void);

    void send_heartbeat(void);
    void send_switchstatus(void);
    void send_throttle(void);

    void parse_char(uint8_t data);

    void recv_decode(uint8_t* data, uint16_t length);

    void checkConnected();

private:
    AP_HAL::UARTDriver* _port; // UART used to send data to receiver

    union PROTOCOL_HEARTBEAT_Union     send_HeartBeat;
    union PROTOCOL_SWITCH_STATUS_Union send_SwitchStatus;
    union PROTOCOL_THROTTLE_Union      send_Throttle;

    union PROTOCOL_HEARTBEAT_Union   recv_HeartBeat;
    union PROTOCOL_SAWSTATUS_Union   recv_SawStatus;
    union PROTOCOL_POWERSTATUS_Union recv_Powerstatus;

    bool recv_heartbeat;

    uint32_t last_get_tick;

    uint8_t recv_buffer[50];
};