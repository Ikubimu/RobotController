#ifndef COMMUNICATION_HANDLER_HPP
#define COMMUNICATION_HANDLER_HPP

#include <stdint.h>
#include <functional>
#include "driver/twai.h"

#define MAX_SERVICES 8
#define MAX_JOINT_SERVICES 8
#define MAX_JOINTS 6


// CAN command IDs
#define CMD_ERROR        0x01
#define CMD_START        0x02
#define CMD_CALIBRATION  0x03
#define CMD_ANNOUNCE     0x04
#define CMD_MOVE_TARGET  0x05
#define CMD_PAUSE        0x06
#define CMD_RESUME       0x07
#define CMD_STATUS       0x08

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
} CAN_Message;

class CommunicationHandler {
public:
    CommunicationHandler() = delete;

    static bool start(uint8_t device_id);
    static bool registerService(uint16_t canId,
                                std::function<void(const CAN_Message*)> callback);
    static bool registerJointService(uint8_t cmdId,
                                     std::function<void(const CAN_Message*)> callback);
    static bool sendMessage(uint32_t id, const uint8_t *data, uint8_t dlc);

private:
    struct ServiceEntry {
        uint16_t id;
        std::function<void(const CAN_Message*)> callback;
    };

    static void taskFunction(void *pvParameters);
    static void run();

    static ServiceEntry services[MAX_SERVICES];
    static ServiceEntry jointServices[MAX_JOINT_SERVICES];
    static uint8_t numServices;
    static uint8_t numJointServices;
    static bool initialized;
    static uint8_t deviceId;
};

#endif
