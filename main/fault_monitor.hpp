#ifndef FAULT_MONITOR_HPP
#define FAULT_MONITOR_HPP

#include <stdint.h>

#define FAULT_MSG_REPEAT_MS 10000
#define FAULT_MAX_EVENTS 32
#define FAULT_MAX_IDS 8

typedef struct {
    uint8_t deviceId;
    uint8_t errorCode;
    uint32_t timestampMs;
} fault_event_t;

void fault_monitor_add(uint8_t deviceId, uint8_t errorCode);
int fault_monitor_get(fault_event_t *out, int max);
void fault_monitor_clear();

#endif