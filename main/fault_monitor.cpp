#include "fault_monitor.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static fault_event_t events[FAULT_MAX_EVENTS];
static int numEvents = 0;
static uint8_t lastIds[FAULT_MAX_IDS];
static uint32_t lastTimestamps[FAULT_MAX_IDS];

static uint32_t nowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void fault_monitor_add(uint8_t deviceId, uint8_t errorCode)
{
    taskENTER_CRITICAL(&mux);

    uint32_t t = nowMs();

    /* Deduplicar por id: si este id ya salio hace menos de FAULT_MSG_REPEAT_MS, se ignora. */
    for (int i = 0; i < FAULT_MAX_IDS; i++) {
        if (lastTimestamps[i] != 0 && lastIds[i] == deviceId &&
            (t - lastTimestamps[i]) < FAULT_MSG_REPEAT_MS) {
            taskEXIT_CRITICAL(&mux);
            return;
        }
    }

    int slot = -1;
    for (int i = 0; i < FAULT_MAX_IDS; i++) {
        if (lastIds[i] == deviceId) { slot = i; break; }
    }
    if (slot < 0) {
        for (int i = 0; i < FAULT_MAX_IDS; i++) {
            if (lastTimestamps[i] == 0) { slot = i; break; }
        }
    }
    if (slot < 0) slot = 0;
    lastIds[slot] = deviceId;
    lastTimestamps[slot] = t;

    if (numEvents < FAULT_MAX_EVENTS) {
        events[numEvents].deviceId = deviceId;
        events[numEvents].errorCode = errorCode;
        events[numEvents].timestampMs = t;
        numEvents++;
    } else {
        memmove(&events[0], &events[1], sizeof(fault_event_t) * (FAULT_MAX_EVENTS - 1));
        events[FAULT_MAX_EVENTS - 1].deviceId = deviceId;
        events[FAULT_MAX_EVENTS - 1].errorCode = errorCode;
        events[FAULT_MAX_EVENTS - 1].timestampMs = t;
    }

    taskEXIT_CRITICAL(&mux);
}

int fault_monitor_get(fault_event_t *out, int max)
{
    int n = 0;
    taskENTER_CRITICAL(&mux);
    n = (numEvents < max) ? numEvents : max;
    for (int i = 0; i < n; i++)
        out[i] = events[i];
    taskEXIT_CRITICAL(&mux);
    return n;
}

void fault_monitor_clear(void)
{
    taskENTER_CRITICAL(&mux);
    numEvents = 0;
    for (int i = 0; i < FAULT_MAX_IDS; i++)
        lastTimestamps[i] = 0;
    taskEXIT_CRITICAL(&mux);
}