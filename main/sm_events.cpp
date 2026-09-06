#include "sm_events.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t pending = 0;
static volatile bool ready = false;

static uint8_t eventMask(SmEvent ev)
{
    return (uint8_t)(1u << ((uint8_t)ev - 1));
}

void sm_post(SmEvent ev)
{
    if (ev == SmEvent::NONE) return;
    taskENTER_CRITICAL(&mux);
    pending |= eventMask(ev);
    taskEXIT_CRITICAL(&mux);
}

bool sm_consume(SmEvent ev)
{
    if (ev == SmEvent::NONE) return false;
    uint8_t mask = eventMask(ev);
    bool present;
    taskENTER_CRITICAL(&mux);
    present = (pending & mask) != 0;
    pending &= ~mask;
    taskEXIT_CRITICAL(&mux);
    return present;
}

void sm_setReady(bool r)
{
    taskENTER_CRITICAL(&mux);
    ready = r;
    taskEXIT_CRITICAL(&mux);
}

bool sm_isReady()
{
    bool r;
    taskENTER_CRITICAL(&mux);
    r = ready;
    taskEXIT_CRITICAL(&mux);
    return r;
}

void sm_clearAll()
{
    taskENTER_CRITICAL(&mux);
    pending = 0;
    taskEXIT_CRITICAL(&mux);
}