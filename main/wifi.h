#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_AP_SSID   "RobotController"
#define WIFI_AP_PASS   "12345678"

#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

#ifdef __cplusplus
extern "C" {
#endif

void wifi_init_ap(void);
EventGroupHandle_t wifi_get_event_group(void);

#ifdef __cplusplus
}
#endif

#endif
