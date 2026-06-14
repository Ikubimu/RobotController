#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_SSID      "PuntoAP"
#define WIFI_PASS      "12345678"

#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

void wifi_init_sta(void);
EventGroupHandle_t wifi_get_event_group(void);

#endif
