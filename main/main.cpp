#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "wifi.h"
#include "web.h"
#include "esp_netif.h"
#include "state_machine.hpp"
#include "communication_handler.hpp"
#include "Arm.hpp"

static const char *TAG = "CAN";
Arm arm(NUM_JOINTS);

static void sync_time(void)
{
    ESP_LOGI(TAG, "Sincronizando hora via NTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t now = 0;
    struct tm ti = {0};
    int retry = 0;
    while (ti.tm_year < (2024 - 1900) && retry < 20) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        time(&now);
        localtime_r(&now, &ti);
        retry++;
    }
    if (ti.tm_year >= (2024 - 1900)) {
        ESP_LOGI(TAG, "Hora sincronizada: %s", asctime(&ti));
    } else {
        ESP_LOGW(TAG, "No se pudo sincronizar hora NTP");
    }
}

extern "C" void app_main(void)
{
    wifi_init_sta();

    xEventGroupWaitBits(wifi_get_event_group(), WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("STA_DEF"), &ip);
    ESP_LOGI(TAG, "WiFi IP: " IPSTR, IP2STR(&ip.ip));

    sync_time();

    StateMachine &sm = StateMachine::get();

    init_web_server();

    CommunicationHandler::start(0);

    while (1)
    {
        sm.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}