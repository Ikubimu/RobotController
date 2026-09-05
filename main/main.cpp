#include <stdio.h>
#include <string.h>
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
    struct tm ti = {};
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

static void watchdog_task(void *arg)
{
    while (1) {
        CommunicationHandler::sendWatchdog();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

extern "C" void app_main(void)
{
    wifi_init_ap();

    xEventGroupWaitBits(wifi_get_event_group(), WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("AP_DEF"), &ip);
    ESP_LOGI(TAG, "WiFi AP IP: " IPSTR, IP2STR(&ip.ip));

    sync_time();

    StateMachine &sm = StateMachine::get();

    init_web_server();

    CommunicationHandler::start(0);

    CommunicationHandler::registerJointService(CMD_STATUS, [](const CAN_Message *msg) {
        uint8_t jointId = (msg->id >> 8) & 0xFF;
        float pos, vel;
        memcpy(&pos, &msg->data[0], sizeof(float));
        memcpy(&vel, &msg->data[4], sizeof(float));
        arm.updateJoint(jointId, vel, pos);
    });

    CommunicationHandler::registerJointService(CMD_CALIBRATION, [](const CAN_Message *msg) {
        uint8_t jointId = (msg->id >> 8) & 0xFF;
        arm.setJointCalibrated(jointId, true);
    });

    xTaskCreate(watchdog_task, "watchdog", 2048, NULL, 1, NULL);

    while (1)
    {
        sm.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}