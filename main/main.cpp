#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "wifi.h"
#include "web.h"
#include "can_msg_queue.h"
#include "esp_netif.h"
#include "state_machine.hpp"

#define CAN_TX GPIO_NUM_21
#define CAN_RX GPIO_NUM_22

static const char *TAG = "CAN";

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

void app_main(void)
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

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX,
        CAN_RX,
        TWAI_MODE_NORMAL
    );

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "CAN listo");
    can_msg_queue_init();

    int64_t lastTx = 0;

    while (1)
    {
        // 🟢 tiempo en microsegundos
        // int64_t now = esp_timer_get_time();

        // if ((now - lastTx) >= 1000000) // 1 segundo = 1,000,000 µs
        // {
        //     lastTx = now;

        //     twai_status_info_t status;
        //     twai_get_status_info(&status);

        //     // printf("State: %d, TX error: %ld, RX error: %ld\n",
        //     //     status.state,
        //     //     status.tx_error_counter,
        //     //     status.rx_error_counter);

        //     twai_message_t msg = {
        //         .identifier = 0x123,
        //         .extd = 0,
        //         .rtr = 0,
        //         .data_length_code = 8,
        //         .data = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88}
        //     };

        //     esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(1000));

        //     if (err == ESP_OK) {
        //         // ESP_LOGI(TAG, "CAN TX enviado");
        //     } else {
        //         // ESP_LOGE(TAG, "Error TX CAN");
        //     }
        // }

        sm.update();

        twai_message_t rx_msg;

        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK)
        {
            can_msg_queue_push(&rx_msg);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}