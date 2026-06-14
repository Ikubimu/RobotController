#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "communication_handler.hpp"
#include "can_msg_queue.h"

static const char *TAG = "CAN";

CommunicationHandler::ServiceEntry CommunicationHandler::services[MAX_SERVICES] = {};
uint8_t CommunicationHandler::numServices = 0;
bool CommunicationHandler::initialized = false;
uint8_t CommunicationHandler::deviceId = 0;

bool CommunicationHandler::start(uint8_t device_id)
{
    if (initialized) return true;
    initialized = true;
    deviceId = device_id;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)21, (gpio_num_t)22, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Error al instalar TWAI");
        return false;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar TWAI");
        return false;
    }

    ESP_LOGI(TAG, "CAN iniciado (device_id=%u)", device_id);
    can_msg_queue_init();

    return xTaskCreate(taskFunction, "CAN", 3072, NULL, 1, NULL) == pdPASS;
}

bool CommunicationHandler::registerService(uint16_t canId,
        std::function<void(const CAN_Message*)> callback)
{
    if (numServices >= MAX_SERVICES)
        return false;
    services[numServices] = {canId, std::move(callback)};
    numServices++;
    ESP_LOGI(TAG, "Servicio registrado para ID 0x%03X", canId);
    return true;
}

bool CommunicationHandler::sendMessage(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = dlc;
    for (uint8_t i = 0; i < dlc && i < 8; i++)
        msg.data[i] = data[i];

    return twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK;
}

void CommunicationHandler::taskFunction(void *pvParameters)
{
    (void)pvParameters;
    run();
}

void CommunicationHandler::run()
{
    for (;;) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            can_msg_queue_push(&rx_msg);

            CAN_Message msg = {};
            msg.id = rx_msg.identifier;
            msg.dlc = rx_msg.data_length_code;
            for (uint8_t i = 0; i < rx_msg.data_length_code && i < 8; i++)
                msg.data[i] = rx_msg.data[i];

            bool handled = false;
            for (uint8_t i = 0; i < numServices; i++) {
                if (services[i].id == msg.id) {
                    services[i].callback(&msg);
                    handled = true;
                    break;
                }
            }

            if (!handled) {
                printf("CAN RX (unhandled): ID=0x%03lX DLC=%u Data=",
                       msg.id, msg.dlc);
                for (uint8_t i = 0; i < msg.dlc; i++)
                    printf("%02X ", msg.data[i]);
                printf("\r\n");

                if (sendMessage(msg.id, msg.data, msg.dlc))
                    printf("CAN TX echo OK\r\n");
                else
                    printf("CAN TX echo ERROR\r\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
