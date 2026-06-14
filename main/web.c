#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "cJSON.h"
#include "can_msg_queue.h"

static const char *TAG = "web";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t root_get_handler(httpd_req_t *req)
{
    size_t len = index_html_end - index_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, len);
    return ESP_OK;
}

static esp_err_t messages_get_handler(httpd_req_t *req)
{
    can_msg_t msgs[MAX_CAN_MSG];
    int count = can_msg_queue_get(msgs, MAX_CAN_MSG);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "[");

    for (int i = 0; i < count; i++) {
        char buf[256];
        int off = 0;
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        off += snprintf(buf + off, sizeof(buf) - off,
            "{\"id\":\"0x%lX\",\"dlc\":%d,\"data\":\"", msgs[i].id, msgs[i].dlc);
        for (int j = 0; j < msgs[i].dlc; j++) {
            off += snprintf(buf + off, sizeof(buf) - off, "%02X ", msgs[i].data[j]);
        }
        off += snprintf(buf + off, sizeof(buf) - off, "\",\"t\":%ld}", (long)msgs[i].timestamp);
        httpd_resp_sendstr_chunk(req, buf);
    }

    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t send_post_handler(httpd_req_t *req)
{
    char buf[256];
    int len = req->content_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    httpd_req_recv(req, buf, len);
    buf[len] = 0;

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"json invalido\"}");
        return ESP_OK;
    }

    cJSON *id_item = cJSON_GetObjectItem(json, "id");
    cJSON *pos_item = cJSON_GetObjectItem(json, "pos");
    cJSON *vel_item = cJSON_GetObjectItem(json, "vel");

    if (!id_item || !pos_item || !vel_item) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan campos\"}");
        return ESP_OK;
    }

    uint32_t can_id = (uint32_t)id_item->valueint;
    float position = (float)pos_item->valuedouble;
    float velocity = (float)vel_item->valuedouble;

    cJSON_Delete(json);

    twai_message_t msg = {
        .identifier = can_id,
        .extd = 0,
        .rtr = 0,
        .data_length_code = 8,
    };

    memcpy(&msg.data[0], &position, 4);
    memcpy(&msg.data[4], &velocity, 4);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));

    httpd_resp_set_type(req, "application/json");
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "TX 0x%lX pos=%.2f vel=%.2f", can_id, position, velocity);
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "TX fallo 0x%lX err=%s", can_id, esp_err_to_name(err));
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"tx fallo\"}");
    }
    return ESP_OK;
}

static esp_err_t send_cal_post_handler(httpd_req_t *req)
{
    char buf[256];
    int len = req->content_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    httpd_req_recv(req, buf, len);
    buf[len] = 0;

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"json invalido\"}");
        return ESP_OK;
    }

    cJSON *id_item = cJSON_GetObjectItem(json, "id");
    cJSON *pos_item = cJSON_GetObjectItem(json, "pos");
    cJSON *ratio_item = cJSON_GetObjectItem(json, "ratio");

    if (!id_item || !pos_item || !ratio_item) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan campos\"}");
        return ESP_OK;
    }

    uint32_t can_id = (uint32_t)id_item->valueint;
    float position = (float)pos_item->valuedouble;
    uint16_t ratio = (uint16_t)ratio_item->valueint;

    cJSON_Delete(json);

    twai_message_t msg = {
        .identifier = can_id,
        .extd = 0,
        .rtr = 0,
        .data_length_code = 6,
    };

    memcpy(&msg.data[0], &position, 4);
    memcpy(&msg.data[4], &ratio, 2);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));

    httpd_resp_set_type(req, "application/json");
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CAL 0x%lX pos=%.2f ratio=%u", can_id, position, ratio);
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "CAL fallo 0x%lX err=%s", can_id, esp_err_to_name(err));
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"tx fallo\"}");
    }
    return ESP_OK;
}

void init_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri = "/", .method = HTTP_GET, .handler = root_get_handler
        });
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri = "/api/messages", .method = HTTP_GET, .handler = messages_get_handler
        });
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri = "/api/send", .method = HTTP_POST, .handler = send_post_handler
        });
        httpd_register_uri_handler(server, &(httpd_uri_t){
            .uri = "/api/send_cal", .method = HTTP_POST, .handler = send_cal_post_handler
        });
        ESP_LOGI(TAG, "Servidor HTTP iniciado en puerto 80");
    } else {
        ESP_LOGE(TAG, "Error al iniciar servidor HTTP en puerto 80");
    }
}
