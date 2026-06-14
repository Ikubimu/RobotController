#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "communication_handler.hpp"
#include "cJSON.h"
#include "can_msg_queue.h"
#include "web.h"
#include "joints_storage.h"

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

    uint8_t data[8] = {0};
    memcpy(&data[0], &position, 4);
    memcpy(&data[4], &velocity, 4);

    bool ok = CommunicationHandler::sendMessage(can_id, data, 8);

    httpd_resp_set_type(req, "application/json");
    if (ok) {
        ESP_LOGI(TAG, "TX 0x%lX pos=%.2f vel=%.2f", can_id, position, velocity);
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "TX fallo 0x%lX", can_id);
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"tx fallo\"}");
    }
    return ESP_OK;
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"numJoints\":%d}", NUM_JOINTS);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t control_post_handler(httpd_req_t *req)
{
    char buf[128];
    int len = req->content_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    httpd_req_recv(req, buf, len);
    buf[len] = 0;

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false}");
        return ESP_OK;
    }

    cJSON *dir_item = cJSON_GetObjectItem(json, "dir");
    if (dir_item && dir_item->valuestring) {
        ESP_LOGI(TAG, "Control: %s", dir_item->valuestring);
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":true}");
        return ESP_OK;
    }

    cJSON *joint_item = cJSON_GetObjectItem(json, "joint");
    cJSON *angle_item = cJSON_GetObjectItem(json, "angle");

    if (!joint_item || !angle_item) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan campos\"}");
        return ESP_OK;
    }

    int joint = joint_item->valueint;
    float angle = (float)angle_item->valuedouble;
    cJSON_Delete(json);

    uint32_t can_id = CAN_BASE_ID + joint;
    uint8_t data[8] = {0};
    float velocity = 0;
    memcpy(&data[0], &angle, 4);
    memcpy(&data[4], &velocity, 4);

    bool ok = CommunicationHandler::sendMessage(can_id, data, 8);

    httpd_resp_set_type(req, "application/json");
    if (ok) {
        ESP_LOGI(TAG, "Joint %d angle=%.2f -> CAN 0x%lX", joint, angle, can_id);
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "TX fallo joint %d", joint);
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

    uint8_t data[8] = {0};
    memcpy(&data[0], &position, 4);
    memcpy(&data[4], &ratio, 2);

    bool ok = CommunicationHandler::sendMessage(can_id, data, 6);

    httpd_resp_set_type(req, "application/json");
    if (ok) {
        ESP_LOGI(TAG, "CAL 0x%lX pos=%.2f ratio=%u", can_id, position, ratio);
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        ESP_LOGE(TAG, "CAL fallo 0x%lX", can_id);
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"tx fallo\"}");
    }
    return ESP_OK;
}

static esp_err_t points_get_handler(httpd_req_t *req)
{
    char buf[512];
    int off = 0;
    off += snprintf(buf + off, sizeof(buf) - off, "{\"count\":%d,\"points\":[", joints_get_count());
    for (int i = 0; i < joints_get_count(); i++) {
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        const joint_point_t *p = joints_get(i);
        off += snprintf(buf + off, sizeof(buf) - off, "[");
        for (int j = 0; j < NUM_JOINTS; j++) {
            if (j > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
            off += snprintf(buf + off, sizeof(buf) - off, "%.1f", p->angles[j]);
        }
        off += snprintf(buf + off, sizeof(buf) - off, "]");
    }
    off += snprintf(buf + off, sizeof(buf) - off, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t points_post_handler(httpd_req_t *req)
{
    char buf[512];
    int len = req->content_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    httpd_req_recv(req, buf, len);
    buf[len] = 0;

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false}");
        return ESP_OK;
    }

    cJSON *angles = cJSON_GetObjectItem(json, "angles");
    if (!angles || !cJSON_IsArray(angles)) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan angles\"}");
        return ESP_OK;
    }

    float arr[NUM_JOINTS];
    int n = cJSON_GetArraySize(angles);
    if (n > NUM_JOINTS) n = NUM_JOINTS;
    for (int i = 0; i < n; i++) {
        cJSON *item = cJSON_GetArrayItem(angles, i);
        arr[i] = item ? (float)item->valuedouble : 0;
    }
    cJSON_Delete(json);

    int idx = joints_add(arr);
    httpd_resp_set_type(req, "application/json");
    if (idx >= 0) {
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"lleno\"}");
    }
    return ESP_OK;
}

static esp_err_t points_load_handler(httpd_req_t *req)
{
    char buf[128];
    int len = req->content_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    httpd_req_recv(req, buf, len);
    buf[len] = 0;

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false}");
        return ESP_OK;
    }

    cJSON *idx_item = cJSON_GetObjectItem(json, "index");
    if (!idx_item) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan index\"}");
        return ESP_OK;
    }

    int idx = idx_item->valueint;
    cJSON_Delete(json);

    const joint_point_t *p = joints_get(idx);
    if (!p) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"indice invalido\"}");
        return ESP_OK;
    }

    for (int i = 0; i < NUM_JOINTS; i++) {
        uint32_t can_id = CAN_BASE_ID + i;
        uint8_t data[8] = {0};
        float velocity = 0;
        memcpy(&data[0], &p->angles[i], 4);
        memcpy(&data[4], &velocity, 4);
        CommunicationHandler::sendMessage(can_id, data, 8);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t points_delete_handler(httpd_req_t *req)
{
    char buf[128];
    int len = req->content_len;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    httpd_req_recv(req, buf, len);
    buf[len] = 0;

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false}");
        return ESP_OK;
    }

    cJSON *idx_item = cJSON_GetObjectItem(json, "index");
    if (!idx_item) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan index\"}");
        return ESP_OK;
    }

    int idx = idx_item->valueint;
    cJSON_Delete(json);

    int ok = joints_delete(idx);
    httpd_resp_set_type(req, "application/json");
    if (ok == 0) {
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"indice invalido\"}");
    }
    return ESP_OK;
}

void init_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

#define REG_URI(uri, method, handler) do { \
        httpd_uri_t u = {.uri = uri, .method = method, .handler = handler}; \
        httpd_register_uri_handler(server, &u); \
    } while(0)

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        auto reg = [&server](const char* uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*)) {
            httpd_uri_t u = {};
            u.uri = uri;
            u.method = method;
            u.handler = handler;
            httpd_register_uri_handler(server, &u);
        };
        reg("/", HTTP_GET, root_get_handler);
        reg("/api/messages", HTTP_GET, messages_get_handler);
        reg("/api/send", HTTP_POST, send_post_handler);
        reg("/api/send_cal", HTTP_POST, send_cal_post_handler);
        reg("/api/config", HTTP_GET, config_get_handler);
        reg("/api/control", HTTP_POST, control_post_handler);
        reg("/api/points", HTTP_GET, points_get_handler);
        reg("/api/points", HTTP_POST, points_post_handler);
        reg("/api/points/load", HTTP_POST, points_load_handler);
        reg("/api/points/delete", HTTP_POST, points_delete_handler);
        ESP_LOGI(TAG, "Servidor HTTP iniciado en puerto 80");
    } else {
        ESP_LOGE(TAG, "Error al iniciar servidor HTTP en puerto 80");
    }
}
