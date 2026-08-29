#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include "esp_http_server.h"
#include "esp_log.h"
#include "communication_handler.hpp"
#include "cJSON.h"
#include "can_msg_queue.h"
#include "web.h"
#include "joints_storage.h"
#include "Arm.hpp"
#include "Config.hpp"
#include "CinematicsUtils.hpp"

static const char *TAG = "web";
extern Arm arm;

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

static esp_err_t joints_get_handler(httpd_req_t *req)
{
    auto positions = arm.getPos();
    char buf[256];
    int off = 0;
    off += snprintf(buf + off, sizeof(buf) - off, "{\"joints\":[");
    for (size_t i = 0; i < positions.size(); i++) {
        if (i > 0) off += snprintf(buf + off, sizeof(buf) - off, ",");
        off += snprintf(buf + off, sizeof(buf) - off, "%.1f", positions[i]);
    }
    off += snprintf(buf + off, sizeof(buf) - off, "]}");
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
    cJSON *delta_item = cJSON_GetObjectItem(json, "delta");

    if (!joint_item || !delta_item) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan campos\"}");
        return ESP_OK;
    }

    int joint = joint_item->valueint;
    float delta = (float)delta_item->valuedouble;

    auto positions = arm.getPos();
    if (joint < 0 || (size_t)joint >= positions.size()) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"joint invalido\"}");
        return ESP_OK;
    }

    float newDeg = positions[joint] + delta;
    cJSON_Delete(json);

    arm.RotateJoint(joint, newDeg, ARM_DEFAULT_VEL_LIN);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
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
    cJSON *mode_item = cJSON_GetObjectItem(json, "mode");
    if (!idx_item || !mode_item || !mode_item->valuestring) {
        cJSON_Delete(json);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"faltan campos\"}");
        return ESP_OK;
    }

    int idx = idx_item->valueint;
    char mode = mode_item->valuestring[0];
    cJSON_Delete(json);

    if (mode == 'J') {
        arm.MoveJ(idx, ARM_DEFAULT_VEL_ANG);
    } else {
        arm.MoveL(idx, ARM_DEFAULT_VEL_LIN);
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

static esp_err_t fk_get_handler(httpd_req_t *req)
{
    auto angles = arm.getPos();
    Matrix T = Cinematics::computeFromJoints(angles);

    char buf[128];
    snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
        T.data[3], T.data[7], T.data[11]);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t config_validate_post_handler(httpd_req_t *req)
{
    char *buf = (char *)malloc(req->content_len + 1);
    if (!buf) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"memoria insuficiente\"}");
        return ESP_OK;
    }

    int recv = 0;
    while (recv < req->content_len) {
        int r = httpd_req_recv(req, buf + recv, req->content_len - recv);
        if (r <= 0) break;
        recv += r;
    }
    buf[recv] = 0;

    const char *error = nullptr;

    cJSON *json = cJSON_Parse(buf);
    free(buf);

    if (!json) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"JSON invalido\"}");
        return ESP_OK;
    }

    cJSON *dh_item = cJSON_GetObjectItem(json, "DH");
    cJSON *rot_item = cJSON_GetObjectItem(json, "rot");
    cJSON *planar_item = cJSON_GetObjectItem(json, "planar");
    cJSON *cal_item = cJSON_GetObjectItem(json, "calibration");

    if (!dh_item || !cJSON_IsArray(dh_item)) {
        error = "Campo 'DH' requerido (array de arrays)";
    } else {
        int dh_count = cJSON_GetArraySize(dh_item);
        if (dh_count == 0) {
            error = "Campo 'DH' no puede estar vacio";
        } else {
            for (int i = 0; i < dh_count; i++) {
                cJSON *row = cJSON_GetArrayItem(dh_item, i);
                if (!cJSON_IsArray(row) || cJSON_GetArraySize(row) != 4) {
                    error = "Cada 'DH' debe ser un array de 4 elementos (theta, alpha, d, a)";
                    break;
                }
                for (int j = 0; j < 4; j++) {
                    cJSON *v = cJSON_GetArrayItem(row, j);
                    if (!cJSON_IsNumber(v)) {
                        error = "Elementos de 'DH' deben ser numericos";
                        break;
                    }
                }
                if (error) break;
            }
        }
    }

    if (!error && (!rot_item || !cJSON_IsArray(rot_item))) {
        error = "Campo 'rot' requerido (array)";
    }

    if (!error && (!planar_item || !cJSON_IsArray(planar_item))) {
        error = "Campo 'planar' requerido (array)";
    }

    if (!error) {
        if (!cal_item || !cJSON_IsArray(cal_item)) {
            error = "Campo 'calibration' requerido (array)";
        } else {
            int cal_count = cJSON_GetArraySize(cal_item);
            for (int i = 0; i < cal_count; i++) {
                cJSON *entry = cJSON_GetArrayItem(cal_item, i);
                if (!cJSON_IsObject(entry)) {
                    error = "Cada 'calibration' debe ser un objeto";
                    break;
                }
                cJSON *pos = cJSON_GetObjectItem(entry, "pos");
                cJSON *ranges = cJSON_GetObjectItem(entry, "ranges");
                cJSON *ratio = cJSON_GetObjectItem(entry, "ratio");

                if (!cJSON_IsNumber(pos)) {
                    error = "Calibration: campo 'pos' numerico requerido";
                    break;
                }
                if (!cJSON_IsArray(ranges) || cJSON_GetArraySize(ranges) != 2) {
                    error = "Calibration: campo 'ranges' debe ser array de 2 elementos";
                    break;
                }
                for (int j = 0; j < 2; j++) {
                    cJSON *rv = cJSON_GetArrayItem(ranges, j);
                    if (!cJSON_IsNumber(rv)) {
                        error = "Calibration: 'ranges' debe ser numerico";
                        break;
                    }
                }
                if (!error && !cJSON_IsNumber(ratio)) {
                    error = "Calibration: campo 'ratio' numerico requerido";
                    break;
                }
            }
        }
    }

    if (!error) {
        Config::dh.clear();
        int dh_count = cJSON_GetArraySize(dh_item);
        for (int i = 0; i < dh_count; i++) {
            cJSON *row = cJSON_GetArrayItem(dh_item, i);
            DH_values v;
            v.theta = (float)cJSON_GetArrayItem(row, 0)->valuedouble;
            v.alpha = (float)cJSON_GetArrayItem(row, 1)->valuedouble;
            v.d     = (float)cJSON_GetArrayItem(row, 2)->valuedouble;
            v.a     = (float)cJSON_GetArrayItem(row, 3)->valuedouble;
            Config::dh.push_back(v);
        }

        Config::rot.clear();
        int rot_count = cJSON_GetArraySize(rot_item);
        for (int i = 0; i < rot_count; i++) {
            Config::rot.push_back(cJSON_GetArrayItem(rot_item, i)->valueint);
        }

        Config::planar.clear();
        int planar_count = cJSON_GetArraySize(planar_item);
        for (int i = 0; i < planar_count; i++) {
            Config::planar.push_back(cJSON_GetArrayItem(planar_item, i)->valueint);
        }

        Config::calibration.clear();
        int cal_count = cJSON_GetArraySize(cal_item);
        for (int i = 0; i < cal_count; i++) {
            cJSON *entry = cJSON_GetArrayItem(cal_item, i);
            Calibration c;
            c.pos = (float)cJSON_GetObjectItem(entry, "pos")->valuedouble;
            c.ranges[0] = (float)cJSON_GetArrayItem(cJSON_GetObjectItem(entry, "ranges"), 0)->valuedouble;
            c.ranges[1] = (float)cJSON_GetArrayItem(cJSON_GetObjectItem(entry, "ranges"), 1)->valuedouble;
            c.ratio = (float)cJSON_GetObjectItem(entry, "ratio")->valuedouble;
            Config::calibration.push_back(c);
        }

        ESP_LOGI(TAG, "== Config extraida del JSON ==");
        for (size_t i = 0; i < Config::dh.size(); i++) {
            const DH_values &v = Config::dh[i];
            ESP_LOGI(TAG, "DH[%d]: theta=%.2f alpha=%.2f d=%.2f a=%.2f",
                     (int)i, v.theta, v.alpha, v.d, v.a);
        }
        std::string rot_str = "rot: [";
        for (size_t i = 0; i < Config::rot.size(); i++) {
            if (i > 0) rot_str += ", ";
            rot_str += std::to_string(Config::rot[i]);
        }
        rot_str += "]";
        ESP_LOGI(TAG, "%s", rot_str.c_str());

        std::string planar_str = "planar: [";
        for (size_t i = 0; i < Config::planar.size(); i++) {
            if (i > 0) planar_str += ", ";
            planar_str += std::to_string(Config::planar[i]);
        }
        planar_str += "]";
        ESP_LOGI(TAG, "%s", planar_str.c_str());

        for (size_t i = 0; i < Config::calibration.size(); i++) {
            const Calibration &c = Config::calibration[i];
            ESP_LOGI(TAG, "calibration[%d]: pos=%.2f ranges=[%.2f, %.2f] ratio=%.2f",
                     (int)i, c.pos, c.ranges[0], c.ranges[1], c.ratio);
        }
    }

    cJSON_Delete(json);

    httpd_resp_set_type(req, "application/json");
    if (error) {
        char resp[256];
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"%s\"}", error);
        httpd_resp_sendstr(req, resp);
    } else {
        httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"estructura valida\"}");
    }
    return ESP_OK;
}

static esp_err_t config_apply_post_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (Config::dh.empty()) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no hay config cargada, valida primero\"}");
        return ESP_OK;
    }

    bool ok = Cinematics::applyConfig();
    if (ok) {
        ESP_LOGI(TAG, "Config aplicada a Cinematics: %d DH, %d joints",
                 Cinematics::getDHCount(), Cinematics::getJointCount());
        httpd_resp_sendstr(req, "{\"ok\":true,\"message\":\"config aplicada\"}");
    } else {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no se pudo aplicar\"}");
    }
    return ESP_OK;
}

void init_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 16;

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
        reg("/api/config/validate", HTTP_POST, config_validate_post_handler);
        reg("/api/config/apply", HTTP_POST, config_apply_post_handler);
        reg("/api/joints", HTTP_GET, joints_get_handler);
        reg("/api/control", HTTP_POST, control_post_handler);
        reg("/api/points", HTTP_GET, points_get_handler);
        reg("/api/points", HTTP_POST, points_post_handler);
        reg("/api/points/load", HTTP_POST, points_load_handler);
        reg("/api/points/delete", HTTP_POST, points_delete_handler);
        reg("/api/fk", HTTP_GET, fk_get_handler);
        ESP_LOGI(TAG, "Servidor HTTP iniciado en puerto 80");
    } else {
        ESP_LOGE(TAG, "Error al iniciar servidor HTTP en puerto 80");
    }
}
