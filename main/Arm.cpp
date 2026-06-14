#include "Arm.hpp"
#include "joints_storage.h"
#include "esp_log.h"

static const char *TAG = "Arm";

Arm::Arm(uint8_t numJoints)
{
    joints.reserve(numJoints);
    for (uint8_t i = 0; i < numJoints; i++) {
        joints.emplace_back(i);
    }
}

void Arm::MoveJ(uint8_t pointIndex, float vel, float acc)
{
    (void)acc;
    const joint_point_t *p = joints_get(pointIndex);
    if (!p) {
        ESP_LOGE(TAG, "MoveJ: punto %d no valido", pointIndex);
        return;
    }
    ESP_LOGI(TAG, "MoveJ punto %d, %zu joints", pointIndex, joints.size());
    for (uint8_t i = 0; i < joints.size(); i++) {
        RotateJoint(i, p->angles[i], vel);
    }
}

void Arm::MoveL(uint8_t pointIndex, float vel, float acc)
{
    (void)acc;
    const joint_point_t *p = joints_get(pointIndex);
    if (!p) {
        ESP_LOGE(TAG, "MoveL: punto %d no valido", pointIndex);
        return;
    }
    ESP_LOGI(TAG, "MoveL punto %d, %zu joints", pointIndex, joints.size());
    // TO DO: Implement linear interpolation for MoveL
}

void Arm::RotateJoint(uint8_t id, float pos, float vel)
{
    if (id >= joints.size()) return;
    float posRad = pos * (3.14159265f / 180.0f);
    joints[id].setPos(vel, posRad);
    ESP_LOGI(TAG, "RotateJoint id=%d pos=%.2f vel=%.2f", id, pos, vel);
}

std::vector<float> Arm::getPos() const
{
    std::vector<float> positions;
    positions.reserve(joints.size());
    for (const auto &j : joints) {
        positions.push_back(j.getPos() * (180.0f / 3.14159265f));
    }
    return positions;
}
