#include "Arm.hpp"
#include "CinematicsUtils.hpp"
#include "joints_storage.h"
#include "sm_events.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstring>

static const char *TAG = "Arm";

void Arm::controlTaskEntry(void *arg) {
    static_cast<Arm*>(arg)->controlTask();
}

void Arm::controlTask() {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (moveActive) {
            std::vector<float> currentAngles = getPos();

            std::vector<Matrix> frames;
            Matrix T_current = Cinematics::computeFromJoints(currentAngles, frames);

            float err_pos[3] = {
                targetPose.data[3] - T_current.data[3],
                targetPose.data[7] - T_current.data[7],
                targetPose.data[11] - T_current.data[11]
            };

            float err_rot[3];
            Cinematics::orientationError(T_current, targetPose, err_rot);

            float pos_norm = sqrtf(err_pos[0] * err_pos[0] + err_pos[1] * err_pos[1] + err_pos[2] * err_pos[2]);
            float rot_norm = sqrtf(err_rot[0] * err_rot[0] + err_rot[1] * err_rot[1] + err_rot[2] * err_rot[2]);
            if (pos_norm < moveThreshold && rot_norm < 0.02f) {
                ESP_LOGI(TAG, "MoveL: objetivo alcanzado");
                moveActive = false;
                break;
            }

            Matrix J = Cinematics::computeJacobian(frames);
            std::vector<float> q_dot = Cinematics::computeJointVelocities(J, moveVel, err_pos, err_rot);

            for (uint8_t i = 0; i < joints.size() && i < q_dot.size(); i++) {
                joints[i].setPos(q_dot[i], targetAngles[i]);
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

Arm::Arm(uint8_t numJoints)
{
    joints.reserve(numJoints);
    for (uint8_t i = 1; i <= numJoints; i++) {
        joints.emplace_back(i);
    }
    targetAngles.resize(numJoints, 0.0f);
    xTaskCreate(controlTaskEntry, "ArmCtrl", 4096, this, 1, &taskHandle);
}

void Arm::MoveJ(uint8_t pointIndex, float vel, float acc)
{
    (void)acc;
    const joint_point_t *p = joints_get(pointIndex);
    if (!p) {
        ESP_LOGE(TAG, "MoveJ: punto %d no valido", pointIndex);
        return;
    }

    std::vector<float> target(joints.size());
    for (size_t i = 0; i < joints.size(); i++)
        target[i] = p->angles[i];

    moveJToAngles(target, vel);
}

void Arm::MoveJTo(const std::vector<float> &targetAngles, float vel)
{
    moveJToAngles(targetAngles, vel);
}

void Arm::moveJToAngles(const std::vector<float> &targetAngles, float vel)
{
    std::vector<float> currentAngles = getPos();

    float maxTime = 0.0f;
    for (size_t i = 0; i < joints.size() && i < targetAngles.size(); i++) {
        float dist = fabsf(targetAngles[i] - currentAngles[i]);
        float t = (vel > 1e-6f) ? dist / vel : 0.0f;
        if (t > maxTime) maxTime = t;
    }

    ESP_LOGI(TAG, "MoveJ, maxTime: %.3f", maxTime);

    for (size_t i = 0; i < joints.size() && i < targetAngles.size(); i++) {
        float diff = targetAngles[i] - currentAngles[i];
        float jointVel = (maxTime > 1e-6f) ? diff / maxTime : 0.0f;
        joints[i].setPos(jointVel, targetAngles[i]);
    }

    jointMoveDeadline = xTaskGetTickCount() +
        pdMS_TO_TICKS((uint32_t)(maxTime * 1000.0f) + 1000);
}

void Arm::MoveL(uint8_t pointIndex, float vel, float acc)
{
    const joint_point_t *p = joints_get(pointIndex);
    if (!p) {
        ESP_LOGE(TAG, "MoveL: punto %d no valido", pointIndex);
        return;
    }
    ESP_LOGI(TAG, "MoveL punto %d, %zu joints", pointIndex, joints.size());

    targetPose = p->pose;

    for (uint8_t i = 0; i < joints.size(); i++) {
        targetAngles[i] = p->angles[i];
    }
    moveVel = vel;
    moveThreshold = acc;
    moveActive = true;
    jointMoveDeadline = 0;
    xTaskNotifyGive(taskHandle);
}

void Arm::updateJoint(uint8_t id, float vel, float pos)
{
    if (id > joints.size() || id < 1) return;
    joints[id - 1].update(vel, pos);
}

void Arm::setJointCalibrated(uint8_t id, bool calibrated)
{
    if (id > joints.size() || id < 1) return;
    joints[id - 1].setCalibrated(calibrated);
}

void Arm::calibrate(const std::vector<Calibration> &calibration)
{
    for (size_t i = 0; i < calibration.size() && i < joints.size(); i++) {
        const Calibration &c = calibration[i];
        joints[i].calibrate(c.pos, c.ratio, c.ranges[0], c.ranges[1]);
    }
    sm_setReady(true);
}

void Arm::RotateJoint(uint8_t id, float pos, float vel)
{
    if (id >= joints.size()) return;
    joints[id].setPos(vel, pos);
    jointMoveDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(JOINT_MOVE_TIMEOUT_MS);
}

bool Arm::isMoving() const
{
    if (moveActive) return true;
    if (jointMoveDeadline != 0 && xTaskGetTickCount() < jointMoveDeadline)
        return true;
    return false;
}

std::vector<float> Arm::getPos() const
{
    std::vector<float> positions;
    positions.reserve(joints.size());
    for (const auto &j : joints) {
        positions.push_back(j.getPos());
    }
    return positions;
}


