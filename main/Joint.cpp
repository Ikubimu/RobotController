#include "Joint.hpp"
#include "communication_handler.hpp"
#include <cstring>
#include "esp_log.h"


Joint::Joint(uint8_t id) : id(id), pos(0.0f), vel(0.0f), calibrated(false) {}

void Joint::calibrate(float pos, float ratio, float minRange, float maxRange)
{
    uint16_t canId = (static_cast<uint16_t>(id) << 8) | CMD_CALIBRATION;
    uint8_t data[8];
    uint16_t p = (uint16_t)(pos * 100.0f);
    int16_t r = (int16_t)(ratio * 100.0f);
    uint16_t mn = (uint16_t)(minRange * 100.0f);
    uint16_t mx = (uint16_t)(maxRange * 100.0f);
    std::memcpy(&data[0], &p, sizeof(uint16_t));
    std::memcpy(&data[2], &r, sizeof(int16_t));
    std::memcpy(&data[4], &mn, sizeof(uint16_t));
    std::memcpy(&data[6], &mx, sizeof(uint16_t));

    ESP_LOGI("Joint", "Joint %d calibrate: pos=%.2f ratio=%.2f range=[%.2f, %.2f]", id, pos, ratio, minRange, maxRange);

    CommunicationHandler::sendMessage(canId, data, 8);

    this->pos = pos;
}

void Joint::setCalibrated(bool calibrated)
{
    this->calibrated = calibrated;
}

bool Joint::isCalibrated() const
{
    return calibrated;
}

void Joint::setPos(float vel, float pos)
{
    uint16_t canId = (static_cast<uint16_t>(id) << 8) | CMD_MOVE_TARGET;
    uint8_t data[8];
    std::memcpy(&data[0], &pos, sizeof(float));
    std::memcpy(&data[4], &vel, sizeof(float));

    CommunicationHandler::sendMessage(canId, data, 8);
}

void Joint::update(float vel, float pos)
{
    this->vel = vel;
    this->pos = pos;
}

float Joint::getPos() const
{
    return pos;
}

float Joint::getVel() const
{
    return vel;
}
