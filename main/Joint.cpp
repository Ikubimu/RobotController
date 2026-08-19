#include "Joint.hpp"
#include "communication_handler.hpp"
#include <cstring>


Joint::Joint(uint8_t id) : id(id), pos(0.0f), vel(0.0f), calibrated(false) {}

void Joint::calibrate(float ratio, float pos)
{
    uint16_t canId = (static_cast<uint16_t>(id) << 8) | CMD_CALIBRATION;
    uint8_t data[8];
    std::memcpy(&data[0], &ratio, sizeof(float));
    std::memcpy(&data[4], &pos, sizeof(float));

    CommunicationHandler::sendMessage(canId, data, 8);

    this->pos = pos;
    this->calibrated = true;
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
