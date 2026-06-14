#include "Arm.hpp"

Arm::Arm(uint8_t numJoints)
{
    joints.reserve(numJoints);
    for (uint8_t i = 0; i < numJoints; i++) {
        joints.emplace_back(i);
    }
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
