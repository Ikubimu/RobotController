#ifndef ARM_HPP
#define ARM_HPP

#include <vector>
#include "Joint.hpp"

#define ARM_DEFAULT_VEL 1.0f
#define ARM_DEFAULT_ACC 0.0f

class Arm {
public:
    Arm(uint8_t numJoints);

    void MoveL(uint8_t pointIndex, float vel = ARM_DEFAULT_VEL, float acc = ARM_DEFAULT_ACC);
    void MoveJ(uint8_t pointIndex, float vel = ARM_DEFAULT_VEL, float acc = ARM_DEFAULT_ACC);
    void updateJoint(uint8_t id, float vel, float pos);
    void RotateJoint(uint8_t id, float pos, float vel = ARM_DEFAULT_VEL);
    std::vector<float> getPos() const;

private:
    std::vector<Joint> joints;
};

#endif
