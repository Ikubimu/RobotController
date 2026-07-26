#ifndef ARM_HPP
#define ARM_HPP

#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Joint.hpp"
#include "joints_storage.h"
#include "matrix_math.hpp"

#define ARM_DEFAULT_VEL_LIN 50.0f
#define ARM_DEFAULT_VEL_ANG 2.0f
#define ARM_DEFAULT_ACC 0.5f

class Arm {
public:
    Arm(uint8_t numJoints);

    void MoveL(uint8_t pointIndex, float vel = ARM_DEFAULT_VEL_LIN, float acc = ARM_DEFAULT_ACC);
    void MoveJ(uint8_t pointIndex, float vel = ARM_DEFAULT_VEL_ANG, float acc = ARM_DEFAULT_ACC);
    void updateJoint(uint8_t id, float vel, float pos);
    void RotateJoint(uint8_t id, float pos, float vel = ARM_DEFAULT_VEL_LIN);
    std::vector<float> getPos() const;

private:
    void controlTask();
    static void controlTaskEntry(void *arg);

    std::vector<Joint> joints;
    TaskHandle_t taskHandle = nullptr;

    Matrix targetPose;
    std::vector<float> targetAngles;
    float moveVel = 0.0f;
    float moveThreshold = ARM_DEFAULT_ACC;
    volatile bool moveActive = false;
};

#endif
