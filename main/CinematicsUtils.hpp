#ifndef CINEMATICS_UTILS_HPP
#define CINEMATICS_UTILS_HPP

#include <cstdint>
#include <vector>
#include "joints_storage.h"
#include "matrix_math.hpp"

#define MAX_DH_PARAMS 10
#define MAX_JOINTS 8
#define MAX_FRAMES 10

struct DH_values {
    float theta;
    float alpha;
    float d;
    float a;
};

namespace Cinematics {

void init();
void addDH(const DH_values &dh);
void clearDH();
uint8_t getDHCount();
const DH_values& getDH(uint8_t index);
void setJointIndices(const std::vector<uint8_t> &indices);
uint8_t getJointCount();
Matrix computeFromJoints(const std::vector<float> &joint_angles);
Matrix computeFromJoints(const std::vector<float> &joint_angles, std::vector<Matrix> &frames);
Matrix computeJacobian(const std::vector<Matrix> &frames);
std::vector<float> computeJointVelocities(const Matrix &J, float v_lin, const float err_pos[3], const float err_rot[3]);
void orientationError(const Matrix &T_current, const Matrix &T_target, float err_rot[3]);

}

#endif
