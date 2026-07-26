#include "CinematicsUtils.hpp"
#include <cstring>
#include <cmath>

namespace Cinematics {

static DH_values s_dh_params[MAX_DH_PARAMS];
static uint8_t s_dh_count = 0;
static uint8_t s_joint_dh_indices[MAX_JOINTS];
static uint8_t s_joint_count = 0;

static Matrix computeDirectKinematics(const std::vector<DH_values> &dh) {
    Matrix T_total = MatMath::create(4, 4);
    MatMath::identity(T_total);

    for (uint8_t i = 0; i < dh.size(); i++) {
        float theta = dh[i].theta;
        float alpha = dh[i].alpha;
        float d     = dh[i].d;
        float a     = dh[i].a;

        float ct = cosf(theta);
        float st = sinf(theta);
        float ca = cosf(alpha);
        float sa = sinf(alpha);

        Matrix T_i = MatMath::create(4, 4);
        T_i.data[0]  = ct;  T_i.data[1]  = -st * ca; T_i.data[2]  =  st * sa; T_i.data[3]  = a * ct;
        T_i.data[4]  = st;  T_i.data[5]  =  ct * ca; T_i.data[6]  = -ct * sa; T_i.data[7]  = a * st;
        T_i.data[8]  = 0;   T_i.data[9]  =  sa;      T_i.data[10] =  ca;      T_i.data[11] = d;
        T_i.data[12] = 0;   T_i.data[13] = 0;        T_i.data[14] = 0;        T_i.data[15] = 1;

        T_total = MatMath::multiply(T_total, T_i);
    }

    return T_total;
}

static Matrix computeDirectKinematics(const std::vector<DH_values> &dh, std::vector<Matrix> &frames) {
    Matrix T_total = MatMath::create(4, 4);
    MatMath::identity(T_total);
    frames.clear();

    for (uint8_t i = 0; i < dh.size(); i++) {
        float theta = dh[i].theta;
        float alpha = dh[i].alpha;
        float d     = dh[i].d;
        float a     = dh[i].a;

        float ct = cosf(theta);
        float st = sinf(theta);
        float ca = cosf(alpha);
        float sa = sinf(alpha);

        Matrix T_i = MatMath::create(4, 4);
        T_i.data[0]  = ct;  T_i.data[1]  = -st * ca; T_i.data[2]  =  st * sa; T_i.data[3]  = a * ct;
        T_i.data[4]  = st;  T_i.data[5]  =  ct * ca; T_i.data[6]  = -ct * sa; T_i.data[7]  = a * st;
        T_i.data[8]  = 0;   T_i.data[9]  =  sa;      T_i.data[10] =  ca;      T_i.data[11] = d;
        T_i.data[12] = 0;   T_i.data[13] = 0;        T_i.data[14] = 0;        T_i.data[15] = 1;

        T_total = MatMath::multiply(T_total, T_i);
        frames.push_back(T_total);
    }

    return T_total;
}

void init() {
    s_dh_count = 0;
    s_joint_count = 0;
    memset(s_dh_params, 0, sizeof(s_dh_params));
    memset(s_joint_dh_indices, 0, sizeof(s_joint_dh_indices));
}

void addDH(const DH_values &dh) {
    if (s_dh_count < MAX_DH_PARAMS) {
        s_dh_params[s_dh_count++] = dh;
    }
}

void clearDH() {
    s_dh_count = 0;
    s_joint_count = 0;
}

uint8_t getDHCount() {
    return s_dh_count;
}

const DH_values& getDH(uint8_t index) {
    return s_dh_params[index];
}

void setJointIndices(const std::vector<uint8_t> &indices) {
    s_joint_count = indices.size();
    for (uint8_t i = 0; i < s_joint_count && i < MAX_JOINTS; i++) {
        s_joint_dh_indices[i] = indices[i];
    }
}

Matrix computeFromJoints(const std::vector<float> &joint_angles) {
    std::vector<DH_values> temp(s_dh_params, s_dh_params + s_dh_count);

    for (uint8_t i = 0; i < s_joint_count; i++) {
        uint8_t dh_idx = s_joint_dh_indices[i];
        if (dh_idx < temp.size() && i < joint_angles.size()) {
            temp[dh_idx].theta += joint_angles[i];
        }
    }

    return computeDirectKinematics(temp);
}

Matrix computeFromJoints(const std::vector<float> &joint_angles, std::vector<Matrix> &frames) {
    std::vector<DH_values> temp(s_dh_params, s_dh_params + s_dh_count);

    for (uint8_t i = 0; i < s_joint_count; i++) {
        uint8_t dh_idx = s_joint_dh_indices[i];
        if (dh_idx < temp.size() && i < joint_angles.size()) {
            temp[dh_idx].theta += joint_angles[i];
        }
    }

    return computeDirectKinematics(temp, frames);
}

std::vector<float> computeJointVelocities(const Matrix &J, float v_lin, const float err_pos[3], const float err_rot[3]) {
    uint8_t n = J.cols;

    float pos_norm = sqrtf(err_pos[0] * err_pos[0] + err_pos[1] * err_pos[1] + err_pos[2] * err_pos[2]);
    float k_lin = (pos_norm > 1e-6f) ? v_lin / pos_norm : 0.0f;

    float vx = err_pos[0] * k_lin;
    float vy = err_pos[1] * k_lin;
    float vz = err_pos[2] * k_lin;

    float rot_norm = sqrtf(err_rot[0] * err_rot[0] + err_rot[1] * err_rot[1] + err_rot[2] * err_rot[2]);
    float k_rot = (rot_norm > 1e-6f) ? v_lin / rot_norm : 0.0f;

    float wx = err_rot[0] * k_rot;
    float wy = err_rot[1] * k_rot;
    float wz = err_rot[2] * k_rot;

    float vel_data[6] = {vx, vy, vz, wx, wy, wz};
    Matrix tcp_vel = MatMath::create(6, 1);
    memcpy(tcp_vel.data, vel_data, sizeof(float) * 6);

    Matrix J_pinv = MatMath::pinv(J);
    Matrix q_dot = MatMath::multiply(J_pinv, tcp_vel);

    std::vector<float> result(n);
    for (uint8_t i = 0; i < n; i++) {
        result[i] = q_dot.data[i];
    }

    return result;
}

Matrix computeJacobian(const std::vector<Matrix> &frames) {
    Matrix J = MatMath::create(6, s_joint_count);

    const Matrix &last = frames.back();
    float o_n[3] = {last.data[3], last.data[7], last.data[11]};

    for (uint8_t i = 0; i < s_joint_count; i++) {
        uint8_t dh_idx = s_joint_dh_indices[i];

        float z[3], o[3];

        if (dh_idx == 0) {
            z[0] = 0; z[1] = 0; z[2] = 1;
            o[0] = 0; o[1] = 0; o[2] = 0;
        } else {
            const Matrix &prev = frames[dh_idx - 1];
            z[0] = prev.data[8]; z[1] = prev.data[9]; z[2] = prev.data[10];
            o[0] = prev.data[3]; o[1] = prev.data[7]; o[2] = prev.data[11];
        }

        float diff[3] = {o_n[0] - o[0], o_n[1] - o[1], o_n[2] - o[2]};

        float cr[3] = {
            z[1] * diff[2] - z[2] * diff[1],
            z[2] * diff[0] - z[0] * diff[2],
            z[0] * diff[1] - z[1] * diff[0]
        };

        J.data[0 * s_joint_count + i] = cr[0];
        J.data[1 * s_joint_count + i] = cr[1];
        J.data[2 * s_joint_count + i] = cr[2];
        J.data[3 * s_joint_count + i] = z[0];
        J.data[4 * s_joint_count + i] = z[1];
        J.data[5 * s_joint_count + i] = z[2];
    }

    return J;
}

}
