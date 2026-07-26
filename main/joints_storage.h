#ifndef JOINTS_STORAGE_H
#define JOINTS_STORAGE_H

#include <cstdint>
#include "web.h"
#include "matrix_math.hpp"

#define MAX_POINTS 20

struct joint_point_t {
    float angles[NUM_JOINTS];
    Matrix pose;
};

int joints_get_count(void);
const joint_point_t *joints_get(int index);
int joints_add(const float *angles);
int joints_delete(int index);
void joints_clear(void);

#endif
