#ifndef JOINTS_STORAGE_H
#define JOINTS_STORAGE_H

#include <stdint.h>
#include "web.h"

#define MAX_POINTS 20

typedef struct {
    float angles[NUM_JOINTS];
} joint_point_t;

#ifdef __cplusplus
extern "C" {
#endif

int joints_get_count(void);
const joint_point_t *joints_get(int index);
int joints_add(const float *angles);
int joints_delete(int index);
void joints_clear(void);

#ifdef __cplusplus
}
#endif

#endif
