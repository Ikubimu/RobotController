#include <string.h>
#include "joints_storage.h"

static joint_point_t s_points[MAX_POINTS];
static int s_count = 0;

int joints_get_count(void)
{
    return s_count;
}

const joint_point_t *joints_get(int index)
{
    if (index < 0 || index >= s_count) return NULL;
    return &s_points[index];
}

int joints_add(const float *angles)
{
    if (s_count >= MAX_POINTS) return -1;
    memcpy(s_points[s_count].angles, angles, sizeof(float) * NUM_JOINTS);
    return s_count++;
}

int joints_delete(int index)
{
    if (index < 0 || index >= s_count) return -1;
    int to_move = s_count - index - 1;
    if (to_move > 0) {
        memmove(&s_points[index], &s_points[index + 1], sizeof(joint_point_t) * to_move);
    }
    s_count--;
    return 0;
}

void joints_clear(void)
{
    s_count = 0;
}
