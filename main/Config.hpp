#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <vector>
#include "joints_storage.h"

struct DH_values {
    float theta;
    float alpha;
    float d;
    float a;
};

struct Calibration {
    float pos;
    float ranges[2];
    float ratio;
};

namespace Config {

extern std::vector<DH_values> dh;
extern std::vector<int> rot;
extern std::vector<int> planar;
extern std::vector<Calibration> calibration;

}

#endif
