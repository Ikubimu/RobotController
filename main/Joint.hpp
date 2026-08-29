#ifndef JOINT_HPP
#define JOINT_HPP

#include <stdint.h>

class Joint {
public:
    Joint(uint8_t id);

    void calibrate(float pos, float ratio, float minRange, float maxRange);
    void setCalibrated(bool calibrated);
    bool isCalibrated() const;
    void setPos(float vel, float pos);
    void update(float vel, float pos);
    float getPos() const;
    float getVel() const;

private:
    uint8_t id;
    float pos;
    float vel;
    bool calibrated;
};

#endif
