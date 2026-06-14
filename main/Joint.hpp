#ifndef JOINT_HPP
#define JOINT_HPP

#include <stdint.h>

class Joint {
public:
    Joint(uint8_t id);

    void calibrate(float ratio, float pos);
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
