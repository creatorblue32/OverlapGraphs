#ifndef SENSOR_H
#define SENSOR_H

#include <vector>

class Sensor {
public:
    virtual std::vector<float> getState() = 0;
    virtual uint8_t overlap(Sensor *other) = 0;
    std::vector<float> minState;
    std::vector<float> maxState;
    uint8_t dimension;
};

#endif // SENSOR_H
