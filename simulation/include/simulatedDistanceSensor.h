#ifndef SIMULATEDDISTANCESENSOR_H
#define SIMULATEDDISTANCESENSOR_H

#include "Sensor.h"
#include <vector>
#include <random>

class simulatedDistanceSensor : public Sensor {
public:
    simulatedDistanceSensor(float minValue, float maxValue, float noiseLevel);
    std::vector<float> getState() override;
    uint8_t overlap(Sensor* other) override;
    void setValue(float value_);
    void operateSensor(float realValue);

private:
    float generateRandomFloat(float minValue, float maxValue);
    float noise;
    float value;
    std::random_device rd;
    std::mt19937 generator;
    std::uniform_real_distribution<float> distribution;
};

#endif // SIMULATEDDISTANCESENSOR_H
