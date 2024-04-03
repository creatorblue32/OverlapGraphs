#ifndef SIMULATEDDEPTHMAPSENSOR_H
#define SIMULATEDDEPTHMAPSENSOR_H

#include "Sensor.h"
#include <vector>
#include <mutex>
#include <utility>

class simulatedDepthMapSensor : public Sensor {
public:
    simulatedDepthMapSensor(std::pair<int, int> offset_);
    std::vector<float> getState() override;
    uint8_t overlap(Sensor* other) override;
    void setValue(float value_[5][5]);
    void operateSensor();

    uint8_t dimension;
    std::vector<float> minState;
    std::vector<float> maxState;
    float noise;
    std::pair<int, int> offset;

private:
    float noiselessValue[5][5];
    float value[5][5];
    std::mutex sensorMutex;
};

#endif // SIMULATEDDEPTHMAPSENSOR_H
