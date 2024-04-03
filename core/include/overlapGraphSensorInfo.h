#ifndef OVERLAPGRAPHSENSORINFO_H
#define OVERLAPGRAPHSENSORINFO_H

#include <vector>
#include "Sensor.h"

class overlapGraphSensorInfo {
public:
    Sensor* sensor;
    uint8_t index;
    uint8_t confidence;
    uint8_t numCompatible;
    std::vector<uint8_t> compatibleSensors;

    overlapGraphSensorInfo(Sensor* sensor_, uint8_t index_, std::vector<uint8_t> compatibleSensors_);
};

#endif // OVERLAPGRAPHSENSORINFO_H