#ifndef TRADITIONALSTATEESTIMATOR_H
#define TRADITIONALSTATEESTIMATOR_H

#include <vector>
#include "Sensor.h"


class TraditionalStateEstimator {
public:
    static std::vector<float> makeEstimation(const std::vector<Sensor*>& sensors);
};

#endif // TRADITIONALSTATEESTIMATOR_H
