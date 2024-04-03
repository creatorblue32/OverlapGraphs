#include "TraditionalStateEstimator.h"

std::vector<float> TraditionalStateEstimator::makeEstimation(const std::vector<Sensor*>& sensors) {
    if (sensors.empty()) return {};

    size_t numStates = sensors.front()->getState().size();
    std::vector<float> averageState(numStates, 0.0);
    size_t numSensors = sensors.size();

    for (const auto& sensor : sensors) {
        auto state = sensor->getState();
        for (size_t i = 0; i < numStates; ++i) {
            averageState[i] += state[i];
        }
    }

    for (float& val : averageState) {
        val /= numSensors;
    }

    return averageState;
}
