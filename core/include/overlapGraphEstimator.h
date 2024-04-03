#ifndef OVERLAPGRAPH_ESTIMATOR_H
#define OVERLAPGRAPH_ESTIMATOR_H

#include <vector>
#include <map>
#include <mutex>
#include "Sensor.h"
#include "overlapGraphSensorInfo.h"
#include <set>

class overlapGraphEstimator {
public:
    overlapGraphEstimator(std::vector<overlapGraphSensorInfo*> sensorIndex_, std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph_, uint8_t dimension_);
    std::vector<float> makeEstimation();
    void discriminate();
    std::vector<overlapGraphSensorInfo*> sensorIndex;
    void updateGraph();    
    std::set<uint8_t> reliableSensors;


private:
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph;
    uint8_t dimension;
    std::mutex graphMutex;

    uint8_t getAgreement(uint8_t currentSensor, uint8_t adjacentSensor);
    void putAgreement(uint8_t currentSensor, uint8_t adjacentSensor, uint8_t agreement);
};

#endif // OVERLAPGRAPH_ESTIMATOR_H
