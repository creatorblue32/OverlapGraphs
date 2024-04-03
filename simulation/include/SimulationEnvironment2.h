#ifndef SIMULATIONENVIRONMENT2_H
#define SIMULATIONENVIRONMENT2_H

#include <vector>
#include <mutex>
#include <map>
#include "../../core/include/Sensor.h"
#include "../../core/include/overlapGraphEstimator.h"
#include "TraditionalStateEstimator.h"
#include "simulatedDepthMapSensor.h"


class SimulationEnvironment2 {
public:
    SimulationEnvironment2(std::vector<Sensor*> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator* graphEstimator_);
    void runSimulation();
    std::vector<simulatedDepthMapSensor *> depthMapSensorVehicle;
    std::vector<Sensor *> sensorVehicle;
    overlapGraphEstimator *graphEstimator;
    TraditionalStateEstimator *tradEstimator;
    std::vector<std::vector<float>> sequence;
    bool simulationComplete;
    void saveToCSV(const std::vector<std::vector<float>>& data, const std::vector<std::string>& columnNames, const std::string& filename);



private:
    void runSequence();
    int realValue;


};

#endif
