#include <iostream>
#include <thread>
#include "SimulationEnvironment2.h"
#include "overlapGraphEstimator.h"
#include "simulatedDepthMapSensor.h"

int runTest2();

int main() {
    try {
        runTest2();
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred in main: " << e.what() << std::endl;
    }
    return 0;
}

int runTest2() {
    simulatedDepthMapSensor* firstSensor = new simulatedDepthMapSensor(std::pair<int, int>(-1, -1));
    simulatedDepthMapSensor* secondSensor = new simulatedDepthMapSensor(std::pair<int, int>(1, -1));
    simulatedDepthMapSensor* thirdSensor = new simulatedDepthMapSensor(std::pair<int, int>(1, 1));
    simulatedDepthMapSensor* fourthSensor = new simulatedDepthMapSensor(std::pair<int, int>(-1, 1));

    std::vector<Sensor*> vehicle = {firstSensor, secondSensor, thirdSensor, fourthSensor};

    std::vector<std::vector<float>> sequence = {{10.0, 0.1, 0.1, 0.1}, {0.1, 10.0, 0.1, 0.1}, {0.1, 0.1, 10.0, 0.1}};

    std::vector<overlapGraphSensorInfo*> sensorMap = {
        new overlapGraphSensorInfo(firstSensor, 0, {1, 2, 3}),
        new overlapGraphSensorInfo(secondSensor, 1, {0, 2, 3}),
        new overlapGraphSensorInfo(thirdSensor, 2, {0, 1, 3}),
        new overlapGraphSensorInfo(fourthSensor, 3, {0, 1, 2})
    };

    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0, 1}, 0}, {{1, 2}, 0}, {{0, 2}, 0}, {{0, 3}, 0}, {{1, 3}, 0}, {{2, 3}, 0}};
    overlapGraphEstimator est(sensorMap, overlapGraph, 2);

    SimulationEnvironment2 sim(vehicle, sequence, 5, &est);

    // Run Simulation, Traditional State Estimation, Overlap Graph Estimation
    std::thread runSimulation(&SimulationEnvironment2::runSimulation, sim);
    std::thread runOverlapGraph(&overlapGraphEstimator::updateGraph, &est);

    runSimulation.join();
    runOverlapGraph.join();

    return 0;
}
