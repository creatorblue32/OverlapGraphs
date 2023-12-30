// main.cpp
#include <iostream>
#include <random>
#include <thread>
#include <unistd.h>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <map>
#include <mutex>



//HELPER FUNCTIONS
float generateRandomFloat(float minValue, float maxValue) {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> distribution(minValue, maxValue);
    return distribution(generator);
}

// CLASS DEFINITIONS
class Sensor {
public:
    virtual std::vector<float> getState() = 0;
    virtual uint8_t overlap(Sensor * other) = 0; 
};


class testSensor : public Sensor {
    public:
    std::vector<float> getState() override {
        return {distance};
    }
    float getValue() {
        return distance;
    }
    void setValue(float value){
        distance = value;
    }
    void operateSensor(float realValue) {
        distance = realValue + generateRandomFloat((-1*noise),noise);
    }
    uint8_t overlap(Sensor* other) override {
        if(dynamic_cast<testSensor*>(other)){
            testSensor * otherTest =  dynamic_cast<testSensor*>(other);
            return 85*abs(distance - otherTest->getValue());
        }
        else { throw std::invalid_argument("Incompatible Sensor in Compatibility Map.");}
    }
    float noise;
    private:
    float distance;
};

class SimulationEnvironment1 {
public:
    std::vector<testSensor*> vehicle;
    std::vector<std::vector<float>> sequence;
    bool simulationComplete;

    SimulationEnvironment1(std::vector<Sensor*> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput){
        vehicle = {};
        for (Sensor* sensor : vehicleInput){
            testSensor * current = dynamic_cast<testSensor*>(sensor);
            if(current){
                vehicle.push_back(current);
            }
        }
        // make a vehicle of sensors that need to be operated. 
        sequence = sequenceInput;
        simulationComplete = false;
        realValue = valueInput;
    }


    void operateSensors(){
        while(!simulationComplete){
            for(int i = 0; i < vehicle.size(); i++){
                vehicle[i]->operateSensor(realValue);
            }
        }
    }

    void runSequence(){
        for(int phase = 0; phase < sequence.size(); phase++){
            for(int sensor = 0; sensor < sequence[phase].size(); sensor++){
                vehicle[sensor]->noise = sequence[phase][sensor];
            }
            sleep(5);
            std::cout << "Phase " << phase << " complete. " << std::endl;
        }
        simulationComplete = true;
    }

    void runSimulation(){
        std::thread run(&SimulationEnvironment1::runSequence, this);
        std::thread sensorNoise(&SimulationEnvironment1::operateSensors, this);
        run.join();
        sensorNoise.join();
        std::cout << "Simulation complete. " << std::endl;
        exit(0);
    }

private:
    int realValue;
};


class sensorInfo{
    public:
        sensorInfo(Sensor* sensor_, uint8_t index_, std::vector<uint8_t> compatibleSensors_){
            sensor = sensor_;
            compatibleSensors = compatibleSensors_;
            index = index_;
            numCompatible = compatibleSensors_.size();
            confidence = 0;
        }
        Sensor * sensor;
        uint8_t index;
        float confidence;
        uint8_t numCompatible;
        std::vector<uint8_t> compatibleSensors;
};

using overlapFunction = float(*) (Sensor*, Sensor*);

class overlapGraphEstimator{
    public:
        overlapGraphEstimator(std::vector<sensorInfo*> sensorIndex_, std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph_){
            sensorIndex = sensorIndex_;
            overlapGraph = overlapGraph_;
        }
        uint8_t getAgreement(uint8_t currentSensor, uint8_t adjacentSensor){
            if(adjacentSensor > currentSensor){
                return overlapGraph[{adjacentSensor,currentSensor}];
            }
            else{
                return overlapGraph[{adjacentSensor,currentSensor}];
            }            
        }
        void makeEstimation(){

        }

        void updateConfidence(uint8_t currSensorIndex, uint8_t prevAgreement, uint8_t currAgreement){
            sensorIndex[currSensorIndex]->confidence += ((float)(currAgreement-prevAgreement)) / sensorIndex[currSensorIndex]->numCompatible;
        }
        
        float updateGraph(){
            std::vector<int> totals(sensorIndex.size(), 0);  
            for (const auto& pair : overlapGraph) {
                uint8_t agreement = getAgreement(pair.first.first, pair.first.second);
                totals[pair.first.first] += agreement;
                totals[pair.first.second] = agreement;
            }

            for(sensorInfo* currentSensor : sensorIndex){
                currentSensor->confidence = totals[currentSensor->index] / currentSensor->numCompatible;
            }
            std::cout << "Completed initialization. " << std::endl;

            while(1==1){
                for(sensorInfo* currentSensor : sensorIndex){
                    //Run through all compatible sensors
                    int totalAgreements = 0;
                    for(uint8_t adjacentSensor : currentSensor->compatibleSensors){
                        uint8_t prevAgreement = getAgreement(currentSensor->index, adjacentSensor);
                        uint8_t currAgreement = currentSensor->sensor->overlap(sensorIndex[adjacentSensor]->sensor);
                        updateConfidence(adjacentSensor, prevAgreement, currAgreement);
                        totalAgreements += currAgreement;
                    }
                    currentSensor->confidence = float(totalAgreements) / currentSensor->numCompatible;
                }
            }
        }
        std::vector<sensorInfo*> sensorIndex;
        std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph;

    private:

};


std::vector<float> traditionalStateEstimator(std::vector<Sensor*> vehicle, int states){
    while(1==1){
        std::vector<float> total = {};
        for(int i = 0; i < states; i++){
            total.push_back(0.0);
        }

        std::vector<float> current_state = {};
        int numSensors = vehicle.size();
        for(int sensor = 0; sensor < numSensors; sensor++){
            current_state = vehicle[sensor]->getState();
            for(int state = 0; state<states;state++){
                total[state] += current_state[state];
            }
        }
        
        for(int state = 0; state < states; state++){
            total[state] = total[state] / (float) numSensors;
            std::cout << "Traditional State Estimation: " << total[state] << std::endl;

        }
        sleep(1);
    }
}

int main() {
    //Make my test sensors, vehicle
    testSensor * firstSensor = new testSensor();
    testSensor * secondSensor = new testSensor();
    testSensor * thirdSensor = new testSensor();
    std::vector<Sensor*> vehicle = {firstSensor, secondSensor, thirdSensor};


    //Make my sequence
    std::vector<std::vector<float>> sequence = {{.001,.001,.25},{.001,.25,.001},{.25, .001,.001}};

    // Make simulation environment
    SimulationEnvironment1 sim = SimulationEnvironment1(vehicle, sequence, 5.0);

    //Make graph estimator
    sensorInfo* firstSensorInfo = new sensorInfo(firstSensor, 0, {1,2});

    std::vector<sensorInfo*> sensorMap = {firstSensorInfo, new sensorInfo(secondSensor,1,{0,2}), new sensorInfo(thirdSensor,2,{0,1})}; 
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0,1},0},{{1,2},0},{{0,2},0}};
    overlapGraphEstimator est = overlapGraphEstimator(sensorMap, overlapGraph);

    //Run Simulation, Traditional State Estimation, Overlap Graph Estimation
    std::thread run(&SimulationEnvironment1::runSimulation, sim); 
    sleep(1);
    std::thread traditional(traditionalStateEstimator, vehicle, 1);
    std::thread overlap(&overlapGraphEstimator::updateGraph, est);

    run.join();
    traditional.join();
    overlap.join();

    return 0;
}