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
        float makeEstimation(){
            // now we have an adjacency list of confidences. how to get an estimation out of it?

            //Take average of confidences (efficiency: recalculate averages after each update.)
            float confidenceAverage = 0;
            for (sensorInfo* currentSensor: sensorIndex){
                confidenceAverage += currentSensor->confidence;
            }
            confidenceAverage = confidenceAverage / sensorIndex.size();
            

            float currentEstimation = 0;
            uint8_t currentActive = 0;

            //std::cout << "Confidence Average " << confidenceAverage <<std::endl;


            for (sensorInfo* currentSensor: sensorIndex){
                //std::cout << "Current Confidence " << currentSensor->confidence <<std::endl;

                if(currentSensor->confidence <= confidenceAverage){
                    currentEstimation += currentSensor->sensor->getState()[0];
                    currentActive++;
                }
            }
            //std::cout << "Num active: " << float(currentActive) << std::endl;
            float estimationVal = currentEstimation / (float) currentActive;
            //std::string estimation = "Overlap Graph Estimation " + std::to_string(estimationVal);
            //std::cout << estimation << std::endl;
            return estimationVal;
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


std::vector<float> traditionalStateEstimator(std::vector<Sensor*> vehicle, uint8_t states){
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
    }
    return total;
}


class SimulationEnvironment1 {
public:
    std::vector<testSensor*> vehicle;
    std::vector<Sensor*> sensorVehicle;
    overlapGraphEstimator* graphEstimator;

    std::vector<std::vector<float>> sequence;
    bool simulationComplete;

    SimulationEnvironment1(std::vector<Sensor*> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator* graphEstimator_) {
        vehicle = {};
        for (Sensor* sensor : vehicleInput){
            testSensor * current = dynamic_cast<testSensor*>(sensor);
            if(current){
                vehicle.push_back(current);
            }
        }
        sensorVehicle = vehicleInput;
        // make a vehicle of sensors that need to be operated. 
        sequence = sequenceInput;
        simulationComplete = false;
        realValue = valueInput;
        graphEstimator = graphEstimator_;
    }


    void operateSensors(){
        while(!simulationComplete){
            for(int i = 0; i < vehicle.size(); i++){
                vehicle[i]->operateSensor(realValue);
            }
        }
    }

    void runSequence(){
        std::vector<float> overlapGraphEstimations = {};
        std::vector<float> traditionalEstimations = {};
        std::vector<long> overlapDurations = {};
        std::vector<long> traditionalDurations = {};


        sleep(1);
        std::cout << "Beginning Simulation." << std::endl;

        for(int phase = 0; phase < sequence.size(); phase++){
            for(int sensor = 0; sensor < sequence[phase].size(); sensor++){
                vehicle[sensor]->noise = sequence[phase][sensor];
            }
            for(int i = 0; i < 5; i++){
                //Request States every 1 second
                auto start = std::chrono::high_resolution_clock::now();
                traditionalEstimations.push_back(traditionalStateEstimator(sensorVehicle, 1)[0]);
                auto stop = std::chrono::high_resolution_clock::now();
                traditionalDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start)).count());

                auto start2 = std::chrono::high_resolution_clock::now();
                overlapGraphEstimations.push_back(graphEstimator->makeEstimation());
                auto stop2 = std::chrono::high_resolution_clock::now();
                overlapDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop2 - start2)).count());


                sleep(1);
            }
            std::cout << "Phase " << phase << " complete. " << std::endl;
        }
        
        std::vector<float> tradDifferences = {};

        //std::cout << "Traditional Estimations: ";
        for (float element : traditionalEstimations) {
            tradDifferences.push_back(abs(element-5));
            //std::cout << element << " ";
        }
        //std::cout << std::endl;

        std::vector<float> overlapDifferences = {};

        //std::cout << "Overlap Graph Estimations: ";
        for (float element : overlapGraphEstimations) {
            overlapDifferences.push_back(abs(element-5));
            //std::cout << element << " ";
        }
        //std::cout << std::endl;   


        float overlapError =  std::accumulate(overlapDifferences.begin(), overlapDifferences.end(), 0.0f) / overlapDifferences.size();
        float tradError =  std::accumulate(tradDifferences.begin(), tradDifferences.end(), 0.0f) / tradDifferences.size();

        std::cout << "Average Traditional Error: " << tradError << std::endl;
        std::cout << "Average Overlap Error: " << overlapError << std::endl;

        float tradTime =  std::accumulate(traditionalDurations.begin(), traditionalDurations.end(), 0.0f) / traditionalDurations.size();
        float overlapTime =  std::accumulate(overlapDurations.begin(), overlapDurations.end(), 0.0f) / overlapDurations.size();
        
        std::cout << "Average Traditional Time: " << tradTime << std::endl;
        std::cout << "Average Overlap Time: " << overlapTime << std::endl;




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


int main() {
    //Make my test sensors, vehicle
    testSensor * firstSensor = new testSensor();
    testSensor * secondSensor = new testSensor();
    testSensor * thirdSensor = new testSensor();
    std::vector<Sensor*> vehicle = {firstSensor, secondSensor, thirdSensor};


    //Make my sequence
    std::vector<std::vector<float>> sequence = {{0.0,0.0,1.0},{0.0,1.0,0.0},{1.0, 0.0,0.0}};


    //Make graph estimator
    sensorInfo* firstSensorInfo = new sensorInfo(firstSensor, 0, {1,2});

    std::vector<sensorInfo*> sensorMap = {firstSensorInfo, new sensorInfo(secondSensor,1,{0,2}), new sensorInfo(thirdSensor,2,{0,1})}; 
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0,1},0},{{1,2},0},{{0,2},0}};
    overlapGraphEstimator est = overlapGraphEstimator(sensorMap, overlapGraph);

    // Make simulation environment
    SimulationEnvironment1 sim = SimulationEnvironment1(vehicle, sequence, 5.0, &est);


    //Run Simulation, Traditional State Estimation, Overlap Graph Estimation
    std::thread run(&SimulationEnvironment1::runSimulation, sim); 
    sleep(1);
    std::thread traditional(traditionalStateEstimator, vehicle, 1);
    std::thread overlaprun(&overlapGraphEstimator::updateGraph, est);
    std::thread overlap(&overlapGraphEstimator::makeEstimation, est);


    run.join();
    traditional.join();
    overlaprun.join();
    overlap.join();

    return 0;
}