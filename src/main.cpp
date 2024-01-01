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
    std::vector<float> minState;
    std::vector<float> maxState;
    uint8_t dimension;
};

class testSensor : public Sensor {
    public:
    std::vector<float> getState() override {
        return {value};
    }
    float getValue() {
        return value;
    }
    void setValue(float value_){
        value = value_;
    }
    void operateSensor(float realValue) {
        value = realValue + generateRandomFloat((-1*noise),noise);
    }
    uint8_t overlap(Sensor* other) override {
        if(dynamic_cast<testSensor*>(other)){
            testSensor * otherTestSensor =  dynamic_cast<testSensor*>(other);
            return 255-(abs(value - otherTestSensor->getValue())*(255/(maxState[0]-minState[0])));
        }
        else { throw std::invalid_argument("Incompatible Sensor in Compatibility Map.");}
    }
    uint8_t dimension = 1;
    std::vector<float> minState = {0.0};
    std::vector<float> maxState = {10.0};
    float noise;
    private:
    float value;
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
        uint8_t confidence;
        uint8_t numCompatible;
        std::vector<uint8_t> compatibleSensors;
};

class overlapGraphEstimator{
    public:   
        overlapGraphEstimator(std::vector<sensorInfo*> sensorIndex_, std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph_, bool verbose_){
            sensorIndex = sensorIndex_;
            overlapGraph = overlapGraph_;
            verbose = verbose_;
        } 
        std::vector<sensorInfo*> sensorIndex;
        std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph;
        bool verbose;
        std::mutex graphMutex;
        float makeEstimation(){
            std::lock_guard<std::mutex> guard(graphMutex);
            std::cout << "Making Estimation ------------- " << std::endl;
            //Take average of confidences (efficiency: recalculate averages after each update.)
            float confidenceAverage = 0;
            for (sensorInfo* currentSensor: sensorIndex){
                confidenceAverage += currentSensor->confidence;
                std::cout << "Confidence in Sensor " << (int) currentSensor->index << ": " << (int) currentSensor ->confidence << std::endl;
            }
            confidenceAverage = confidenceAverage / sensorIndex.size();
            

            float currentEstimation = 0;
            uint8_t currentActive = 0;

            //std::cout << "Confidence Average " << confidenceAverage <<std::endl;


            for (sensorInfo* currentSensor: sensorIndex){
                std::cout << "Sensor " << (int) currentSensor->index << " Confidence " << (int) currentSensor->confidence <<std::endl;

                if(currentSensor->confidence >= confidenceAverage){
                    currentEstimation += currentSensor->sensor->getState()[0];
                    currentActive++;
                }
                else{
                    std::cout << "So... we're excluding it - sensor: " << (int) currentSensor->index << std::endl;
                }
            }
            //std::cout << "Num active: " << float(currentActive) << std::endl;
            float estimationVal = currentEstimation / (float) currentActive;
            //std::string estimation = "Overlap Graph Estimation " + std::to_string(estimationVal);
            //std::cout << estimation << std::endl;
            return estimationVal;
        }
        
        float updateGraph(){
            graphMutex.lock();
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
            graphMutex.unlock();
            while(1==1){
                std::lock_guard<std::mutex> guard(graphMutex);
                for(sensorInfo* currentSensor : sensorIndex){
                    //Run through all compatible sensors
                    uint16_t totalAgreements = 0;
                    for(uint8_t adjacentSensor : currentSensor->compatibleSensors){
                        uint8_t prevAgreement = getAgreement(currentSensor->index, adjacentSensor);
                        uint8_t currAgreement = currentSensor->sensor->overlap(sensorIndex[adjacentSensor]->sensor);
                        //if(verbose) {std::cout << "Agreement between sensor " << (int) currentSensor->index << " and " << (int) sensorIndex[adjacentSensor]->index << " is " << (int) currAgreement << std::endl;}
                        //std::cout << "The confidence in sensor " << (int) currentSensor->index << " is " << (int) currentSensor->confidence << std::endl;
                        //sensorIndex[adjacentSensor]->confidence += ((float)(currAgreement-prevAgreement)) / sensorIndex[adjacentSensor]->numCompatible;
                        //std::cout << "Adjusted sensor " << (int) adjacentSensor << " to confidence " << (int) sensorIndex[adjacentSensor]->confidence << std::endl;
                        totalAgreements += currAgreement;
                    }
                    currentSensor->confidence = (uint8_t) (totalAgreements / currentSensor->numCompatible);
                    //std::cout << "AND finally, changed sensor " << (int) currentSensor->index << " to confidence " << (int) currentSensor->confidence << std::endl;
                }
            }
        }
        private:
        uint8_t getAgreement(uint8_t currentSensor, uint8_t adjacentSensor){
            if(adjacentSensor > currentSensor){
                return overlapGraph[{adjacentSensor,currentSensor}];
            }
            else{
                return overlapGraph[{adjacentSensor,currentSensor}];
            }            
        }
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




class SimulationEnvironment2 {
public:
    std::vector<testSensor*> vehicle;
    std::vector<Sensor*> sensorVehicle;
    overlapGraphEstimator* graphEstimator;

    std::vector<std::vector<float>> sequence;
    bool simulationComplete;

    SimulationEnvironment2(std::vector<Sensor*> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator* graphEstimator_) {
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
        std::thread run(&SimulationEnvironment2::runSequence, this);
        std::thread sensorNoise(&SimulationEnvironment2::operateSensors, this);
        run.join();
        sensorNoise.join();
        std::cout << "Simulation complete. " << std::endl;
        exit(0);
    }

private:
    int realValue;
};

int main() {


/*
    //Test 1:
    testSensor * sensor1 = new testSensor();
    testSensor * sensor2 = new testSensor();
    testSensor * sensor3 = new testSensor();

    sensor1->setValue(5.0);
    sensor2->setValue(5.0);
    sensor3->setValue(4.0);


    std::vector<sensorInfo*> sensorMap1 = {new sensorInfo(sensor1, 0, {1,2}), new sensorInfo(sensor2,1,{0,2}), new sensorInfo(sensor3,2,{0,1})}; 

    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph1 = {{{0,1},0},{{1,2},0},{{0,2},0}};
    overlapGraphEstimator estimator = overlapGraphEstimator(sensorMap1, overlapGraph1, true);

    std::thread overlapEstimation(&overlapGraphEstimator::updateGraph, estimator);
    sleep(2);

    std::string a = "Final Estimation: " +std::to_string(estimator.makeEstimation());
    std::cout << a << std::endl;

    exit(0);*/


    //Test 2:

    //Make my test sensors, vehicle
    testSensor * firstSensor = new testSensor();
    testSensor * secondSensor = new testSensor();
    testSensor * thirdSensor = new testSensor();
    std::vector<Sensor*> vehicle = {firstSensor, secondSensor, thirdSensor};


    //Make my sequence
    std::vector<std::vector<float>> sequence = {{0.0,0.0,4.0},{0.0,4.0,0.0},{4.0, 0.0,0.0}};


    //Make graph estimator
    sensorInfo* firstSensorInfo = new sensorInfo(firstSensor, 0, {1,2});

    std::vector<sensorInfo*> sensorMap = {firstSensorInfo, new sensorInfo(secondSensor,1,{0,2}), new sensorInfo(thirdSensor,2,{0,1})}; 
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0,1},0},{{1,2},0},{{0,2},0}};
    overlapGraphEstimator est(sensorMap, overlapGraph, false);

    // Make simulation environment
    SimulationEnvironment2 sim = SimulationEnvironment2(vehicle, sequence, 5.0, &est);


    //Run Simulation, Traditional State Estimation, Overlap Graph Estimation
    std::thread run(&SimulationEnvironment2::runSimulation, sim); 
    std::thread traditional(traditionalStateEstimator, vehicle, 1);
    std::thread overlaprun(&overlapGraphEstimator::updateGraph, &est);
    sleep(1);
    std::thread overlap(&overlapGraphEstimator::makeEstimation, &est);


    run.join();
    traditional.join();
    overlaprun.join();
    overlap.join();

    return 0;
}