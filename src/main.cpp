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

class simulatedDistanceSensor : public Sensor {
    public:
    uint8_t dimension = 1;
    std::vector<float> minState = {0.0};
    std::vector<float> maxState = {10.0};
    float noise;
    float value;
    std::vector<float> getState() override {
        return {value};
    }
    void setValue(float value_){
        value = value_;
    }
    void operateSensor(float realValue) {
        value = realValue + generateRandomFloat((-1*noise),noise);
    }
    uint8_t overlap(Sensor* other) override {
        if(dynamic_cast<simulatedDistanceSensor*>(other)){
            simulatedDistanceSensor * otherDistSensor =  dynamic_cast<simulatedDistanceSensor*>(other);
            return 255-(abs(value - otherDistSensor->getState()[0])*(255/(maxState[0]-minState[0])));
        }
        else { throw std::invalid_argument("Incompatible Sensor in Compatibility Map.");}
    }
};

class simulatedDepthMapSensor : public Sensor {
    public:
    uint8_t dimension = 2;
    std::vector<float> minState = {0.0, 0.0};
    std::vector<float> maxState = {15.0, 15.0};
    float noiselessValue[5][5];
    float noise;
    std::pair<int,int> offset;
    std::mutex sensorMutex;
    float value[5][5];

    simulatedDepthMapSensor(std::pair<int,int> offset_){
        offset = offset_;
    }

    std::vector<float> getState() override {
        sensorMutex.lock();       
        int minVal = std::numeric_limits<int>::max();
        int minX = 0, minY = 0;
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                if (value[i][j] < minVal) {
                    minVal = value[i][j];
                    minX = i;
                    minY = j;
                }
            }
        }
        sensorMutex.unlock();
        return {(float) (minX+offset.first-2), (float) (minY+offset.second-2)};
    }

    void setValue(float value_[5][5]){
        std::lock_guard<std::mutex> guard(sensorMutex);
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                value[i][j] = value_[i][j];
            }
        }
    }

    void printValue(){
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                std::cout << value[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void operateSensor() {
        std::lock_guard<std::mutex> guard(sensorMutex);
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                value[i][j] = noiselessValue[i][j]+generateRandomFloat((-1*noise),noise);
            }
        }
    }

    uint8_t overlap(Sensor* other) override {
        if(dynamic_cast<simulatedDepthMapSensor*>(other)){
            simulatedDepthMapSensor * otherDepthMapSensor =  dynamic_cast<simulatedDepthMapSensor*>(other);
            int startRow = std::max(offset.first, otherDepthMapSensor->offset.first);
            int endRow = std::min(offset.first + 5, otherDepthMapSensor->offset.first + 5);
            int startCol = std::max(offset.second, otherDepthMapSensor->offset.second);
            int endCol = std::min(offset.second + 5, otherDepthMapSensor->offset.second + 5);
            std::lock_guard<std::mutex> guard(sensorMutex);
            std::lock_guard<std::mutex> guard2(otherDepthMapSensor->sensorMutex);

            float accumulatedError = 0.0;
            uint8_t overlapCount = 0;

            for (int i = startRow; i < endRow; ++i) {
                for (int j = startCol; j < endCol; ++j) {
                    int adj_i1 = i - offset.first;
                    int adj_j1 = j - offset.second;
                    int adj_i2 = i - otherDepthMapSensor->offset.first;
                    int adj_j2 = j - otherDepthMapSensor->offset.second;

                    if (adj_i1 >= 0 && adj_i1 < 5 && adj_j1 >= 0 && adj_j1 < 5 &&
                        adj_i2 >= 0 && adj_i2 < 5 && adj_j2 >= 0 && adj_j2 < 5) {
                        accumulatedError += abs(value[adj_i1][adj_j1]-otherDepthMapSensor->value[adj_i2][adj_j2]);
                        overlapCount++;
                    }
                }
            }
            if (overlapCount == 0){
                throw std::invalid_argument("No Sensor Overlap Region.");
            }
            return 255-((accumulatedError / (float) overlapCount)*(255/(maxState[0]-minState[0])));
        }
        else { throw std::invalid_argument("Tried to Calculate Overlap with Incompatible Sensor");}
    }
};

class overlapGraphSensorInfo{
    public:
        overlapGraphSensorInfo(Sensor* sensor_, uint8_t index_, std::vector<uint8_t> compatibleSensors_){
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
        std::vector<overlapGraphSensorInfo*> sensorIndex;
        std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph;
        uint8_t dimension;
        std::mutex graphMutex;

        overlapGraphEstimator(std::vector<overlapGraphSensorInfo*> sensorIndex_, std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph_, uint8_t dimension_){
            sensorIndex = sensorIndex_;
            overlapGraph = overlapGraph_;
            dimension = dimension_;
        } 

        std::vector<float> makeEstimation(){
            std::lock_guard<std::mutex> guard(graphMutex);

            //Determine the average confidence for all sensors in the graph
            float confidenceAverage = 0;
            for (overlapGraphSensorInfo* currentSensor: sensorIndex){
                confidenceAverage += currentSensor->confidence;
            }
            confidenceAverage = confidenceAverage / sensorIndex.size();

            //Sum all of the states outputted by sensors that have above the average confidence 
            std::vector<float> currentEstimation(dimension, 0.0);
            uint8_t currentActive = 0;
            for (overlapGraphSensorInfo* currentSensor: sensorIndex){
                if(currentSensor->confidence >= confidenceAverage){
                    std::vector<float> sensorStates = currentSensor->sensor->getState();
                    for(uint8_t i = 0; i < dimension; i++){
                        currentEstimation[i] += sensorStates[i];
                    }
                    currentActive++;
                }
            }

            //Divide each sum by the number of sensors included in that sum
            for (float &estimation : currentEstimation) {
                estimation /= (float) currentActive;
            }

            return currentEstimation;
        }
        
        void updateGraph(){
            while(true){
                for(overlapGraphSensorInfo* currentSensor : sensorIndex){
                    std::lock_guard<std::mutex> guard(graphMutex);
                    uint16_t totalAgreements = 0;
                    for(uint8_t adjacentSensor : currentSensor->compatibleSensors){
                        uint8_t prevAgreement = getAgreement(currentSensor->index, adjacentSensor);
                        uint8_t currAgreement = currentSensor->sensor->overlap(sensorIndex[adjacentSensor]->sensor);
                        totalAgreements += currAgreement;
                    }
                    currentSensor->confidence = (uint8_t) (totalAgreements / currentSensor->numCompatible);
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

class SimulationEnvironment1 {
public:
    std::vector<simulatedDistanceSensor*> vehicle;
    std::vector<Sensor*> sensorVehicle;
    overlapGraphEstimator* graphEstimator;

    std::vector<std::vector<float>> sequence;
    bool simulationComplete;

    SimulationEnvironment1(std::vector<Sensor*> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator* graphEstimator_) {
        vehicle = {};
        for (Sensor* sensor : vehicleInput){
            simulatedDistanceSensor * current = dynamic_cast<simulatedDistanceSensor*>(sensor);
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
                overlapGraphEstimations.push_back(graphEstimator->makeEstimation()[0]);
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
class SimulationEnvironment2 {
public:
    std::vector<simulatedDepthMapSensor*> depthMapSensorVehicle;
    std::vector<Sensor*> sensorVehicle;
    overlapGraphEstimator* graphEstimator;
    std::vector<std::vector<float>> sequence;
    bool simulationComplete;

    SimulationEnvironment2(std::vector<Sensor*> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator* graphEstimator_) {
        depthMapSensorVehicle = {};
        float value = 10.0;
        float vehicleFieldOfView[9][9];
        
        for (int layer = 0; layer < (10) / 2; ++layer) {
            int fillValue = 10 - layer;
            for (int i = layer; i < 9 - layer; ++i) {
                for (int j = layer; j < 9 - layer; ++j) {
                    vehicleFieldOfView[i][j] = fillValue;
                }
            }
        }

        for (Sensor* sensor : vehicleInput){
            simulatedDepthMapSensor * currentSensor = dynamic_cast<simulatedDepthMapSensor*>(sensor);
            if(currentSensor){
                depthMapSensorVehicle.push_back(currentSensor);
            }
        }

        for(simulatedDepthMapSensor * depthMapSensor : depthMapSensorVehicle){
                for (int i = 0; i < 5; ++i) {
                    for (int j = 0; j < 5; ++j) {
                        depthMapSensor->noiselessValue[i][j] = vehicleFieldOfView[i+(depthMapSensor->offset.first)+2][j+(depthMapSensor->offset.second)+2];
                    }
                }
                depthMapSensor->noise = 0.0;
                depthMapSensor->operateSensor();
        }
        
        sensorVehicle = vehicleInput;
        sequence = sequenceInput;
        simulationComplete = false;
        realValue = valueInput;
        graphEstimator = graphEstimator_;
    }


    void operateSensors(){
        while(!simulationComplete){
            for(int i = 0; i < depthMapSensorVehicle.size(); i++){
                depthMapSensorVehicle[i]->operateSensor();
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
                depthMapSensorVehicle[sensor]->noise = sequence[phase][sensor];
            }
            for(int i = 0; i < 5; i++){
                //Request States every 1 second
                auto start = std::chrono::high_resolution_clock::now();
                traditionalEstimations.push_back(traditionalStateEstimator(sensorVehicle, 1)[0]);
                auto stop = std::chrono::high_resolution_clock::now();
                traditionalDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start)).count());

                auto start2 = std::chrono::high_resolution_clock::now();
                overlapGraphEstimations.push_back(graphEstimator->makeEstimation()[0]);
                auto stop2 = std::chrono::high_resolution_clock::now();
                overlapDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop2 - start2)).count());


                sleep(1);
            }
            std::cout << "Phase " << phase << " complete. " << std::endl;
        }
        
        std::vector<float> tradDifferences = {};

        //std::cout << "Traditional Estimations: ";
        for (float element : traditionalEstimations) {
            tradDifferences.push_back(abs(element));
            //std::cout << element << " ";
        }
        //std::cout << std::endl;

        std::vector<float> overlapDifferences = {};

        //std::cout << "Overlap Graph Estimations: ";
        for (float element : overlapGraphEstimations) {
            overlapDifferences.push_back(abs(element));
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

int runTest1(){
        //Make my test sensors, vehicle
        simulatedDistanceSensor * firstSensor = new simulatedDistanceSensor();
        simulatedDistanceSensor * secondSensor = new simulatedDistanceSensor();
        simulatedDistanceSensor * thirdSensor = new simulatedDistanceSensor();
        std::vector<Sensor*> vehicle = {firstSensor, secondSensor, thirdSensor};


        //Make my sequence
        std::vector<std::vector<float>> sequence = {{0.0,0.0,4.0},{0.0,4.0,0.0},{4.0, 0.0,0.0}};


        //Make graph estimator
        overlapGraphSensorInfo* firstSensorInfo = new overlapGraphSensorInfo(firstSensor, 0, {1,2});

        std::vector<overlapGraphSensorInfo*> sensorMap = {firstSensorInfo, new overlapGraphSensorInfo(secondSensor,1,{0,2}), new overlapGraphSensorInfo(thirdSensor,2,{0,1})}; 
        std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0,1},0},{{1,2},0},{{0,2},0}};
        overlapGraphEstimator est(sensorMap, overlapGraph, 1);

        // Make simulation environment
        SimulationEnvironment1 sim = SimulationEnvironment1(vehicle, sequence, 5.0, &est);


        //Run Simulation, Traditional State Estimation, Overlap Graph Estimation
        std::thread run(&SimulationEnvironment1::runSimulation, sim); 
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

int runTest2(){
        simulatedDepthMapSensor * firstSensor = new simulatedDepthMapSensor(std::pair<int, int>(-1,-1));
        simulatedDepthMapSensor * secondSensor = new simulatedDepthMapSensor(std::pair<int, int>(1,-1));
        simulatedDepthMapSensor * thirdSensor = new simulatedDepthMapSensor(std::pair<int, int>(1,1));
        
        std::vector<Sensor*> vehicle = {firstSensor, secondSensor, thirdSensor};

        std::vector<std::vector<float>> sequence = {{0.0,0.0,5.0},{0.0,5.0,0.0},{5.0, 0.0,0.0}};


        std::vector<overlapGraphSensorInfo*> sensorMap = {new overlapGraphSensorInfo(firstSensor,0,{1,2}), new overlapGraphSensorInfo(secondSensor,1,{0,2}), new overlapGraphSensorInfo(thirdSensor,2,{0,1})}; 
        std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0,1},0},{{1,2},0},{{0,2},0}};
        overlapGraphEstimator est(sensorMap, overlapGraph, 2);

        SimulationEnvironment2 sim = SimulationEnvironment2(vehicle, sequence, 5, &est);


        //Run Simulation, Traditional State Estimation, Overlap Graph Estimation
        std::thread run(&SimulationEnvironment2::runSimulation, sim); 
        //std::thread traditional(traditionalStateEstimator, vehicle, 1);
        std::thread overlaprun(&overlapGraphEstimator::updateGraph, &est);
        //std::thread overlap(&overlapGraphEstimator::makeEstimation, &est);


        run.join();
        //traditional.join();
        overlaprun.join();
        //overlap.join();

        return 0;  
    }

int main() {
    runTest1();
}