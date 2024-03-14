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
#include <iostream>
#include <fstream>
#include <vector>

// HELPER FUNCTIONS
float generateRandomFloat(float minValue, float maxValue)
{
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> distribution(minValue, maxValue);
    return distribution(generator);
}

// Saves a 2D Vector to CSV
void saveToCSV(const std::vector<std::vector<float>> &data, const std::vector<std::string> &columnNames, const std::string &filename)
{
    std::ofstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // Write column headers
    file << "time";
    for (const auto &columnName : columnNames)
    {
        file << "," << columnName;
    }
    file << "\n";

    // Write data to the file
    for (size_t i = 0; i < data[0].size(); ++i)
    {
        file << i + 1; // "time" column advancing by 1 for every record
        for (size_t j = 0; j < data.size(); ++j)
        {
            file << "," << data[j][i];
        }
        file << "\n";
    }

    file.close();
}

// CLASS DEFINITIONS
class Sensor
{
public:
    virtual std::vector<float> getState() = 0;
    virtual uint8_t overlap(Sensor *other) = 0;
    std::vector<float> minState;
    std::vector<float> maxState;
    uint8_t dimension;
};

class simulatedDistanceSensor : public Sensor
{
public:
    uint8_t dimension = 1;
    std::vector<float> minState = {0.0};
    std::vector<float> maxState = {10.0};
    float noise;
    float value;
    std::vector<float> getState() override
    {
        return {value};
    }
    void setValue(float value_)
    {
        value = value_;
    }
    void operateSensor(float realValue)
    {
        value = realValue + generateRandomFloat((-1 * noise), noise);
    }
    uint8_t overlap(Sensor *other) override
    {
        if (dynamic_cast<simulatedDistanceSensor *>(other))
        {
            simulatedDistanceSensor *otherDistSensor = dynamic_cast<simulatedDistanceSensor *>(other);
            return 255 - (abs(value - otherDistSensor->getState()[0]) * (255 / (maxState[0] - minState[0])));
        }
        else
        {
            throw std::invalid_argument("Incompatible Sensor in Compatibility Map.");
        }
    }
};

class simulatedDepthMapSensor : public Sensor
{
public:
    uint8_t dimension = 2;
    std::vector<float> minState = {0.0, 0.0};
    std::vector<float> maxState = {15.0, 15.0};
    float noiselessValue[5][5];
    float value[5][5];
    float noise;
    std::pair<int, int> offset;
    std::mutex sensorMutex;

    simulatedDepthMapSensor(std::pair<int, int> offset_)
    {
        offset = offset_;
    }

    std::vector<float> getState() override
    {
        sensorMutex.lock();
        int minVal = std::numeric_limits<int>::max();
        int minX = 0, minY = 0;
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                if (value[i][j] < minVal)
                {
                    minVal = value[i][j];
                    minX = i;
                    minY = j;
                }
            }
        }
        sensorMutex.unlock();
        return {(float)(minX + offset.first - 2), (float)(minY + offset.second - 2)};
    }

    void setValue(float value_[5][5])
    {
        std::lock_guard<std::mutex> guard(sensorMutex);
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                value[i][j] = value_[i][j];
            }
        }
    }

    void printValue()
    {
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                std::cout << value[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void operateSensor()
    {
        std::lock_guard<std::mutex> guard(sensorMutex);
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 5; ++j)
            {
                value[i][j] = noiselessValue[i][j] + generateRandomFloat((-1 * noise), noise);
            }
        }
    }

    uint8_t overlap(Sensor *other) override
    {
        if (dynamic_cast<simulatedDepthMapSensor *>(other))
        {
            simulatedDepthMapSensor *otherDepthMapSensor = dynamic_cast<simulatedDepthMapSensor *>(other);
            int startRow = std::max(offset.first, otherDepthMapSensor->offset.first);
            int endRow = std::min(offset.first + 5, otherDepthMapSensor->offset.first + 5);
            int startCol = std::max(offset.second, otherDepthMapSensor->offset.second);
            int endCol = std::min(offset.second + 5, otherDepthMapSensor->offset.second + 5);
            std::lock_guard<std::mutex> guard(sensorMutex);
            std::lock_guard<std::mutex> guard2(otherDepthMapSensor->sensorMutex);

            float accumulatedError = 0.0;
            uint8_t overlapCount = 0;

            for (int i = startRow; i < endRow; ++i)
            {
                for (int j = startCol; j < endCol; ++j)
                {
                    int adj_i1 = i - offset.first;
                    int adj_j1 = j - offset.second;
                    int adj_i2 = i - otherDepthMapSensor->offset.first;
                    int adj_j2 = j - otherDepthMapSensor->offset.second;

                    if (adj_i1 >= 0 && adj_i1 < 5 && adj_j1 >= 0 && adj_j1 < 5 &&
                        adj_i2 >= 0 && adj_i2 < 5 && adj_j2 >= 0 && adj_j2 < 5)
                    {
                        accumulatedError += abs(value[adj_i1][adj_j1] - otherDepthMapSensor->value[adj_i2][adj_j2]);
                        overlapCount++;
                    }
                }
            }
            if (overlapCount == 0)
            {
                throw std::invalid_argument("No Sensor Overlap Region.");
            }
            return 255 - ((accumulatedError / (float)overlapCount) * (255 / (maxState[0] - minState[0])));
        }
        else
        {
            throw std::invalid_argument("Tried to Calculate Overlap with Incompatible Sensor");
        }
    }
};

class overlapGraphSensorInfo
{
public:
    overlapGraphSensorInfo(Sensor *sensor_, uint8_t index_, std::vector<uint8_t> compatibleSensors_)
    {
        sensor = sensor_;
        compatibleSensors = compatibleSensors_;
        index = index_;
        numCompatible = compatibleSensors_.size();
        confidence = 0;
    }
    Sensor *sensor;
    uint8_t index;
    uint8_t confidence;
    uint8_t numCompatible;
    std::vector<uint8_t> compatibleSensors;
};

class overlapGraphEstimator
{
public:
    std::vector<overlapGraphSensorInfo *> sensorIndex;
    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph;
    uint8_t dimension;
    std::mutex graphMutex;

    overlapGraphEstimator(std::vector<overlapGraphSensorInfo *> sensorIndex_, std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph_, uint8_t dimension_)
    {
        sensorIndex = sensorIndex_;
        overlapGraph = overlapGraph_;
        dimension = dimension_;
    }

    std::vector<float> makeEstimation()
    {
        std::lock_guard<std::mutex> guard(graphMutex);

        // Determine the average confidence for all sensors in the graph
        float confidenceAverage = 0;
        for (overlapGraphSensorInfo *currentSensor : sensorIndex)
        {
            confidenceAverage += currentSensor->confidence;
        }
        confidenceAverage = confidenceAverage / sensorIndex.size();

        // Sum all of the states outputted by sensors that have above the average confidence
        std::vector<float> currentEstimation(dimension, 0.0);
        uint8_t currentActive = 0;
        for (overlapGraphSensorInfo *currentSensor : sensorIndex)
        {

            if (currentSensor->confidence >= confidenceAverage)
            {
                std::cout << "Including " << std::to_string(currentSensor->index) << " with confidence " << std::to_string(currentSensor->confidence) << std::endl;
                std::vector<float> sensorStates = currentSensor->sensor->getState();
                for (uint8_t i = 0; i < dimension; i++)
                {
                    currentEstimation[i] += sensorStates[i];
                }
                currentActive++;
            }
            else
            {
                std::cout << " >> EXCLUDING " << std::to_string(currentSensor->index) << " with confidence " << std::to_string(currentSensor->confidence) << std::endl;
            }
        }

        // Divide each sum by the number of sensors included in that sum
        for (float &estimation : currentEstimation)
        {
            estimation /= (float)currentActive;
        }

        std::cout << "Overlap Estimation: x-" << std::to_string(currentEstimation[0]) << " y-" << std::to_string(currentEstimation[1]) << std::endl;
        std::cout << std::endl;

        return currentEstimation;
    }

    void updateGraph()
    {
        while (true)
        {
            for (overlapGraphSensorInfo *currentSensor : sensorIndex)
            {
                std::lock_guard<std::mutex> guard(graphMutex);
                uint16_t agreementSum = 0;
                for (uint8_t adjacentSensorIndex : currentSensor->compatibleSensors)
                {
                    overlapGraphSensorInfo *adjacentSensor = sensorIndex[adjacentSensorIndex];
                    uint8_t prevAgreement = getAgreement(currentSensor->index, adjacentSensorIndex);
                    uint8_t currAgreement = currentSensor->sensor->overlap(adjacentSensor->sensor);
                    putAgreement(currentSensor->index, adjacentSensorIndex, currAgreement);
                    adjacentSensor->confidence += ((currAgreement - prevAgreement) / adjacentSensor->numCompatible);
                    agreementSum += currAgreement;
                }
                currentSensor->confidence = (uint8_t)(agreementSum / currentSensor->numCompatible);
            }
        }
    }

private:
    uint8_t getAgreement(uint8_t currentSensor, uint8_t adjacentSensor)
    {
        if (adjacentSensor > currentSensor)
        {
            return overlapGraph[{adjacentSensor, currentSensor}];
        }
        else
        {
            return overlapGraph[{adjacentSensor, currentSensor}];
        }
    }
    void putAgreement(uint8_t currentSensor, uint8_t adjacentSensor, uint8_t agreement)
    {
        if (adjacentSensor > currentSensor)
        {
            overlapGraph[{adjacentSensor, currentSensor}] = agreement;
        }
        else
        {
            overlapGraph[{adjacentSensor, currentSensor}] = agreement;
        }
    }
};

std::vector<float> traditionalStateEstimator(std::vector<Sensor *> vehicle, uint8_t num_states)
{
    std::vector<float> total = {};
    for (int i = 0; i < num_states; i++)
    {
        total.push_back(0.0);
    }

    std::vector<float> current_state = {};
    int numSensors = vehicle.size();
    for (int sensor = 0; sensor < numSensors; sensor++)
    {
        current_state = vehicle[sensor]->getState();
        for (int state = 0; state < num_states; state++)
        {
            total[state] += current_state[state];
        }
    }

    for (int state = 0; state < num_states; state++)
    {
        total[state] = total[state] / (float)numSensors;
    }

    std::cout << "Traditional Estimation: x-" << std::to_string(total[0]) << " y-" << std::to_string(total[1]) << std::endl;
    std::cout << std::endl;
    return total;
}

class SimulationEnvironment2
{
public:
    std::vector<simulatedDepthMapSensor *> depthMapSensorVehicle;
    std::vector<Sensor *> sensorVehicle;
    overlapGraphEstimator *graphEstimator;
    std::vector<std::vector<float>> sequence;
    bool simulationComplete;

    SimulationEnvironment2(std::vector<Sensor *> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator *graphEstimator_)
    {
        depthMapSensorVehicle = {};
        float value = 10.0;
        float vehicleFieldOfView[9][9];

        for (int layer = 0; layer < (10) / 2; ++layer)
        {
            int fillValue = 10 - layer;
            for (int i = layer; i < 9 - layer; ++i)
            {
                for (int j = layer; j < 9 - layer; ++j)
                {
                    vehicleFieldOfView[i][j] = fillValue;
                }
            }
        }

        for (Sensor *sensor : vehicleInput)
        {
            simulatedDepthMapSensor *currentSensor = dynamic_cast<simulatedDepthMapSensor *>(sensor);
            if (currentSensor)
            {
                depthMapSensorVehicle.push_back(currentSensor);
            }
        }

        for (simulatedDepthMapSensor *depthMapSensor : depthMapSensorVehicle)
        {
            for (int i = 0; i < 5; ++i)
            {
                for (int j = 0; j < 5; ++j)
                {
                    depthMapSensor->noiselessValue[i][j] = vehicleFieldOfView[i + (depthMapSensor->offset.first) + 2][j + (depthMapSensor->offset.second) + 2];
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

    void runSequence()
    {
        std::vector<float> overlapGraphEstimations = {};
        std::vector<float> traditionalEstimations = {};
        std::vector<float> overlapDurations = {};
        std::vector<float> traditionalDurations = {};

        sleep(1);
        std::cout << "Beginning Simulation." << std::endl;

        std::vector<std::vector<float>> sim_data = {};
        std::vector<std::string> column_names = {};
        for (int sensor = 0; sensor < sensorVehicle.size(); sensor++)
        {
            column_names.push_back("sensorVal" + std::to_string(sensor));
            column_names.push_back("sensorValConfidence" + std::to_string(sensor));
            sim_data.push_back({});
            sim_data.push_back({});
        }

        for (int phase = 0; phase < sequence.size(); phase++)
        {
            for (int sensor = 0; sensor < sequence[phase].size(); sensor++)
            {
                depthMapSensorVehicle[sensor]->noise = sequence[phase][sensor];
            }
            for (int j = 0; j < 5; j++)
            {
                std::cout << "---------------- New Round -------------" << std::endl;
                for (int i = 0; i < depthMapSensorVehicle.size(); i++)
                {

                    depthMapSensorVehicle[i]->operateSensor();
                    std::cout << "Sensor " << std::to_string(i) << " Noise: " << depthMapSensorVehicle[i]->noise;
                    std::cout << " Sensor State: X:" << depthMapSensorVehicle[i]->getState()[0] << " Y:" << depthMapSensorVehicle[i]->getState()[1];
                    std::cout << std::endl;
                }

                std::vector<float> current_state = {};
                int numSensors = sensorVehicle.size();
                for (int sensor = 0; sensor < numSensors; sensor++)
                {
                    current_state = sensorVehicle[sensor]->getState();
                    for (int state = 0; state < 1; state++)
                    {
                        sim_data[2 * sensor].push_back(current_state[0]);
                        sim_data[2 * sensor + 1].push_back(graphEstimator->sensorIndex[sensor]->confidence);
                    }
                }

                // Request States every 1 second
                auto start = std::chrono::high_resolution_clock::now();
                traditionalEstimations.push_back(traditionalStateEstimator(sensorVehicle, 1)[0]);
                auto stop = std::chrono::high_resolution_clock::now();
                traditionalDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start)).count());

                auto start2 = std::chrono::high_resolution_clock::now();
                overlapGraphEstimations.push_back(graphEstimator->makeEstimation()[0]);
                auto stop2 = std::chrono::high_resolution_clock::now();
                overlapDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop2 - start2)).count());
                std::cout << std::endl;
            }
            std::cout << "Phase " << phase << " complete. " << std::endl;
        }

        std::vector<float> tradDifferences = {};

        // std::cout << "Traditional Estimations: ";
        for (float element : traditionalEstimations)
        {
            tradDifferences.push_back(abs(element));
            // std::cout << element << " ";
        }
        // std::cout << std::endl;

        std::vector<float> overlapDifferences = {};

        // std::cout << "Overlap Graph Estimations: ";
        for (float element : overlapGraphEstimations)
        {
            overlapDifferences.push_back(abs(element));
            // std::cout << element << " ";
        }
        // std::cout << std::endl;

        sim_data.push_back(tradDifferences);
        sim_data.push_back(overlapDifferences);
        sim_data.push_back(traditionalDurations);
        sim_data.push_back(overlapDurations);

        column_names.push_back("traditionalEstDiff");
        column_names.push_back("overlapEstDiff");
        column_names.push_back("tradDuration");
        column_names.push_back("overlapDuration");

        std::ofstream file("/Users/elyasmasrour/Documents/localRepos/overlap-graphs/data/simData.csv");

        if (!file.is_open())
        {
            std::cerr << "Failed to open file: "
                      << "/Users/elyasmasrour/Documents/localRepos/overlap-graphs/data/simData.csv" << std::endl;
            return;
        }

        // Write column headers
        file << "time";
        for (const auto &columnName : column_names)
        {
            file << "," << columnName;
        }
        file << "\n";

        std::cout << "Got right before Writing. " << std::endl;

        for (const auto &row : sim_data)
        {
            std::cout << "Row size: " << row.size() << " ";
            for (float element : row)
            {
                std::cout << element << " ";
            }
            std::cout << std::endl;
        }

        // Write data to the file
        for (size_t i = 0; i < sim_data[0].size(); ++i)
        {
            file << i + 1; // "time" column advancing by 1 for every record
            for (size_t j = 0; j < sim_data.size(); ++j)
            {
                file << "," << sim_data[j][i];
            }
            file << "\n";
        }

        file.close();

        float overlapError = std::accumulate(overlapDifferences.begin(), overlapDifferences.end(), 0.0f) / overlapDifferences.size();
        float tradError = std::accumulate(tradDifferences.begin(), tradDifferences.end(), 0.0f) / tradDifferences.size();

        std::cout << "Average Traditional Error: " << tradError << std::endl;
        std::cout << "Average Overlap Error: " << overlapError << std::endl;

        float tradTime = std::accumulate(traditionalDurations.begin(), traditionalDurations.end(), 0.0f) / traditionalDurations.size();
        float overlapTime = std::accumulate(overlapDurations.begin(), overlapDurations.end(), 0.0f) / overlapDurations.size();

        std::cout << "Average Traditional Time: " << tradTime << std::endl;
        std::cout << "Average Overlap Time: " << overlapTime << std::endl;

        simulationComplete = true;
    }

    void runSimulation()
    {
        std::thread run(&SimulationEnvironment2::runSequence, this);
        run.join();
        std::cout << "Simulation complete. " << std::endl;
        exit(0);
    }

private:
    int realValue;
};

int runTest2()
{
    simulatedDepthMapSensor *firstSensor = new simulatedDepthMapSensor(std::pair<int, int>(-1, -1));
    simulatedDepthMapSensor *secondSensor = new simulatedDepthMapSensor(std::pair<int, int>(1, -1));
    simulatedDepthMapSensor *thirdSensor = new simulatedDepthMapSensor(std::pair<int, int>(1, 1));
    simulatedDepthMapSensor *fourthSensor = new simulatedDepthMapSensor(std::pair<int, int>(-1, 1));

    std::vector<Sensor *> vehicle = {firstSensor, secondSensor, thirdSensor, fourthSensor};

    std::vector<std::vector<float>> sequence = {{10.0, 0.1, 0.1, 0.1}, {0.1, 10.0, 0.1, 0.1}, {0.1, 0.1, 10.0, 0.1}};

    std::vector<overlapGraphSensorInfo *> sensorMap = {new overlapGraphSensorInfo(firstSensor, 0, {1, 2, 3}),
                                                       new overlapGraphSensorInfo(secondSensor, 1, {0, 2, 3}),
                                                       new overlapGraphSensorInfo(thirdSensor, 2, {0, 1, 3}),
                                                       new overlapGraphSensorInfo(fourthSensor, 3, {0, 1, 2})};

    std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph = {{{0, 1}, 0}, {{1, 2}, 0}, {{0, 2}, 0}, {{0, 3}, 0}, {{1, 3}, 0}, {{2, 3}, 0}};
    overlapGraphEstimator est(sensorMap, overlapGraph, 2);

    SimulationEnvironment2 sim = SimulationEnvironment2(vehicle, sequence, 5, &est);

    // Run Simulation, Traditional State Estimation, Overlap Graph Estimation
    std::thread runSimulation(&SimulationEnvironment2::runSimulation, sim);
    std::thread runOverlapGraph(&overlapGraphEstimator::updateGraph, &est);

    runSimulation.join();
    runOverlapGraph.join();

    return 0;
}

int main()
{
    runTest2();
}