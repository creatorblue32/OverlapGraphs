#include "SimulationEnvironment2.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <limits.h>

SimulationEnvironment2::SimulationEnvironment2(std::vector<Sensor *> vehicleInput, std::vector<std::vector<float>> sequenceInput, float valueInput, overlapGraphEstimator *graphEstimator_)
    : sensorVehicle(vehicleInput), sequence(sequenceInput), realValue(valueInput), graphEstimator(graphEstimator_), simulationComplete(false)
{
    for (Sensor *sensor : sensorVehicle)
    {
        simulatedDepthMapSensor *depthSensor = dynamic_cast<simulatedDepthMapSensor *>(sensor);
        if (depthSensor)
        {
            depthMapSensorVehicle.push_back(depthSensor);
        }
    }
}

void SimulationEnvironment2::runSimulation()
{
    std::thread run(&SimulationEnvironment2::runSequence, this);
    run.join();
    std::cout << "Simulation complete. " << std::endl;
    exit(0);
}

void SimulationEnvironment2::runSequence()
{
    std::vector<float> overlapGraphEstimations = {};
    std::vector<float> traditionalEstimations = {};
    std::vector<float> overlapDurations = {};
    std::vector<float> traditionalDurations = {};
    std::vector<float> updateGraphDurations = {};

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
        for (int j = 0; j < 50; j++)
        {
            std::cout << "---------------- New Time Step ------------- Phase:" << std::to_string(phase) << " TS: " << std::to_string(j) << std::endl;
            for (int i = 0; i < depthMapSensorVehicle.size(); i++)
            {

                depthMapSensorVehicle[i]->operateSensor();

                auto start = std::chrono::high_resolution_clock::now();
                graphEstimator->updateGraph();
                auto stop = std::chrono::high_resolution_clock::now();
                updateGraphDurations.push_back((std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start)).count());

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

            graphEstimator->discriminate();

            // Request States every 1 second
            auto start = std::chrono::high_resolution_clock::now();
            traditionalEstimations.push_back(tradEstimator->makeEstimation(sensorVehicle)[0]);
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
    sim_data.push_back(updateGraphDurations);

    column_names.push_back("traditionalEstDiff");
    column_names.push_back("overlapEstDiff");
    column_names.push_back("tradDuration");
    column_names.push_back("overlapDuration");
    column_names.push_back("updateDuration");

    std::ofstream file("simulation/results/simData.csv");

    if (!file.is_open())
    {
        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd)) != NULL)
        {
            std::cout << "Current working directory: " << cwd << std::endl;
        }
        else
        {
            std::cerr << "getcwd() error" << std::endl;
        }
        std::cerr << "Failed to open file: "
                  << "simData.csv" << std::endl;
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
    float updateTime = std::accumulate(updateGraphDurations.begin(), updateGraphDurations.end(), 0.0f) / updateGraphDurations.size();

    std::cout << "Average Traditional Time: " << tradTime << std::endl;
    std::cout << "Average Overlap Estimation Time: " << overlapTime << std::endl;
    std::cout << "Average Update Graph Time: " << updateTime << std::endl;

    simulationComplete = true;
}

void SimulationEnvironment2::saveToCSV(const std::vector<std::vector<float>> &data, const std::vector<std::string> &columnNames, const std::string &filename)
{
    std::ofstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (const auto &columnName : columnNames)
    {
        file << columnName << ",";
    }
    file << "\n";

    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            file << data[i][j];
            if (j < data[i].size() - 1)
                file << ",";
        }
        file << "\n";
    }

    file.close();
}
