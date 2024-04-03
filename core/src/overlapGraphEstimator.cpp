#include "overlapGraphEstimator.h"
#include <iostream>
#include <stdexcept>

overlapGraphEstimator::overlapGraphEstimator(std::vector<overlapGraphSensorInfo *> sensorIndex_, std::map<std::pair<uint8_t, uint8_t>, uint8_t> overlapGraph_, uint8_t dimension_)
    : sensorIndex(sensorIndex_), overlapGraph(overlapGraph_), dimension(dimension_)
{
}

std::vector<float> overlapGraphEstimator::makeEstimation()
{
        std::lock_guard<std::mutex> guard(graphMutex);

        std::vector<float> currentEstimation(dimension, 0.0);
        uint8_t currentActive = 0;
        for (uint8_t currentSensorIndex : reliableSensors)
        {
            std::vector<float> sensorStates = sensorIndex[currentSensorIndex]->sensor->getState();
            for (uint8_t i = 0; i < dimension; i++)
            {
                currentEstimation[i] += sensorStates[i];
            }
            currentActive++;
        }

        for (float &estimation : currentEstimation)
        {
            estimation /= (float)currentActive;
        }
        return currentEstimation;}

void overlapGraphEstimator::updateGraph()
{
    std::lock_guard<std::mutex> guard(graphMutex);
    for (auto &currentSensorInfo : sensorIndex)
    {
        uint16_t agreementSum = 0;
        for (auto &adjacentSensorIndex : currentSensorInfo->compatibleSensors)
        {
            auto &adjacentSensor = sensorIndex[adjacentSensorIndex];
            uint8_t prevAgreement = getAgreement(currentSensorInfo->index, adjacentSensorIndex);
            uint8_t currAgreement = currentSensorInfo->sensor->overlap(adjacentSensor->sensor);
            putAgreement(currentSensorInfo->index, adjacentSensorIndex, currAgreement);
            adjacentSensor->confidence += ((currAgreement - prevAgreement) / static_cast<float>(adjacentSensor->numCompatible));
            agreementSum += currAgreement;
        }
        currentSensorInfo->confidence = static_cast<uint8_t>(agreementSum / currentSensorInfo->numCompatible);
    }
}

uint8_t overlapGraphEstimator::getAgreement(uint8_t currentSensor, uint8_t adjacentSensor)
{
    auto it = overlapGraph.find(std::make_pair(std::min(currentSensor, adjacentSensor), std::max(currentSensor, adjacentSensor)));
    if (it != overlapGraph.end())
    {
        return it->second;
    }
    else
    {
        throw std::invalid_argument("Agreement not found");
    }
}

void overlapGraphEstimator::discriminate()
{

    std::lock_guard<std::mutex> guard(graphMutex);

    float confidenceAverage = 0;
    for (overlapGraphSensorInfo *currentSensor : sensorIndex)
    {
        confidenceAverage += currentSensor->confidence;
    }
    confidenceAverage = confidenceAverage / sensorIndex.size();

    reliableSensors.clear();
    for (overlapGraphSensorInfo *currentSensor : sensorIndex)
    {
        if (currentSensor->confidence >= confidenceAverage)
        {
            reliableSensors.insert(currentSensor->index);
        }
    }
}

void overlapGraphEstimator::putAgreement(uint8_t currentSensor, uint8_t adjacentSensor, uint8_t agreement)
{
    overlapGraph[std::make_pair(std::min(currentSensor, adjacentSensor), std::max(currentSensor, adjacentSensor))] = agreement;
}