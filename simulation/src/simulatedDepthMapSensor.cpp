#include "simulatedDepthMapSensor.h"
#include <cmath>
#include <stdexcept>
#include <limits>

simulatedDepthMapSensor::simulatedDepthMapSensor(std::pair<int, int> offset_)
    : offset(offset_), noise(0.0), dimension(2)
{
    minState = {0.0, 0.0};
    maxState = {15.0, 15.0};
}

std::vector<float> simulatedDepthMapSensor::getState()
{
    std::lock_guard<std::mutex> guard(sensorMutex);
    float minValue = std::numeric_limits<float>::max();
    std::pair<int, int> minPosition;
    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            if (value[i][j] < minValue)
            {
                minValue = value[i][j];
                minPosition = {i, j};
            }
        }
    }
    return {static_cast<float>(minPosition.first + offset.first), static_cast<float>(minPosition.second + offset.second)};
}

uint8_t simulatedDepthMapSensor::overlap(Sensor *other)
{
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
}

void simulatedDepthMapSensor::setValue(float value_[5][5])
{
    std::lock_guard<std::mutex> guard(sensorMutex);
    std::copy(&value_[0][0], &value_[0][0] + 25, &value[0][0]);
}

void simulatedDepthMapSensor::operateSensor()
{
    std::lock_guard<std::mutex> guard(sensorMutex);
    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 5; ++j)
        {
            value[i][j] = noiselessValue[i][j] + ((rand() % 100) / 100.0 * 2 * noise) - noise;
        }
    }
}
