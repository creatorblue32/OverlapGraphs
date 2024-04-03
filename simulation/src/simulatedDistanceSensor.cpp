#include "simulatedDistanceSensor.h"

simulatedDistanceSensor::simulatedDistanceSensor(float minValue, float maxValue, float noiseLevel)
    : noise(noiseLevel), value(0.0), generator(rd()), distribution(minValue, maxValue)
{
}

std::vector<float> simulatedDistanceSensor::getState()
{
    return {value};
}

uint8_t simulatedDistanceSensor::overlap(Sensor *other)
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

void simulatedDistanceSensor::setValue(float value_)
{
    value = value_;
}

void simulatedDistanceSensor::operateSensor(float realValue)
{
    value = realValue + generateRandomFloat(-noise, noise);
}

float simulatedDistanceSensor::generateRandomFloat(float minValue, float maxValue)
{
    std::uniform_real_distribution<float> localDistribution(minValue, maxValue);
    return localDistribution(generator);
}
