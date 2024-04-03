#include "overlapGraphSensorInfo.h"

overlapGraphSensorInfo::overlapGraphSensorInfo(Sensor* sensor_, uint8_t index_, std::vector<uint8_t> compatibleSensors_)
    : sensor(sensor_), index(index_), compatibleSensors(compatibleSensors_), confidence(0), numCompatible(compatibleSensors_.size()) {
}