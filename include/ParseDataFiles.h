#ifndef  PARSE_DATA_FILES_H
#define PARSE_DATA_FILES_H

#include <vector>
#include <Core.h>
#include <Geometry.h>
#include <iostream>
#include <fstream>

struct SensorFlag{
    bool imuAvail;
    bool uwbAvail;
    bool mocapAvail;
};

struct ImuData{
    Eigen::Vector3f accMeas;
    Eigen::Vector3f gyroMeas;
    Eigen::Vector3f magnMeas;
};

struct UwbData{
    Eigen::VectorXf meas;
};


#endif
