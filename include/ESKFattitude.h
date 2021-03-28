#pragma once

#ifndef ESKF_ATTITUDE
#define ESKF_ATTITUDE

#include <Core.h>

//9.812 m/s^2
const Eigen::Vector3f gravityVec = Eigen::Vector3f(0, 0, 9.781);

/**
Global Magnetic reference Vector
Ref: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
Singapore 2021-01-05
Model Used: 	WMM-2020 	More information
Latitude: 	1.3521° N
Longitude: 	103.8198° E
Elevation: 	0.0 km Mean Sea Level
Declination ( + E  | - W )      0.1228°
Inclination ( + D  | - U )      -13.4250°
Horizontal Intensity 	  	    40,985.3 nT
North Comp (+ N  | - S)         40,985.2 nT
East Comp  (+ E  | - W)         87.9 nT
Vertical Comp (+ D  | - U)      -9,783.0 nT
Total Field                     42,136.7 nT
**/
const Eigen::Vector3f globalMagnVec = Eigen::Vector3f(40.9853, 0, 9.7830);
// const Eigen::Vector3f globalMagnVec = Eigen::Vector3f(0, 0, 9.7830);

#define STD_ACC 0.0225  // m/s^2
#define STD_MAG 0.15    // miuT
#define STD_GYRO 0.05   //

#define STD_AX STD_ACC
#define STD_AY STD_ACC
#define STD_AZ STD_ACC
#define ALPHA_ACC 16
#define MAX_MULT_ACC 400

#define STD_MX STD_MAG
#define STD_MY STD_MAG
#define STD_MZ STD_MAG
#define ALPHA_MAG 8
#define MAX_MULT_MAG 400

#define STD_WX STD_GYRO
#define STD_WY STD_GYRO
#define STD_WZ STD_GYRO
struct SensorContext {
    //Standard Deviation
    float std[3];

    //Reference Vector
    Eigen::Vector3f refVector;
    float alpha;
    float maxMult;
};

const SensorContext accContext = {
    {STD_AX,
    STD_AY,
    STD_AZ},
    gravityVec,
    ALPHA_ACC,
    MAX_MULT_ACC
};
const SensorContext magnContext = {
    {STD_MX,
    STD_MY,
    STD_MZ},
    globalMagnVec,
    ALPHA_MAG,
    MAX_MULT_MAG
};

#endif
