#pragma once
//clang-linter doesn't get #pragma once, what an asshole

#ifndef ESKF_UWB
#define ESKF_UWB

#define UWB_ANCHOR_COUNT 3

#include "ESKF.h"


const Eigen::Vector3f S1 = Eigen::Vector3f(0, 0.3, 0.15);
const Eigen::Vector3f S2 = Eigen::Vector3f(13.2, 0, 0.47);
const Eigen::Vector3f S3 = Eigen::Vector3f(12.4, 10.25, 0.12);
const Eigen::Vector3f S4 = Eigen::Vector3f(3, 0, 100);


const Eigen::Vector3f anchorsPos[UWB_ANCHOR_COUNT] = {
    S1,
    S2,
    S3
#if UWB_ANCHOR_COUNT == 4
    ,S4
#endif
};


#define UWB_STD_CONS 0.5
//this one should be used in the simulator no?
const float UWB_STD[UWB_ANCHOR_COUNT] = {
    UWB_STD_CONS,
    UWB_STD_CONS,
    UWB_STD_CONS
#if UWB_ANCHOR_COUNT == 4
    ,UWB_STD_CONS
#endif
};


#if UWB_ANCHOR_COUNT == 4
    const Eigen::Matrix<float, UWB_ANCHOR_COUNT, UWB_ANCHOR_COUNT>
UWB_COVARIANCE = Eigen::Matrix4f::Identity() * UWB_STD_CONS * UWB_STD_CONS;
#else
const Eigen::Matrix<float, UWB_ANCHOR_COUNT, UWB_ANCHOR_COUNT>
UWB_COVARIANCE = Eigen::Matrix3f::Identity() * UWB_STD_CONS * UWB_STD_CONS;
#endif

#endif
