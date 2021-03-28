#include "ESKF.h"
#include "unrolledFPFt.h"
#include <cmath>
#include "ESKFattitude.h"
#include "debug.h"

#define I_dx (Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity())

/** Get the constanta to multiply
the standard Deviation of the sensor magnetic/accelerometer
to get the covariance, based on some fuzzy residual algorithm(?)
Reference Fei Liu et al, page 8
**/

//TODO ref vector ? float exception
float ESKF::getMultiplierConst(Eigen::Vector3f measWithoutBias, Eigen::Vector3f referenceVector,
        const float alpha, const float maxC) {
    float measNorm = measWithoutBias.norm();
    float refNorm = referenceVector.norm();
    /**Magnitude Residual
    Reference Fei Liu et al, page 8
    **/
    float magnitudeResidue = 0;
    if(measNorm == 0 || refNorm == 0){
        magnitudeResidue = 7;//big number
    }else{
        magnitudeResidue = std::max(measNorm/refNorm,
            refNorm/measNorm);
    }
    /**Directional Residual
    Reference Fei Liu et al, page 8
    **/
    float directionalResidue = 0;
    if(measNorm == 0 || refNorm == 0){
        directionalResidue = 7;//big number
    }else{
        directionalResidue = ((measWithoutBias/measNorm)-(referenceVector/refNorm)).norm();
    }

    return std::min(std::exp(alpha*magnitudeResidue*directionalResidue),
                maxC);
}

/** Get Sensor Covariance for magnetometer/accelerometer based on
Reference Fei Liu et al, page 8
**/
Eigen::Matrix3f ESKF::getAttitudeSensorCovariance(Eigen::Vector3f measWithoutBias, SensorContext ctx) {

    float multConst = getMultiplierConst(measWithoutBias, ctx.refVector,
                                ctx.alpha, ctx.maxMult);
    Eigen::Matrix<float, 3, 3> cov;

    float stdX = ctx.std[0];
    float stdY = ctx.std[1];
    float stdZ = ctx.std[2];

    cov <<
        stdX*stdX*multConst, 0, 0,
        0, stdY*stdY*multConst, 0,
        0, 0, stdZ*stdZ*multConst;
    return cov;
}



const Eigen::Matrix<float, 3, dSTATE_SIZE>
ESKF::getAttitudeSensorH(Eigen::Matrix3f bodyToGlobal, SensorContext ctx){
    Eigen::Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    // H.block<3, 3>(0, dTHETA_IDX) = bodyToGlobal*getSkew(ctx.refVector);
    H.block<3, 3>(0, dTHETA_IDX) = getSkew(ctx.refVector);

    DEBUG_PRINT("H attitude sensor: << "  << H);
    return -H;
}

Eigen::Vector3f ESKF::getObservationVector(Eigen::Vector3f measWithoutBias,
    Eigen::Matrix<float, 3, 3> bodyToGlobal,
    SensorContext ctx){
        // return measWithoutBias - bodyToGlobal*ctx.refVector;
        return bodyToGlobal*measWithoutBias - ctx.refVector;
    }

void ESKF::update_3D_attitudeSensor(
            const Eigen::Vector3f& delta_measurement,
            const Eigen::Matrix3f& meas_covariance,
            const Eigen::Matrix<float, 3, dSTATE_SIZE>& H) {
        // Kalman gain
        Eigen::Matrix<float, dSTATE_SIZE, 3> PHt = P_*H.transpose();
        Eigen::Matrix<float, dSTATE_SIZE, 3> K;
        K = PHt * (H*PHt + meas_covariance).inverse();
        // Correction error state
        Eigen::Matrix<float, dSTATE_SIZE, 1> errorState = K * delta_measurement;
        // Update P (simple form)
        // P = (I_dx - K*H)*P;
        // Update P (Joseph form)
        Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dx - K*H;
        P_ = I_KH*P_*I_KH.transpose() + K*meas_covariance*K.transpose();
        injectErrorState(errorState);
}

void ESKF::measureAttitudeSensor(const Eigen::Vector3f& measWithoutBias, SensorContext ctx) {
    Eigen::Quaternionf q_gb_nominal = getQuat();
    Eigen::Matrix3f bodyToGlobal = rotVecToMat(quatToRotVec(q_gb_nominal)).transpose();

    Eigen::Matrix<float, 3, dSTATE_SIZE> H = getAttitudeSensorH(bodyToGlobal, ctx);
    Eigen::Vector3f observ = getObservationVector(measWithoutBias, bodyToGlobal, ctx);
    // printf("observation Vector: %f %f %f\n", observ(0), observ(1), observ(2));
    Eigen::Matrix3f cov = getAttitudeSensorCovariance(measWithoutBias, ctx);
    // Apply update
    update_3D_attitudeSensor(observ, cov, H);
}
