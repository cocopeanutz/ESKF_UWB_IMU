#include "ESKF.h"
#include "unrolledFPFt.h"
#include "debug.h"
#define I_dx (Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity())

#include <iostream>

using namespace std;
void ESKF::measureUWB (Eigen::VectorXf uwbMeas) {
    auto delta_measurement = getUWB_Observ(uwbMeas);
    DEBUG_PRINT("uwbMeas: " << uwbMeas);
    DEBUG_PRINT("deltaMeasurement: " << delta_measurement  );
    auto uwbH = getUWB_H();
    ESKF::update_UWB(
            delta_measurement,
            UWB_COVARIANCE,
            uwbH);
}
/**
    Get the Measured Value of the UWB in the form of vector
    Ref: Fei Liu et al, page 6
    **/
Eigen::VectorXf ESKF::getUWB_Observ (Eigen::VectorXf uwbMeas) {
    auto vec = Eigen::VectorXf(UWB_ANCHOR_COUNT);
    Eigen::Vector3f nominalPos = getPos();
    for(int i = 0; i< UWB_ANCHOR_COUNT; i++){
        vec[i] = uwbMeas[i] - (anchorsPos[i]-nominalPos).norm();
    }
    return vec;
}


/**
    Get the H of the UWB measurement, where H is the Jacobian of the
    h(x, err_x) with respect to err_x
    Ref: Fei Liu et al, page 6
    **/
Eigen::Matrix<float, UWB_ANCHOR_COUNT, dSTATE_SIZE>
ESKF::getUWB_H(){
    Eigen::Matrix<float, UWB_ANCHOR_COUNT, dSTATE_SIZE> H;

    for(int i = 0; i < UWB_ANCHOR_COUNT; i++){
        Eigen::Vector3f anchorPos = anchorsPos[i];
        Eigen::Vector3f nominalPos = getPos();
        float distNorm = (anchorPos-nominalPos).norm();
        int j = 0;
        for(; j< 3; j++){
            H(i,j) = -(anchorPos[j] - nominalPos[j])/distNorm;
        }
        for(; j< dSTATE_SIZE; j++){
            H(i,j) = 0;
        }
    }
    return H;
}

Eigen::Matrix<float, UWB_ANCHOR_COUNT, UWB_ANCHOR_COUNT>
ESKF::getUWB_Covariance(){
    Eigen::Matrix<float, UWB_ANCHOR_COUNT, UWB_ANCHOR_COUNT> cov;
	for(int i = 0; i< UWB_ANCHOR_COUNT; i++){
		for(int j = 0; j< UWB_ANCHOR_COUNT; j++){
			if(i==j){
				cov << UWB_STD[j];
			}else{
				cov << 0;
			}
		}
	}
    return cov;
}



void ESKF::update_UWB(
        const Eigen::VectorXf& delta_measurement,
        const Eigen::Matrix<float, UWB_ANCHOR_COUNT, UWB_ANCHOR_COUNT>& meas_covariance,
        const Eigen::Matrix<float, UWB_ANCHOR_COUNT, dSTATE_SIZE>& H) {

    // Kalman gain
    Eigen::Matrix<float, dSTATE_SIZE, UWB_ANCHOR_COUNT> PHt = P_*H.transpose();
    Eigen::Matrix<float, dSTATE_SIZE, UWB_ANCHOR_COUNT> K;

    K = PHt * (H*PHt + meas_covariance).inverse();
    DEBUG_PRINT(K);
    // Correction error state
    Eigen::Matrix<float, dSTATE_SIZE, 1> errorState = K * delta_measurement;

    DEBUG_PRINT("Error state:" << errorState);
    DEBUG_PRINT("H * ErrorState"<< H*errorState);
    DEBUG_PRINT("deltaMeas: " <<  delta_measurement );
    // Update P (simple form)
    // P = (I_dx - K*H)*P;
    // Update P (Joseph form)
    Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dx - K*H;
    P_ = I_KH*P_*I_KH.transpose() + K*meas_covariance*K.transpose();

    injectErrorState(errorState);
}
