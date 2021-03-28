#include <ESKF.h>
#include <iostream>
#include <chrono>
#include <ParseDataFiles.h>
#include <VehicleSimulator.h>
#include <math.h>
#include <lTime.h>
#include <fenv.h>
#include <debug.h>

using namespace Eigen;
#define SQ(x) (x*x)
#define I_3 (Eigen::Matrix3f::Identity())

template<int row>
Eigen::VectorXf vectorEigenFromArray(float array[]){
    Eigen::VectorXf vector(row);
    for(int i = 0; i< row; i++){
        vector << array[i];
    }
    return vector;
}

template <int row, int col>
Eigen::Matrix<float, row, col> matrixEigenFromArray (float array[][col]){
    Eigen::Matrix<float, row, col> matrix(row, col);
    for(int i = 0; i< row; i++){
        for(int j = 0; j< col; j++){
            matrix[i][j] << array[i][j];
        }
    }
    return matrix;
}

extern "C"
__attribute__((visibility("default")))
ESKF * glueCodeCreateESKF(){
    float sigma_accel = 5; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    float sigma_gyro = 0.3; // [rad/s] (value derived from Noise Spectral Density in datasheet)

    float sigma_accel_drift = 0.0001f*0.001f*sigma_accel; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 0.0001f*0.001f*sigma_gyro; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 10*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 10*sigma_gyro_drift; // [rad/s]
    ESKF * eskfObj = new ESKF(
                // Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
                gravityVec, // Acceleration due to gravity in global frame
                ESKF::makeState(
                        Vector3f(0, 0, 1.31), // init pos
                        Vector3f(0, 0, 0), // init vel
                        Quaternionf(AngleAxisf(0.0f, Vector3f(0, 0, 1))), // init quaternion
                            Vector3f(0, 0, 0), // init accel bias
                            Vector3f(0.0, 0.0, 0) // init gyro bias
                ),

                ESKF::makeP(
                    SQ(sigma_init_pos) * I_3,
                    SQ(sigma_init_vel) * I_3,
                    SQ(sigma_init_dtheta) * I_3,
                    SQ(sigma_init_accel_bias) * I_3,
                    SQ(sigma_init_gyro_bias) * I_3
                ),
                SQ(sigma_accel),
                SQ(sigma_gyro),
                SQ(sigma_accel_drift),
                SQ(sigma_gyro_drift),
                ESKF::delayTypes::noMethod,100);
    return eskfObj;
}



extern "C"
__attribute__((visibility("default")))
void glueCodePredictESKF(ESKF * eskfPtr,
     float accMeas[3], float gyroMeas[3], float deltaTime){
    ImuData imu;
    imu.accMeas = Vector3f(accMeas);
    imu.gyroMeas = Vector3f(gyroMeas);
    // printf("gyroMeas: %f %f %f\n", gyroMeas[0], gyroMeas[1], gyroMeas[2]);
    eskfPtr->predictIMU(imu.accMeas, imu.gyroMeas, deltaTime);
}

extern "C"
__attribute__((visibility("default")))
void glueCodeUpdateEskfImu(ESKF * eskfPtr,
    float accMeas[3], float magnMeas[3]){
    ImuData imu;
    imu.accMeas = Vector3f(accMeas);
    imu.magnMeas = Vector3f(magnMeas);

    eskfPtr->measureMagnetometer(imu.magnMeas);
    eskfPtr->measureAccelerometer(imu.accMeas-eskfPtr->getAccelBias());
}

extern "C"
__attribute__((visibility("default")))
void glueCodeUpdateEskfAccelerometer(ESKF * eskfPtr,
    float accMeas[3]){
        ImuData imu;
        imu.accMeas = Vector3f(accMeas);
        eskfPtr->measureAccelerometer(imu.accMeas-eskfPtr->getAccelBias());
}

extern "C"
__attribute__((visibility("default")))
void glueCodeUpdateEskfMagnetometer(ESKF * eskfPtr,
    float magnMeas[3]){
    ImuData imu;
    imu.magnMeas = Vector3f(magnMeas);

    eskfPtr->measureMagnetometer(imu.magnMeas);
}



extern "C"
__attribute__((visibility("default")))
void glueCodeUpdateEskfUwb(ESKF * eskfPtr,
    float uwbMeas[3]){
    UwbData uwb;
    uwb.meas = Vector3f(uwbMeas);
    // printf("before Measure UWB: %f\n", eskfPtr->getPos()[0]);
    eskfPtr->measureUWB(uwb.meas);
    // printf("After Measure UWB: %f\n", eskfPtr->getPos()[0]);
}


extern "C"
__attribute__((visibility("default")))
void glueCodeGetPosition(float retPos[3], ESKF * eskfPtr){
    // printf("testing inside c: %f\n", eskfPtr->getPos()[0]);
    auto pos = eskfPtr->getPos();
    for(int i = 0; i< 3; i++){
        retPos[i] = pos(i);
    }
}

extern "C"
__attribute__((visibility("default")))
void glueCodeGetVelocity(float* retVel, ESKF * eskfPtr){
    auto vel = eskfPtr->getVel();
    for(int i = 0; i< 3; i++){
        retVel[i] = vel(i);
    }
}

extern "C"
__attribute__((visibility("default")))
void glueCodeGetQuaternion(float* retQuat, ESKF * eskfPtr){
    auto quat = eskfPtr->getQuatVector();
    for(int i = 0; i< 4; i++){
        retQuat[i] = quat(i);
    }
}
