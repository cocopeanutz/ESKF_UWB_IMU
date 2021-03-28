#include <ESKF.h>
#include <iostream>
#include <chrono>
#include <ParseDataFiles.h>
#include <VehicleSimulator.h>
#include <math.h>
#include <lTime.h>
#include <fenv.h>
#include <debug.h>


#define SQ(x) (x*x)
#define GRAVITY 	9.781  // Singapore g value.
#define I_3 (Eigen::Matrix3f::Identity())

using namespace Eigen;
using namespace std;

double difference(ESKF eskfRef,DotVehicle dotVehicle);
Vector3f angleDifference(ESKF eskfTest, DotVehicle realVehicle);
double horizontalDifference(ESKF eskfTest, DotVehicle realVehicle);
Vector3f velocityDifference(ESKF eskfTest, DotVehicle realVehicle);

float simulate();
int main(int argc, char** argv) {
    /** Trap NaN first appearance
        Ref: https://stackoverflow.com/questions/11933919/debug-c-code-catch-first-nan-appearance/11946546
    **/
    feenableexcept(FE_INVALID | FE_OVERFLOW);
    float average = 0;
    float n = 0;
    for(int i = 0; i < 50 ; i ++ ){
        n+=1;
        average = average * ((n-1)/n) + simulate()/n;
    }
    cout << "average is " << average << std::endl;
}

float simulate(){
    // float sigma_accel = STD_ACC; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    // float sigma_accel = 5; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    // float sigma_gyro = 10; // [rad/s] (value derived from Noise Spectral Density in datasheet)

    float sigma_accel = 5; // [m/s^2]  (value derived from Noise Spectral Density in datasheet)
    float sigma_gyro = 3; // [rad/s] (value derived from Noise Spectral Density in datasheet)

    float sigma_accel_drift = 0.0001f*0.001f*sigma_accel; // [m/s^2 sqrt(s)] (Educated guess, real value to be measured)
    float sigma_gyro_drift = 0.0001f*0.001f*sigma_gyro; // [rad/s sqrt(s)] (Educated guess, real value to be measured)

    float sigma_init_pos = 1.0; // [m]
    float sigma_init_vel = 0.1; // [m/s]
    float sigma_init_dtheta = 1.0; // [rad]
    float sigma_init_accel_bias = 10*sigma_accel_drift; // [m/s^2]
    float sigma_init_gyro_bias = 10*sigma_gyro_drift; // [rad/s]

    cout << "before ESKF spoof" << std::endl;
    ESKF eskfSpoof(
            // Vector3f(0, 0, -GRAVITY), // Acceleration due to gravity in global frame
            gravityVec, // Acceleration due to gravity in global frame
            ESKF::makeState(
                    Vector3f(0, 0, 0), // init pos
                    Vector3f(0, 0, 0), // init vel
                    Quaternionf(AngleAxisf(0.0f, Vector3f(0, 0, 1))), // init quaternion
                        // Vector3f(-1.26, -1.09, -1.977), // init accel bias
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


    SensorFlag sensorFlag;
    sensorFlag.imuAvail = true;
    sensorFlag.uwbAvail = true;
    double deltaTime = 0.01;//in seconds
    DotVehicle dotVehicle;
    dotVehicle.timeStepState(deltaTime, sensorFlag);
    for(int i = 0; i< 10001; i++){
    // for(int i = 0; i< 10; i++){
        DEBUG_PRINT("iteration>>>>>");
        DEBUG_PRINT(i);
        // cout << ">>>>>>>>>>>>>iteration "<< i << std::endl;
        sensorFlag.imuAvail = true;
        // sensorFlag.uwbAvail = true;
        if(!(i % 100)){
            sensorFlag.uwbAvail = true;
        }else{
            sensorFlag.uwbAvail = false;
        }

        ImuData imu = dotVehicle.imuData;
        UwbData uwb = dotVehicle.uwbData;
        Vector3f angleDifference_;
        // cout << "angleDifferenceBefore " << angleDifference(eskfSpoof, dotVehicle).norm() << std::endl;
        //type imu data + uwb data
        if(sensorFlag.imuAvail){
            // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
            // cout << "Velocity Difference Before:"<< velocityDifference(eskfSpoof, dotVehicle) << std::endl;
            // cout << "orientation" << ESKF::quatToRotVec(ESKF::quatFromHamilton(eskfSpoof.getQuatVector())) << std::endl;
            // cout << "angleDifferenceBefore " << angleDifference(eskfSpoof, dotVehicle).norm() << std::endl;
            eskfSpoof.predictIMU(imu.accMeas, imu.gyroMeas, deltaTime);
            // cout << "Velocity Difference after:"<< velocityDifference(eskfSpoof, dotVehicle) << std::endl;
            // cout << "angleDifferenceAfter " << angleDifference(eskfSpoof, dotVehicle).norm() << std::endl;
            //
            // cout << "Difference:"<< horizontalDifference(eskfSpoof, dotVehicle) << std::endl;
            // cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
            eskfSpoof.measureMagnetometer(imu.magnMeas);
            eskfSpoof.measureAccelerometer(imu.accMeas-eskfSpoof.getAccelBias());
        }

        // cout << "angleDifferenceAfter " << angleDifference(eskfSpoof, dotVehicle).norm() << std::endl;

        auto beforeUwb =  difference(eskfSpoof, dotVehicle);
        //if uwb
        if(sensorFlag.uwbAvail){
            //             cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
            // cout << "Velocity Difference Before:"<< velocityDifference(eskfSpoof, dotVehicle) << std::endl;
            // cout << "orientation" << ESKF::quatToRotVec(ESKF::quatFromHamilton(eskfSpoof.getQuatVector())) << std::endl;
            // cout << "angleDifferenceBefore " << angleDifference(eskfSpoof, dotVehicle).norm() << std::endl;
            eskfSpoof.measureUWB(uwb.meas);
            // cout << "Velocity Difference after:"<< velocityDifference(eskfSpoof, dotVehicle) << std::endl;
            // cout << "angleDifferenceAfter " << angleDifference(eskfSpoof, dotVehicle).norm() << std::endl;
            //
            // cout << "Difference:"<< horizontalDifference(eskfSpoof, dotVehicle) << std::endl;
            // cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        }
        dotVehicle.timeStepState(deltaTime, sensorFlag);
    }
    cout << "Final Difference:"<< horizontalDifference(eskfSpoof, dotVehicle) << std::endl;
    return horizontalDifference(eskfSpoof, dotVehicle);
}

double horizontalDifference(ESKF eskfTest, DotVehicle realVehicle){
    Vector3f posRef = realVehicle.pos;
    DEBUG_PRINT("real Pos:" << posRef );
    Vector3f posTest = eskfTest.getPos();
    DEBUG_PRINT("eskf Pos:" << posRef );
    Vector3f out = posTest - posRef;
    return sqrt(out(0)*out(0)+out(1)*out(1));
}

double difference(ESKF eskfTest, DotVehicle realVehicle){
    Vector3f posRef = realVehicle.pos;
    DEBUG_PRINT("real Pos:" << posRef );
    Vector3f posTest = eskfTest.getPos();
    DEBUG_PRINT("eskf Pos:" << posRef );
    Vector3f out = posTest - posRef;
    return out.norm();
}

Vector3f angleDifference(ESKF eskfTest, DotVehicle realVehicle){
    Vector3f realOrientation = ESKF::quatToRotVec(realVehicle.orientation);
    Vector3f predictedOrientation = ESKF::quatToRotVec(ESKF::quatFromHamilton(eskfTest.getQuatVector()));
    return (VectorXf) (realOrientation - predictedOrientation);
}

Vector3f velocityDifference(ESKF eskfTest, DotVehicle realVehicle){
    Vector3f velRef = realVehicle.vel;
    Vector3f velTest = eskfTest.getVel();
    Vector3f out = velTest - velRef;
    return out;
}
