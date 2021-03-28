#include <ESKF.h>
#include <math.h>
#include <ctime>

#include <cstdlib>
#include <Core.h>
#include <random>
#include "ESKFattitude.h"
#include "ESKFuwb.h"
#include "ParseDataFiles.h"

const SensorContext gyroSimulatorContext = {
    {STD_WX,
    STD_WY,
    STD_WZ},
    {0, 0, 0},
    0,
    0
};

const Eigen::Vector3f ACC_BIAS = {
    0.0, 0.0, 0.0
};
const SensorContext accSimulatorContext = {
    {STD_AX,
    STD_AY,
    STD_AZ},
    gravityVec,
    0,
    0
};

const Eigen::Vector3f MAG_BIAS = {
    0.0, 0.0, 0.0
};
const SensorContext magnSimulatorContext = {
    {STD_MX,
    STD_MY,
    STD_MZ},
    globalMagnVec,
    0,
    0
};

const Eigen::Vector3f GYRO_BIAS = {
    0.0, 0.0, 0.0
};


/** This class will is a simulation for some vehicle which have the form of a dot
    This class will have orientation, acceleration, velocity and position

    TODO
    Temporarily this will inherit the base class, because it is easier for me as the ESKF
    have many function implementing motion, but this is a very bad way and should be changed
    later
    */
class DotVehicle : public ESKF{
    /**
    Based on this Paper to get maximum value
    acceleration            -> 2m/s
    velocity                -> 1.5 m/s^2
    angular velocity        -> 2 rad/s
    angular acceleration    -> 1.5 rad/s^2
    **/

    const float MAX_ACC = 0.05;
    const float MAX_VEL = 1.5;
    const float MAX_ANG_VEL = 2;
    const float MAX_ANG_ACC = 0.1;
    std::default_random_engine randomEngine;


    public:
    float timestamp = 0;
    ImuData imuData;

    Eigen::VectorXf tmpUwbMeas = Eigen::VectorXf(UWB_ANCHOR_COUNT);
    UwbData uwbData;

    Eigen::Vector3f acc={0, 0, 0};
    Eigen::Vector3f vel={0, 0, 0};
    Eigen::Vector3f pos={0, 0, 0};

    Eigen::Vector3f angacc={0, 0, 0};
    Eigen::Vector3f angvel={1, 0, 0};
    Eigen::Quaternionf orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.0f, Eigen::Vector3f(1, 0, 0)));

    DotVehicle(){
        srand(time(NULL));
        randomEngine.seed(time(NULL));
        uwbData.meas = tmpUwbMeas;
    }

    void generateLog(SensorFlag flag){
        /**
            Format IMU Avail, UWB Avail,
            acc,
            magn,
            gyro,
            uwb
        **/
        // printf("%f", uwbData.meas(0));
        // printf("<%d,%d,\\
        // %f,%f,%f,\\
        // %f,%f,%f,\\
        // %f,%f,%f,\\
        // %f,%f,%f>",
        //     flag.imuAvail, flag.uwbAvail,
        //     imuData.accMeas(0), imuData.accMeas(1), imuData.accMeas(2),
        //     imuData.magnMeas(0), imuData.magnMeas(1), imuData.magnMeas(2),
        //     imuData.gyroMeas(0), imuData.gyroMeas(1), imuData.gyroMeas(2),
        //     uwbData.meas(0), uwbData.meas(1), uwbData.meas(2)
        // );
    }
    /** time step the state of the vehicle to the newest state **/
    void timeStepState(float deltaTime, SensorFlag flag){
        timestamp += deltaTime;

        //Time Step based on previous step, with no randomness
        timeStepBasedOnPreviousState(deltaTime);
        /**
        Randomize the generated factor, for our case it is acceleration
        but maybe it can be velocity of acceleration or anything else
        **/
        generateRandomizedFactor();
        if(flag.imuAvail){
            imuData.accMeas = generateAccOrMagnMeas(acc, ACC_BIAS, accSimulatorContext);
            // std::cout << "acc: " << imuData.accMeas << std::endl;
            Eigen::Vector3f magBodyVector = {0.0, 0.0, 0.0};
            imuData.magnMeas = generateAccOrMagnMeas(magBodyVector, MAG_BIAS, magnSimulatorContext);
            // std::cout << "magn: " << imuData.magnMeas << std::endl;
            // imuData.gyroMeas = generateGyroMeas(angvel, GYRO_BIAS);
            imuData.gyroMeas = generateAccOrMagnMeas(angvel, GYRO_BIAS, gyroSimulatorContext);
        }

        if(flag.uwbAvail){
            uwbData.meas = generateUWBMeas();
        }
        generateLog(flag);
    }

    void timeStepBasedOnPreviousState(float dt){
        pos += vel * dt + 0.5 * acc * dt * dt;
        vel += acc * dt;

        Eigen::Quaternionf deltaTheta = rotVecToQuat(angvel * dt + 0.5 * angacc * dt * dt);
        angvel += angacc * dt;

        orientation *= deltaTheta;
        orientation.normalize();
    }

    void generateRandomizedFactor(){
        /**
        The model is a random walk of the velocity and angular velocity
        With acceleration and angular acceleration randomized with limit

        If velocity or angular velocity is more than the max, the kicking mechanism of the
        system will kick off, which basically is
        if the acceleration that we got doesn't decrease norm of the velocity, it will be
        randomized again, we will see the increase/decrease of the norm by
        seeing the derivative of the norm of the velocity
        **/
        // randomizeWithKickingMechanism(acc, MAX_ACC, vel, MAX_VEL);
        randomizeWithKickingMechanism(angacc, MAX_ANG_ACC, angvel, MAX_ANG_VEL);
    }


    /** This function generates measurement for accelerometer or magnetometer **/
    /** Basically (real acceleration on body frame+ gravity)* transformToBodyFrame + bias + perturbations **/
    Eigen::Vector3f generateAccOrMagnMeas(Eigen::Vector3f realAcc, Eigen::Vector3f bias,
        SensorContext ctx){
        /** Increase by reference Vector amount **/
        Eigen::Vector3f measAcc = realAcc + ctx.refVector;

        Eigen::Vector3f perturbations;
        for(int i = 0; i< 3; i++){
            std::normal_distribution<float> distribution(0.0, ctx.std[i]);
            perturbations(i) = distribution(randomEngine);
        }


        auto globalToBodyTransform = rotVecToMat(quatToRotVec(orientation));
        measAcc = globalToBodyTransform * measAcc + perturbations + bias;
        return measAcc;
    }

    //TODO give perturbations please
    Eigen::Vector3f generateGyroMeas(Eigen::Vector3f rotVel, Eigen::Vector3f bias){
        auto globalToBodyTransform = rotVecToMat(quatToRotVec(orientation));
        Eigen::Vector3f measGyro = globalToBodyTransform * rotVel + bias;
        return measGyro;
    }

    Eigen::VectorXf generateUWBMeas(){
        Eigen::VectorXf vec = Eigen::VectorXf(UWB_ANCHOR_COUNT);
        for(int i = 0; i< UWB_ANCHOR_COUNT; i++){
            Eigen::Vector3f distToAnchor = anchorsPos[i] - pos;

            float perturbations;
            for(int i = 0; i< 3; i++){
                std::normal_distribution<float> distribution(0.0, UWB_STD[i]);
                perturbations = distribution(randomEngine);
            }
            vec[i] = distToAnchor.norm() + perturbations;
        }
        return vec;
    }

    static void randomizeWithKickingMechanism(Eigen::Vector3f &acc, float maxAcc,
        Eigen::Vector3f &vel, float maxVelocity){
        // acc = {0, 0, 0};
        float derivativeOfVelNorm = 0;
        do{
            randomizeVector3f(acc, maxAcc);
            //The derivative of the velocity norm is
            //0.5 * 1/norm 2*vx*ax
            derivativeOfVelNorm = 0;
            if(vel.norm() == 0){
                derivativeOfVelNorm = acc.norm();
            }else{
                for(int i = 0; i< 3; i++){
                    derivativeOfVelNorm += 0.5 / vel.norm() * 2*vel(i)*acc(i);
                }
            }
        }while(vel.norm() > maxVelocity && derivativeOfVelNorm > 0);
    };

    static void randomizeVector3f(Eigen::Vector3f &vector, float maxNorm){
        //TODO change to normal distribution
        float norm = (float)(rand() % 10000)*maxNorm /10000;
        float x = (float)(rand() % 10000)*maxNorm /10000 - 0.5 * maxNorm;
        float y = (float)(rand() % 10000)*maxNorm /10000 - 0.5 * maxNorm;
        float z = (float)(rand() % 10000)*maxNorm /10000 - 0.5 * maxNorm;
        vector(0) = x;
        vector(1) = y;
        vector(2) = z;
        vector.normalize();
        vector *= norm;
    }
};
