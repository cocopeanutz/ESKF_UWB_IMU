#include "ESKF.h"
#include "unrolledFPFt.h"
#include "debug.h"

#define SQ(x) (x*x)
#define I_3 (Eigen::Matrix3f::Identity())
#define I_dx (Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>::Identity())

using namespace Eigen;
using namespace std;


ESKF::ESKF(Eigen::Vector3f a_gravity,
        const Eigen::Matrix<float, STATE_SIZE, 1>& initialState,
        const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& initalP,
        float var_acc, float var_omega, float var_acc_bias, float var_omega_bias,
        int delayHandling,
           int bufferL)
        : var_acc_(var_acc),
        var_omega_(var_omega),
        var_acc_bias_(var_acc_bias),
        var_omega_bias_(var_omega_bias),
        a_gravity_(a_gravity),
        nominalState_(initialState),
        P_(initalP) {

    // Jacobian of the state transition: page 59, eqn 269
    // Precompute constant part only
    F_x_.setZero();
    // dPos row
    F_x_.block<3, 3>(dPOS_IDX, dPOS_IDX) = I_3;
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dVEL_IDX) = I_3;
    // dTheta row
    // dAccelBias row
    F_x_.block<3, 3>(dAB_IDX, dAB_IDX) = I_3;
    // dGyroBias row
    F_x_.block<3, 3>(dGB_IDX, dGB_IDX) = I_3;

    // how to handle delayed messurements.
    delayHandling_ = delayHandling;
    bufferL_ = bufferL;
    recentPtr = 0;
    firstMeasTime = lTime(INT32_MAX,INT32_MAX);


}


Matrix<float, STATE_SIZE, 1> ESKF::makeState(
            const Vector3f& p,
            const Vector3f& v,
            const Quaternionf& q,
            const Vector3f& a_b,
            const Vector3f& omega_b) {
    Matrix<float, STATE_SIZE, 1> out;
    out << p, v, quatToHamilton(q).normalized(), a_b, omega_b;
    return out;
}

Matrix<float, dSTATE_SIZE, dSTATE_SIZE> ESKF::makeP(
        const Matrix3f& cov_pos,
        const Matrix3f& cov_vel,
        const Matrix3f& cov_dtheta,
        const Matrix3f& cov_a_b,
        const Matrix3f& cov_omega_b) {
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> P;
    P.setZero();
    P.block<3, 3>(dPOS_IDX, dPOS_IDX) = cov_pos;
    P.block<3, 3>(dVEL_IDX, dVEL_IDX) = cov_vel;
    P.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = cov_dtheta;
    P.block<3, 3>(dAB_IDX, dAB_IDX) = cov_a_b;
    P.block<3, 3>(dGB_IDX, dGB_IDX) = cov_omega_b;
    return P;
}

Matrix3f ESKF::getDCM() {
    return getQuat().matrix();
}

Quaternionf ESKF::quatFromHamilton(const Vector4f& qHam) {
    return Quaternionf(
        (Vector4f() <<
            qHam.block<3, 1>(1, 0), // x, y, z
            qHam.block<1, 1>(0, 0) // w
        ).finished());
}

Vector4f ESKF::quatToHamilton(const Quaternionf& q){
    return (Vector4f() <<
            q.coeffs().block<1, 1>(3, 0), // w
            q.coeffs().block<3, 1>(0, 0) // x, y, z
        ).finished();
}

Matrix3f ESKF::getSkew(const Vector3f& in) {
    Matrix3f out;
    out << 0, -in(2), in(1),
        in(2), 0, -in(0),
        -in(1), in(0), 0;
    return out;
}

Matrix3f ESKF::rotVecToMat(const Vector3f& in) {
    float angle = in.norm();
    Vector3f axis = (angle == 0) ? Vector3f(1, 0, 0) : in.normalized();
    AngleAxisf angAx(angle, axis);
    return angAx.toRotationMatrix();
}

Quaternionf ESKF::rotVecToQuat(const Vector3f& in) {
    float angle = in.norm();
    Vector3f axis = (angle == 0) ? Vector3f(1, 0, 0) : in.normalized();
    return Quaternionf(AngleAxisf(angle, axis));
}

Vector3f ESKF::quatToRotVec(const Quaternionf& q) {
    AngleAxisf angAx(q);
    return angAx.angle() * angAx.axis();
}

void ESKF::justPredict(const Vector3f& a_m, const Vector3f& omega_m, const float dt) {
    recentPtr ++;
    // DCM of current state
    Matrix3f Rot = getDCM();
    // Accelerometer measurement
    Vector3f acc_body = a_m - getAccelBias();
    Vector3f acc_global = Rot * acc_body;
    // Gyro measruement
    Vector3f omega = omega_m - getGyroBias();
    Vector3f delta_theta = omega * dt;
    Quaternionf q_delta_theta = rotVecToQuat(delta_theta);
    Matrix3f R_delta_theta = q_delta_theta.toRotationMatrix();


    DEBUG_PRINT("sanity check acc : " << (acc_global + a_gravity_));
    DEBUG_PRINT("sanity check accbias : " << getAccelBias());


    /**
    Below is already computed in unrolled manner below this implementation
    **/
    // Jacobian of the state transition (eqn 269, page 59)
    // Update dynamic parts only
    // dPos row
    F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt); // = I_3 * _dt
    // dVel row
    F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
    F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
    // dTheta row
    F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
    F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt); // = -I_3 * dt;

    // Predict P and inject variance (with diagonal optimization)
    P_ = F_x_*P_*F_x_.transpose();

    // Matrix<float, dSTATE_SIZE, dSTATE_SIZE> Pnew;
    // unrolledFPFt(P_, Pnew, dt,
    //     -Rot * getSkew(acc_body) * dt,
    //     -Rot * dt,
    //     R_delta_theta.transpose());
    // P_ = Pnew;

    // Inject process noise
    P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * SQ(dt);
    P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_omega_ * SQ(dt);
    P_.diagonal().block<3, 1>(dAB_IDX, 0).array() += var_acc_bias_ * dt;
    P_.diagonal().block<3, 1>(dGB_IDX, 0).array() += var_omega_bias_ * dt;
}

void ESKF::predictIMU(const Vector3f& a_m, const Vector3f& omega_m, const float dt) {
    recentPtr ++;
    // DCM of current state
    // Matrix3f Rot = getDCM();
    Matrix3f Rot = rotVecToMat(quatToRotVec(quatFromHamilton(this->getQuatVector())));
    // Accelerometer measurement
    Vector3f acc_body = a_m - getAccelBias();
    Vector3f acc_global = Rot.transpose() * acc_body;
    std::cout << "acceleration: \n" << acc_global << std::endl;
    // Gyro measruement
    Vector3f omega = omega_m - getGyroBias();
    Vector3f delta_theta = omega * dt;
    Quaternionf q_delta_theta = rotVecToQuat(delta_theta);
    Matrix3f R_delta_theta = q_delta_theta.toRotationMatrix();

    // std::cout << "sanity check acc : " << (acc_global - a_gravity_) << std::endl;
    DEBUG_PRINT("sanity check accbias : " << getAccelBias());

    // Nominal state kinematics (eqn 259, pg 58)

    Vector3f delta_pos = getVel()*dt + 0.5f*(acc_global - a_gravity_)*dt*dt;
    nominalState_.block<3, 1>(POS_IDX, 0) += delta_pos;
    nominalState_.block<3, 1>(VEL_IDX, 0) += (acc_global - a_gravity_)*dt;
    std::cout << "vel: \n" << getVel() << std::endl;
    std::cout << "pos: \n" << getPos() << std::endl;
    std::cout << "deltaPos: \n" << delta_pos << std::endl;

    nominalState_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat()*q_delta_theta).normalized();


    /**This is already computed in unrolled manner below
    // // Jacobian of the state transition (eqn 269, page 59)
    // // Update dynamic parts only
    // // dPos row
    // F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt); // = I_3 * _dt
    // // dVel row
    // F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
    // F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
    // // dTheta row
    // F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
    // F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt); // = -I_3 * dt;
    //
    // // Predict P and inject variance (with diagonal optimization)
    // P_ = F_x_*P_*F_x_.transpose();
    **/

    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> Pnew;
    unrolledFPFt(P_, Pnew, dt,
        -Rot * getSkew(acc_body) * dt,
        -Rot * dt,
        R_delta_theta.transpose());
    P_ = Pnew;

    // Inject process noise
    P_.diagonal().block<3, 1>(dVEL_IDX, 0).array() += var_acc_ * SQ(dt);
    P_.diagonal().block<3, 1>(dTHETA_IDX, 0).array() += var_omega_ * SQ(dt);
    P_.diagonal().block<3, 1>(dAB_IDX, 0).array() += var_acc_bias_ * dt;
    P_.diagonal().block<3, 1>(dGB_IDX, 0).array() += var_omega_bias_ * dt;
}

// eqn 280, page 62
Matrix<float, 4, 3> ESKF::getQ_dtheta() {
    Vector4f qby2 = 0.5f*getQuatVector();
    // Assing to letters for readability. Note Hamilton order.
    float w = qby2[0];
    float x = qby2[1];
    float y = qby2[2];
    float z = qby2[3];
    Matrix<float, 4, 3>Q_dtheta;
    Q_dtheta <<
        -x, -y, -z,
        w, -z, y,
        z, w, -x,
        -y, x, w;
    return Q_dtheta;
}

//get best time from history of state
int ESKF::getClosestTime(std::vector<std::pair<lTime,Eigen::Matrix<float, STATE_SIZE, 1>>>* ptr, lTime stamp){
    //we find the first time in the history that is older, or take the oldest one if the buffer does not extend far enough
    int complete = 0;
    int index = recentPtr;
    while(!complete){
        if(ptr->at(index%bufferL_).first <= stamp){
            if(!ptr->at(index%bufferL_).first.isZero()) return index%bufferL_;

            else{
                return recentPtr%bufferL_;
            }

        }
        index --; // scroll back in time.
        if(index <= recentPtr - bufferL_) complete =1;
    }
    return recentPtr%bufferL_;
}

//get best time from history of imu
int ESKF::getClosestTime(std::vector<imuMeasurement>*  ptr, lTime stamp){
    //we find the first time in the history that is older, or take the oldest one if the buffer does not extend far enough
    int complete = 0;
    int index = recentPtr;
    while(!complete){
        if(ptr->at(index%bufferL_).time <= stamp){
            if(!ptr->at(index%bufferL_).time.isZero()) return index%bufferL_;

            else{
                return recentPtr%bufferL_;
            }

        }
        index --; // scroll back in time.
        if(index <= recentPtr - bufferL_) complete =1;
    }
    return recentPtr%bufferL_;
}

//stamp + now
void ESKF::measurePos(const Vector3f& pos_meas, const Matrix3f& pos_covariance,lTime stamp ,lTime now) {
    // delta measurement
    if(firstMeasTime == lTime(INT32_MAX,INT32_MAX)) firstMeasTime = now;

    Vector3f delta_pos;
    if(delayHandling_ == noMethod || delayHandling_ == larsonAverageIMU){
        delta_pos = pos_meas - getPos();

    }

    if(delayHandling_ == applyUpdateToNew ){
        if(lastMeasurement < stateHistoryPtr_->at((recentPtr + 1)%bufferL_).first) firstMeasTime = now;
        if(stamp > firstMeasTime){
        int bestTimeIndex = getClosestTime(stateHistoryPtr_,stamp);
        delta_pos = pos_meas - stateHistoryPtr_->at(bestTimeIndex).second.block<3, 1>(POS_IDX, 0);
        }
        else delta_pos = pos_meas - getPos();
    }
    if(delayHandling_ == larsonAverageIMU){
        if(lastMeasurement < imuHistoryPtr_->at((recentPtr + 1)%bufferL_).time) firstMeasTime = now;
    }
    lastMeasurement = now;
    // H is a trivial observation of purely the position
    Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dPOS_IDX) = I_3;

    // Apply update
    update_3D(delta_pos, pos_covariance, H, stamp, now);
}

void ESKF::measureQuat(const Quaternionf& q_gb_meas, const Matrix3f& theta_covariance, lTime stamp ,lTime now) {
    // Transform the quaternion measurement to a measurement of delta_theta:
    // a rotation in the body frame from nominal to measured.
    // This is identical to the form of dtheta in the error_state,
    // so this becomes a trivial measurement of dtheta.
    if(firstMeasTime == lTime(INT32_MAX,INT32_MAX)) firstMeasTime = now;
    Quaternionf q_gb_nominal = getQuat();
    if(delayHandling_ == noMethod || delayHandling_ == larsonAverageIMU){
        q_gb_nominal = getQuat();
    }
    if(delayHandling_ == applyUpdateToNew){
        if(stamp > firstMeasTime){
        int bestTimeIndex = getClosestTime(stateHistoryPtr_,stamp);
        q_gb_nominal = quatFromHamilton(stateHistoryPtr_->at(bestTimeIndex).second.block<4, 1>(QUAT_IDX, 0));
        }
        else q_gb_nominal = getQuat();
    }
    Quaternionf q_bNominal_bMeas = q_gb_nominal.conjugate() * q_gb_meas;
    Vector3f delta_theta = quatToRotVec(q_bNominal_bMeas);
    // Because of the above construction, H is a trivial observation of dtheta
    Matrix<float, 3, dSTATE_SIZE> H;
    H.setZero();
    H.block<3, 3>(0, dTHETA_IDX) = I_3;

    // Apply update
    update_3D(delta_theta, theta_covariance, H, stamp, now);
}

void ESKF::update_3D(
        const Vector3f& delta_measurement,
        const Matrix3f& meas_covariance,
        const Matrix<float, 3, dSTATE_SIZE>& H,
        lTime stamp,
        lTime now) {
    //generate M matrix for time correction methods
    int bestTimeIndex;
    int normalPass = 1;
    if (delayHandling_ == larsonAverageIMU){
        if(stamp > firstMeasTime) normalPass = 0;
    }
    if(delayHandling_ == larsonAverageIMU && !normalPass){
            imuMeasurement avMeas = getAverageIMU(stamp);
            float dt = (now - stamp).toSec();
            Vector3f acc_body = avMeas.acc - getAccelBias();
            Vector3f omega = avMeas.gyro - getGyroBias();
            Vector3f delta_theta = omega * dt;
            Quaternionf q_delta_theta = rotVecToQuat(delta_theta);
            Matrix3f R_delta_theta = q_delta_theta.toRotationMatrix();
            bestTimeIndex = getClosestTime(stateHistoryPtr_,stamp);

         Matrix3f Rot = quatFromHamilton(stateHistoryPtr_->at(bestTimeIndex).second.block<4, 1>(QUAT_IDX, 0)).matrix();
         // dPos row
         F_x_.block<3, 3>(dPOS_IDX, dVEL_IDX).diagonal().fill(dt); // = I_3 * _dt
         // dVel row
         F_x_.block<3, 3>(dVEL_IDX, dTHETA_IDX) = -Rot * getSkew(acc_body) * dt;
         F_x_.block<3, 3>(dVEL_IDX, dAB_IDX) = -Rot * dt;
         // dTheta row
         F_x_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) = R_delta_theta.transpose();
         F_x_.block<3, 3>(dTHETA_IDX, dGB_IDX).diagonal().fill(-dt); // = -I_3 * dt;

    }

    // Kalman gain
    Matrix<float, dSTATE_SIZE, 3> PHt = P_*H.transpose();
    Matrix<float, dSTATE_SIZE, 3> K;
    if((delayHandling_ == noMethod || delayHandling_ == applyUpdateToNew || delayHandling_ == larsonAverageIMU)){
           K = PHt * (H*PHt + meas_covariance).inverse();
    }
    if(delayHandling_ == larsonAverageIMU && !normalPass){
        K = F_x_*K;
    }
    // Correction error state
    Matrix<float, dSTATE_SIZE, 1> errorState = K * delta_measurement;
    // Update P (simple form)
    // P = (I_dx - K*H)*P;
    // Update P (Joseph form)
    Matrix<float, dSTATE_SIZE, dSTATE_SIZE> I_KH = I_dx - K*H;
    if(delayHandling_ == noMethod || delayHandling_ == applyUpdateToNew){
        P_ = I_KH*P_*I_KH.transpose() + K*meas_covariance*K.transpose();
    }
    if(delayHandling_ == larsonAverageIMU  && !normalPass){

        P_ = P_ - K*H*PHistoryPtr_->at(bestTimeIndex).second*F_x_;
    }

    injectErrorState(errorState);
}

ESKF::imuMeasurement ESKF::getAverageIMU(lTime stamp){
    Vector3f accelAcc(0,0,0);
    Vector3f gyroAcc(0,0,0);
    int complete = 0;
    int index = recentPtr;
    int count = 0;
    while(!complete){
        if(imuHistoryPtr_->at(index%bufferL_).time >= stamp){
            if(!imuHistoryPtr_->at(index%bufferL_).time.isZero()){
                //should acc
                accelAcc += imuHistoryPtr_->at(index%bufferL_).acc;
                gyroAcc += imuHistoryPtr_->at(index%bufferL_).gyro;
                count ++;
            }
        }
        else{
            break;
        }
        index --; // scroll back in time.
        if(index <= recentPtr - bufferL_) complete =1;
    }
    accelAcc = accelAcc / count;
    gyroAcc = gyroAcc / count;
    ESKF::imuMeasurement ret;
    ret.acc = accelAcc;
    ret.gyro = gyroAcc;
    ret.time = imuHistoryPtr_->at(index%bufferL_).time;
    return ret;
}

void ESKF::injectErrorState(const Matrix<float, dSTATE_SIZE, 1>& error_state) {\
    // Inject error state into nominal state (eqn 282, pg 62)
    nominalState_.block<3, 1>(POS_IDX, 0) += error_state.block<3, 1>(dPOS_IDX, 0);
    nominalState_.block<3, 1>(VEL_IDX, 0) += error_state.block<3, 1>(dVEL_IDX, 0);
    Vector3f dtheta = error_state.block<3, 1>(dTHETA_IDX, 0);
    Quaternionf q_dtheta = rotVecToQuat(dtheta);
    nominalState_.block<4, 1>(QUAT_IDX, 0) = quatToHamilton(getQuat()*q_dtheta).normalized();
    nominalState_.block<3, 1>(AB_IDX, 0) += error_state.block<3, 1>(dAB_IDX, 0);
    nominalState_.block<3, 1>(GB_IDX, 0) += error_state.block<3, 1>(dGB_IDX, 0);

    // Reflect this tranformation in the P matrix, aka ESKF Reset
    // Note that the document suggests that this step is optional
    // eqn 287, pg 63
    Matrix3f G_theta = I_3 - getSkew(0.5f * dtheta);
    P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) =
            G_theta * P_.block<3, 3>(dTHETA_IDX, dTHETA_IDX) * G_theta.transpose();
}
