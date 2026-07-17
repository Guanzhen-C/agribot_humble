#include <cassert>
#include <cmath>

#include "common/earth.h"
#include "common/rotation.h"
#include "kf-gins/insmech.h"

#include "agribot_rl_nav/rtk_gi_engine.hpp"

namespace {

double wrapAngle(double angle) {

    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

Eigen::RowVector3d yawJacobianFromPhiAngle(const Eigen::Vector3d &euler) {

    const double pitch = euler[1];
    const double yaw = euler[2];
    const double tan_pitch = std::tan(pitch);

    Eigen::RowVector3d jacobian;
    jacobian << tan_pitch * std::cos(yaw), tan_pitch * std::sin(yaw), 1.0;
    return jacobian;
}

}

RtkGIEngine::RtkGIEngine(GINSOptions &options) {

    options_ = options;
    options_.print_options();
    timestamp_ = 0.0;

    Cov_.resize(RANK, RANK);
    Qc_.resize(NOISERANK, NOISERANK);
    dx_.resize(RANK, 1);
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();

    auto imunoise = options_.imunoise;
    Qc_.block(ARW_ID, ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block(VRW_ID, VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
    Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();

    initialize(options_.initstate, options_.initstate_std);
}

void RtkGIEngine::initialize(const NavState &initstate, const NavState &initstate_std) {

    pvacur_.pos = initstate.pos;
    pvacur_.vel = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
    imuerror_ = initstate.imuerror;

    pvapre_ = pvacur_;

    ImuError imuerror_std = initstate_std.imuerror;
    Cov_.block(P_ID, P_ID, 3, 3) = initstate_std.pos.cwiseProduct(initstate_std.pos).asDiagonal();
    Cov_.block(V_ID, V_ID, 3, 3) = initstate_std.vel.cwiseProduct(initstate_std.vel).asDiagonal();
    Cov_.block(PHI_ID, PHI_ID, 3, 3) = initstate_std.euler.cwiseProduct(initstate_std.euler).asDiagonal();
    Cov_.block(BG_ID, BG_ID, 3, 3) = imuerror_std.gyrbias.cwiseProduct(imuerror_std.gyrbias).asDiagonal();
    Cov_.block(BA_ID, BA_ID, 3, 3) = imuerror_std.accbias.cwiseProduct(imuerror_std.accbias).asDiagonal();
    Cov_.block(SG_ID, SG_ID, 3, 3) = imuerror_std.gyrscale.cwiseProduct(imuerror_std.gyrscale).asDiagonal();
    Cov_.block(SA_ID, SA_ID, 3, 3) = imuerror_std.accscale.cwiseProduct(imuerror_std.accscale).asDiagonal();
}

void RtkGIEngine::newImuProcess() {

    timestamp_ = imucur_.time;
    double updatetime = measurement_.isvalid ? measurement_.time : -1.0;

    int res = isToUpdate(imupre_.time, imucur_.time, updatetime);

    if (res == 0) {
        insPropagation(imupre_, imucur_);
    } else if (res == 1) {
        measurementUpdate(measurement_);
        stateFeedback();

        pvapre_ = pvacur_;
        insPropagation(imupre_, imucur_);
    } else if (res == 2) {
        insPropagation(imupre_, imucur_);
        measurementUpdate(measurement_);
        stateFeedback();
    } else {
        IMU midimu;
        imuInterpolate(imupre_, imucur_, updatetime, midimu);

        insPropagation(imupre_, midimu);
        measurementUpdate(measurement_);
        stateFeedback();

        pvapre_ = pvacur_;
        insPropagation(midimu, imucur_);
    }

    checkCov();

    pvapre_ = pvacur_;
    imupre_ = imucur_;
}

void RtkGIEngine::imuCompensate(IMU &imu) {

    imu.dtheta -= imuerror_.gyrbias * imu.dt;
    imu.dvel -= imuerror_.accbias * imu.dt;

    Eigen::Vector3d gyrscale = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    Eigen::Vector3d accscale = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
    imu.dvel = imu.dvel.cwiseProduct(accscale.cwiseInverse());
}

void RtkGIEngine::insPropagation(IMU &imupre, IMU &imucur) {

    imuCompensate(imucur);
    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

    Eigen::MatrixXd Phi, F, Qd, G;

    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(RANK, NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();

    Eigen::Vector2d rmrn;
    Eigen::Vector3d wie_n, wen_n;
    double gravity;
    rmrn = Earth::meridianPrimeVerticalRadius(pvapre_.pos[0]);
    gravity = Earth::gravity(pvapre_.pos);
    wie_n << WGS84_WIE * cos(pvapre_.pos[0]), 0, -WGS84_WIE * sin(pvapre_.pos[0]);
    wen_n << pvapre_.vel[1] / (rmrn[1] + pvapre_.pos[2]), -pvapre_.vel[0] / (rmrn[0] + pvapre_.pos[2]),
        -pvapre_.vel[1] * tan(pvapre_.pos[0]) / (rmrn[1] + pvapre_.pos[2]);

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh = rmrn[0] + pvapre_.pos[2];
    rnh = rmrn[1] + pvapre_.pos[2];
    accel = imucur.dvel / imucur.dt;
    omega = imucur.dtheta / imucur.dt;

    temp.setZero();
    temp(0, 0) = -pvapre_.vel[2] / rmh;
    temp(0, 2) = pvapre_.vel[0] / rmh;
    temp(1, 0) = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1) = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2) = pvapre_.vel[1] / rnh;
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    temp.setZero();
    temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
                 pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh -
                 pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
    temp(1, 0) = 2 * WGS84_WIE * (pvapre_.vel[0] * cos(pvapre_.pos[0]) - pvapre_.vel[2] * sin(pvapre_.pos[0])) /
                     rmh +
                 pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(1, 2) =
        (pvapre_.vel[1] * pvapre_.vel[2] + pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) / rnh / rnh;
    temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
    temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh - pow(pvapre_.vel[0], 2) / rmh / rmh +
                 2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;

    temp.setZero();
    temp(0, 0) = pvapre_.vel[2] / rmh;
    temp(0, 1) = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
    temp(0, 2) = pvapre_.vel[0] / rmh;
    temp(1, 0) = 2 * WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1) = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2) = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
    temp(2, 0) = -2 * pvapre_.vel[0] / rmh;
    temp(2, 1) = -2 * (WGS84_WIE * cos(pvapre_.pos(0)) + pvapre_.vel[1] / rnh);
    F.block(V_ID, V_ID, 3, 3) = temp;
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
    F.block(V_ID, BA_ID, 3, 3) = pvapre_.att.cbn;
    F.block(V_ID, SA_ID, 3, 3) = pvapre_.att.cbn * accel.asDiagonal();

    temp.setZero();
    temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
    temp(0, 2) = pvapre_.vel[1] / rnh / rnh;
    temp(1, 2) = -pvapre_.vel[0] / rmh / rmh;
    temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
                 pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh / rnh;
    F.block(PHI_ID, P_ID, 3, 3) = temp;

    temp.setZero();
    temp(0, 1) = 1 / rnh;
    temp(1, 0) = -1 / rmh;
    temp(2, 1) = -tan(pvapre_.pos[0]) / rnh;
    F.block(PHI_ID, V_ID, 3, 3) = temp;
    F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
    F.block(PHI_ID, BG_ID, 3, 3) = -pvapre_.att.cbn;
    F.block(PHI_ID, SG_ID, 3, 3) = -pvapre_.att.cbn * omega.asDiagonal();

    F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();

    G.block(V_ID, VRW_ID, 3, 3) = pvapre_.att.cbn;
    G.block(PHI_ID, ARW_ID, 3, 3) = pvapre_.att.cbn;
    G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

    Phi.setIdentity();
    Phi = Phi + F * imucur.dt;

    Qd = G * Qc_ * G.transpose() * imucur.dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;

    EKFPredict(Phi, Qd);
}

void RtkGIEngine::measurementUpdate(RtkPoseMeasurement &measurement) {

    Eigen::Vector3d antenna_pos;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv = Earth::DRi(pvacur_.pos);
    Dr = Earth::DR(pvacur_.pos);
    antenna_pos = pvacur_.pos + Dr_inv * pvacur_.att.cbn * options_.antlever;

    const bool use_yaw = measurement.has_yaw && measurement.yaw_std > 0.0;
    const int measurement_dim = use_yaw ? 4 : 3;

    Eigen::MatrixXd dz = Eigen::MatrixXd::Zero(measurement_dim, 1);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(measurement_dim, Cov_.rows());
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);

    dz.block(0, 0, 3, 1) = Dr * (antenna_pos - measurement.blh);
    H.block(0, P_ID, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever);
    R.block(0, 0, 3, 3) = measurement.std.cwiseProduct(measurement.std).asDiagonal();

    if (use_yaw) {
        dz(3, 0) = wrapAngle(measurement.yaw - pvacur_.att.euler[2]);
        H.block(3, PHI_ID, 1, 3) = yawJacobianFromPhiAngle(pvacur_.att.euler);
        R(3, 3) = measurement.yaw_std * measurement.yaw_std;
    }

    EKFUpdate(dz, H, R);
    measurement.isvalid = false;
}

int RtkGIEngine::isToUpdate(double imutime1, double imutime2, double updatetime) const {

    if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {
        return 1;
    } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {
        return 2;
    } else if (imutime1 < updatetime && updatetime < imutime2) {
        return 3;
    } else {
        return 0;
    }
}

void RtkGIEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_ = Phi * dx_;
}

void RtkGIEngine::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R) {

    assert(H.cols() == Cov_.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);

    auto temp = H * Cov_ * H.transpose() + R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    Eigen::MatrixXd I;
    I.resizeLike(Cov_);
    I.setIdentity();
    I = I - K * H;
    dx_ = dx_ + K * (dz - H * dx_);
    Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}

void RtkGIEngine::stateFeedback() {

    Eigen::Vector3d vectemp;

    Eigen::Vector3d delta_r = dx_.block(P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv = Earth::DRi(pvacur_.pos);
    pvacur_.pos -= Dr_inv * delta_r;

    vectemp = dx_.block(V_ID, 0, 3, 1);
    pvacur_.vel -= vectemp;

    vectemp = dx_.block(PHI_ID, 0, 3, 1);
    Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(vectemp);
    pvacur_.att.qbn = qpn * pvacur_.att.qbn;
    pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
    pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);

    vectemp = dx_.block(BG_ID, 0, 3, 1);
    imuerror_.gyrbias += vectemp;
    vectemp = dx_.block(BA_ID, 0, 3, 1);
    imuerror_.accbias += vectemp;

    vectemp = dx_.block(SG_ID, 0, 3, 1);
    imuerror_.gyrscale += vectemp;
    vectemp = dx_.block(SA_ID, 0, 3, 1);
    imuerror_.accscale += vectemp;

    dx_.setZero();
}

NavState RtkGIEngine::getNavState() {

    NavState state;
    state.pos = pvacur_.pos;
    state.vel = pvacur_.vel;
    state.euler = pvacur_.att.euler;
    state.imuerror = imuerror_;
    return state;
}
