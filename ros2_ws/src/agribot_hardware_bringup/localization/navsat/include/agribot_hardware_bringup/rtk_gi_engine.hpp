#ifndef AGRIBOT_HARDWARE_BRINGUP__RTK_GI_ENGINE_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__RTK_GI_ENGINE_HPP_

#include <Eigen/Dense>

#include "common/types.h"
#include "kf-gins/kf_gins_types.h"

struct RtkPoseMeasurement {
    double time = 0.0;
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();
    Eigen::Vector3d std = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_std = 0.0;
    bool has_yaw = false;
    bool isvalid = false;
};

class RtkGIEngine {

public:
    explicit RtkGIEngine(GINSOptions &options);

    ~RtkGIEngine() = default;

    void addImuData(const IMU &imu, bool compensate = false) {

        imupre_ = imucur_;
        imucur_ = imu;

        if (compensate) {
            imuCompensate(imucur_);
        }
    }

    void addPoseMeasurement(const RtkPoseMeasurement &measurement) {

        measurement_ = measurement;
        measurement_.isvalid = true;
    }

    void newImuProcess();

    static void imuInterpolate(const IMU &imu1, IMU &imu2, const double timestamp, IMU &midimu) {

        if (imu1.time > timestamp || imu2.time < timestamp) {
            return;
        }

        double lamda = (timestamp - imu1.time) / (imu2.time - imu1.time);

        midimu.time   = timestamp;
        midimu.dtheta = imu2.dtheta * lamda;
        midimu.dvel   = imu2.dvel * lamda;
        midimu.dt     = timestamp - imu1.time;

        imu2.dtheta = imu2.dtheta - midimu.dtheta;
        imu2.dvel   = imu2.dvel - midimu.dvel;
        imu2.dt     = imu2.dt - midimu.dt;
    }

    double timestamp() const {
        return timestamp_;
    }

    NavState getNavState();

    Eigen::MatrixXd getCovariance() {
        return Cov_;
    }

private:
    void initialize(const NavState &initstate, const NavState &initstate_std);

    void imuCompensate(IMU &imu);

    int isToUpdate(double imutime1, double imutime2, double updatetime) const;

    void insPropagation(IMU &imupre, IMU &imucur);

    void measurementUpdate(RtkPoseMeasurement &measurement);

    void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);

    void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);

    void stateFeedback();

    void checkCov() {

        for (int i = 0; i < RANK; i++) {
            if (Cov_(i, i) < 0) {
                std::cout << "Covariance is negative at " << std::setprecision(10) << timestamp_ << " !"
                          << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
    }

private:
    GINSOptions options_;

    double timestamp_;

    const double TIME_ALIGN_ERR = 0.001;

    IMU imupre_;
    IMU imucur_;
    RtkPoseMeasurement measurement_;

    PVA pvacur_;
    PVA pvapre_;
    ImuError imuerror_;

    Eigen::MatrixXd Cov_;
    Eigen::MatrixXd Qc_;
    Eigen::MatrixXd dx_;

    const int RANK = 21;
    const int NOISERANK = 18;

    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, BG_ID = 9, BA_ID = 12, SG_ID = 15, SA_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, BGSTD_ID = 6, BASTD_ID = 9, SGSTD_ID = 12, SASTD_ID = 15 };
};

#endif  // AGRIBOT_HARDWARE_BRINGUP__RTK_GI_ENGINE_HPP_
