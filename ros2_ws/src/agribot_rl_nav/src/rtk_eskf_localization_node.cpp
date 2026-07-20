#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <noah_msgs/msg/gnss_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "common/angle.h"
#include "common/earth.h"
#include "common/rotation.h"

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

double yawFromQuaternion(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {

    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

Eigen::Vector3d fluToFrd(const Eigen::Vector3d &flu) {
    return {flu.x(), -flu.y(), -flu.z()};
}

Eigen::Vector3d mapEnuToNed(
    const Eigen::Vector3d &map_enu,
    double extra_yaw_rad) {

    // Local map frame is ENU-like: x=east, y=north, z=up.
    // KF-GINS state is NED: x=north, y=east, z=down.
    Eigen::Vector3d ned_base(map_enu.y(), map_enu.x(), -map_enu.z());
    const double c = std::cos(extra_yaw_rad);
    const double s = std::sin(extra_yaw_rad);
    return Eigen::Vector3d(
        c * ned_base.x() - s * ned_base.y(),
        s * ned_base.x() + c * ned_base.y(),
        ned_base.z());
}

Eigen::Vector3d nedToMapEnu(
    const Eigen::Vector3d &ned,
    double extra_yaw_rad) {

    const double c = std::cos(extra_yaw_rad);
    const double s = std::sin(extra_yaw_rad);
    Eigen::Vector3d ned_base(
        c * ned.x() + s * ned.y(),
        -s * ned.x() + c * ned.y(),
        ned.z());
    return Eigen::Vector3d(ned_base.y(), ned_base.x(), -ned_base.z());
}

Eigen::Matrix3d mapEnuFromNed(double extra_yaw_rad) {

    const double c = std::cos(extra_yaw_rad);
    const double s = std::sin(extra_yaw_rad);
    Eigen::Matrix3d transform;
    transform << -s, c, 0.0,
                  c, s, 0.0,
                0.0, 0.0, -1.0;
    return transform;
}

double mapYawToNedHeading(double map_yaw, double extra_yaw_rad) {
    return wrapAngle(M_PI / 2.0 - map_yaw + extra_yaw_rad);
}

double nedHeadingToMapYaw(double ned_heading, double extra_yaw_rad) {
    return wrapAngle(M_PI / 2.0 + extra_yaw_rad - ned_heading);
}

Eigen::Matrix3d enuFluToNedFrd(const geometry_msgs::msg::Quaternion &qmsg) {

    Eigen::Quaterniond q_enu_flu(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
    Eigen::Matrix3d r_enu_flu = q_enu_flu.normalized().toRotationMatrix();

    Eigen::Matrix3d t_ned_enu;
    t_ned_enu << 0.0, 1.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 0.0, -1.0;

    Eigen::Matrix3d t_flu_frd = Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
    return t_ned_enu * r_enu_flu * t_flu_frd;
}

geometry_msgs::msg::Quaternion nedFrdToMapEnuFlu(
    const Eigen::Vector3d &euler,
    double extra_yaw_rad) {

    const Eigen::Matrix3d r_ned_frd = Rotation::euler2matrix(euler);
    const Eigen::Matrix3d t_frd_flu =
        Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0);
    const Eigen::Matrix3d r_map_flu =
        mapEnuFromNed(extra_yaw_rad) * r_ned_frd * t_frd_flu;
    const Eigen::Quaterniond quaternion(r_map_flu);
    const Eigen::Quaterniond normalized = quaternion.normalized();

    geometry_msgs::msg::Quaternion msg;
    msg.x = normalized.x();
    msg.y = normalized.y();
    msg.z = normalized.z();
    msg.w = normalized.w();
    return msg;
}

struct InitialPose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
};

struct PoseSample {
    double time = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_std = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_std = 0.0;
    bool has_global_blh = false;
    Eigen::Vector3d global_blh = Eigen::Vector3d::Zero();
};

}

class RtkEskfLocalizationNode : public rclcpp::Node {

public:
    RtkEskfLocalizationNode()
        : Node("rtk_eskf_localization") {

        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
        rtk_heading_topic_ =
            declare_parameter<std::string>("rtk_heading_topic", "/rtk/heading");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/rtk/odom");
        pose_message_type_ = declare_parameter<std::string>("pose_message_type", "odometry");
        output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/odometry/filtered_navsat");
        raw_pose_topic_ = declare_parameter<std::string>("raw_pose_topic", "/navsat_pose");
        initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
        publish_raw_pose_ = declare_parameter<bool>("publish_raw_pose", true);
        align_measurement_to_initial_pose_ =
            declare_parameter<bool>("align_measurement_to_initial_pose", true);
        imu_flu_frame_ = declare_parameter<bool>("imu_flu_frame", true);
        use_imu_orientation_for_initial_roll_pitch_ =
            declare_parameter<bool>("use_imu_orientation_for_initial_roll_pitch", false);
        use_pose_yaw_measurement_ =
            declare_parameter<bool>("use_pose_yaw_measurement", true);
        use_rtk_heading_ = declare_parameter<bool>("use_rtk_heading", false);
        require_rtk_heading_for_initialization_ = declare_parameter<bool>(
            "require_rtk_heading_for_initialization", true);
        rtk_heading_timeout_sec_ =
            declare_parameter<double>("rtk_heading_timeout_sec", 2.0);
        rtk_heading_std_rad_ =
            declare_parameter<double>("rtk_heading_std_deg", 1.0) * D2R;
        noah_heading_in_degrees_ = declare_parameter<bool>("noah_heading_in_degrees", true);
        auto_reference_from_first_noah_gnss_ =
            declare_parameter<bool>("auto_reference_from_first_noah_gnss", false);
        auto_reference_from_first_navsat_fix_ =
            declare_parameter<bool>("auto_reference_from_first_navsat_fix", false);

        default_initial_roll_deg_ = declare_parameter<double>("default_initial_roll_deg", 0.0);
        default_initial_pitch_deg_ = declare_parameter<double>("default_initial_pitch_deg", 0.0);
        reference_alt_param_m_ = declare_parameter<double>("reference_alt_m", 20.0);
        map_to_ned_yaw_rad_ = declare_parameter<double>("map_to_ned_yaw_deg", 0.0) * D2R;

        default_position_std_ = loadVector3("measurement_position_std_m", {0.03, 0.03, 0.05});
        default_yaw_std_rad_ = declare_parameter<double>("measurement_yaw_std_deg", 1.0) * D2R;

        pending_initial_pose_.position = Eigen::Vector3d(
            declare_parameter<double>("initial_pose_x", 0.0),
            declare_parameter<double>("initial_pose_y", 0.0),
            declare_parameter<double>("initial_pose_z", 0.0));
        pending_initial_pose_.yaw = declare_parameter<double>("initial_pose_yaw", 0.0);
        reference_map_position_ = pending_initial_pose_.position;

        if (auto_reference_from_first_noah_gnss_ || auto_reference_from_first_navsat_fix_) {
            reference_blh_initialized_ = false;
        } else {
            reference_blh_ = Eigen::Vector3d(
                declare_parameter<double>("reference_lat_deg", 30.5) * D2R,
                declare_parameter<double>("reference_lon_deg", 114.0) * D2R,
                reference_alt_param_m_);
            reference_blh_initialized_ = true;
        }

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 20);
        raw_pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(raw_pose_topic_, 10);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 200, std::bind(&RtkEskfLocalizationNode::handleImu, this, std::placeholders::_1));
        if (use_rtk_heading_) {
            rtk_heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
                rtk_heading_topic_,
                20,
                std::bind(
                    &RtkEskfLocalizationNode::handleRtkHeading,
                    this,
                    std::placeholders::_1));
        }
        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_,
            10,
            std::bind(&RtkEskfLocalizationNode::handleInitialPose, this, std::placeholders::_1));

        if (pose_message_type_ == "odometry") {
            odom_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                pose_topic_,
                20,
                std::bind(&RtkEskfLocalizationNode::handleOdomPose, this, std::placeholders::_1));
        } else if (pose_message_type_ == "pose_with_covariance") {
            pose_cov_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                pose_topic_,
                20,
                std::bind(&RtkEskfLocalizationNode::handlePoseWithCovariance, this, std::placeholders::_1));
        } else if (pose_message_type_ == "pose_stamped") {
            pose_stamped_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                pose_topic_,
                20,
                std::bind(&RtkEskfLocalizationNode::handlePoseStamped, this, std::placeholders::_1));
        } else if (pose_message_type_ == "noah_gnss") {
            noah_gnss_sub_ = create_subscription<noah_msgs::msg::GNSSValue>(
                pose_topic_,
                20,
                std::bind(&RtkEskfLocalizationNode::handleNoahGnss, this, std::placeholders::_1));
        } else if (pose_message_type_ == "navsat_fix") {
            navsat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                pose_topic_,
                20,
                std::bind(&RtkEskfLocalizationNode::handleNavSatFix, this, std::placeholders::_1));
        } else {
            throw std::runtime_error("Unsupported pose_message_type: " + pose_message_type_);
        }

        RCLCPP_INFO(
            get_logger(),
            "RTK ESKF ready: imu=%s pose=%s(%s) heading=%s(%s) odom_out=%s",
            imu_topic_.c_str(),
            pose_topic_.c_str(),
            pose_message_type_.c_str(),
            rtk_heading_topic_.c_str(),
            use_rtk_heading_ ? "enabled" : "disabled",
            output_odom_topic_.c_str());
    }

private:
    Eigen::Vector3d loadVector3(const std::string &name, const std::vector<double> &defaults) {
        auto values = declare_parameter<std::vector<double>>(name, defaults);
        if (values.size() != 3) {
            throw std::runtime_error("Parameter " + name + " must have exactly 3 elements");
        }
        return {values[0], values[1], values[2]};
    }

    void handleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

        if (alignment_initialized_) {
            return;
        }

        pending_initial_pose_.position = Eigen::Vector3d(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        pending_initial_pose_.yaw = yawFromQuaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        reference_map_position_ = pending_initial_pose_.position;
    }

    void handleOdomPose(const nav_msgs::msg::Odometry::SharedPtr msg) {
        PoseSample sample;
        sample.time = rclcpp::Time(msg->header.stamp).seconds();
        sample.position = Eigen::Vector3d(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        sample.position_std = extractPositionStd(msg->pose.covariance);
        sample.yaw = yawFromQuaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        sample.yaw_std = extractYawStd(msg->pose.covariance);
        processPoseSample(sample);
    }

    void handlePoseWithCovariance(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        PoseSample sample;
        sample.time = rclcpp::Time(msg->header.stamp).seconds();
        sample.position = Eigen::Vector3d(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        sample.position_std = extractPositionStd(msg->pose.covariance);
        sample.yaw = yawFromQuaternion(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        sample.yaw_std = extractYawStd(msg->pose.covariance);
        processPoseSample(sample);
    }

    void handlePoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        PoseSample sample;
        sample.time = rclcpp::Time(msg->header.stamp).seconds();
        sample.position =
            Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        sample.position_std = default_position_std_;
        sample.yaw = yawFromQuaternion(
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        sample.yaw_std = default_yaw_std_rad_;
        processPoseSample(sample);
    }

    void handleNoahGnss(const noah_msgs::msg::GNSSValue::SharedPtr msg) {
        PoseSample sample;
        sample.time = rclcpp::Time(msg->header.stamp).seconds();
        sample.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        sample.position_std = default_position_std_;
        sample.yaw = noah_heading_in_degrees_ ? msg->heading * D2R : msg->heading;
        sample.yaw_std = default_yaw_std_rad_;
        if (std::isfinite(msg->latitude) && std::isfinite(msg->longitude)) {
            sample.has_global_blh = true;
            sample.global_blh = Eigen::Vector3d(
                msg->latitude * D2R,
                msg->longitude * D2R,
                reference_alt_param_m_);
        }
        processPoseSample(sample);
    }

    void handleNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
            !std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
            !std::isfinite(msg->altitude)) {
            return;
        }

        const double fix_time = rclcpp::Time(msg->header.stamp).seconds();
        const bool have_fresh_rtk_heading =
            use_rtk_heading_ && latest_rtk_heading_yaw_.has_value() &&
            latest_rtk_heading_time_.has_value() &&
            std::abs(fix_time - *latest_rtk_heading_time_) <= rtk_heading_timeout_sec_;
        if (use_rtk_heading_ && require_rtk_heading_for_initialization_ &&
            !engine_ && !have_fresh_rtk_heading) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for RTK heading before initializing from NavSatFix");
            return;
        }
        if (!have_fresh_rtk_heading && !latest_imu_orientation_.has_value()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for RTK or IMU heading before accepting NavSatFix");
            return;
        }

        const Eigen::Vector3d global_blh(
            msg->latitude * D2R, msg->longitude * D2R, msg->altitude);
        if (!reference_blh_initialized_) {
            if (!auto_reference_from_first_navsat_fix_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "NavSat reference is not initialized");
                return;
            }
            reference_blh_ = global_blh;
            reference_blh_initialized_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Reference BLH anchored from NavSatFix: lat=%.8f lon=%.8f alt=%.3f",
                msg->latitude, msg->longitude, msg->altitude);
        }

        PoseSample sample;
        sample.time = fix_time;
        const Eigen::Vector3d local_ned = Earth::global2local(reference_blh_, global_blh);
        sample.position =
            reference_map_position_ + nedToMapEnu(local_ned, map_to_ned_yaw_rad_);
        sample.position_std = default_position_std_;
        if (msg->position_covariance[0] > 0.0) {
            sample.position_std.x() = std::sqrt(msg->position_covariance[0]);
        }
        if (msg->position_covariance[4] > 0.0) {
            sample.position_std.y() = std::sqrt(msg->position_covariance[4]);
        }
        if (msg->position_covariance[8] > 0.0) {
            sample.position_std.z() = std::sqrt(msg->position_covariance[8]);
        }
        if (have_fresh_rtk_heading) {
            sample.yaw = *latest_rtk_heading_yaw_;
            sample.yaw_std = rtk_heading_std_rad_;
        } else {
            const auto &orientation = *latest_imu_orientation_;
            sample.yaw = yawFromQuaternion(
                orientation.x, orientation.y, orientation.z, orientation.w);
            sample.yaw_std = default_yaw_std_rad_;
            if (use_rtk_heading_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "RTK heading is unavailable or stale; using IMU yaw");
            }
        }
        sample.has_global_blh = true;
        sample.global_blh = global_blh;
        processPoseSample(sample);
    }

    void handleRtkHeading(
        const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {

        const auto &orientation = msg->quaternion;
        const double norm_squared =
            orientation.x * orientation.x + orientation.y * orientation.y +
            orientation.z * orientation.z + orientation.w * orientation.w;
        if (!std::isfinite(norm_squared) || norm_squared < 1e-12) {
            return;
        }
        const double inverse_norm = 1.0 / std::sqrt(norm_squared);
        latest_rtk_heading_yaw_ = yawFromQuaternion(
            orientation.x * inverse_norm,
            orientation.y * inverse_norm,
            orientation.z * inverse_norm,
            orientation.w * inverse_norm);
        latest_rtk_heading_time_ = rclcpp::Time(msg->header.stamp).seconds();
    }

    Eigen::Vector3d extractPositionStd(const std::array<double, 36> &covariance) const {

        Eigen::Vector3d std = default_position_std_;
        if (covariance[0] > 0.0) {
            std.x() = std::sqrt(covariance[0]);
        }
        if (covariance[7] > 0.0) {
            std.y() = std::sqrt(covariance[7]);
        }
        if (covariance[14] > 0.0) {
            std.z() = std::sqrt(covariance[14]);
        }
        return std;
    }

    double extractYawStd(const std::array<double, 36> &covariance) const {
        if (covariance[35] > 0.0) {
            return std::sqrt(covariance[35]);
        }
        return default_yaw_std_rad_;
    }

    void processPoseSample(PoseSample sample) {

        if (!alignment_initialized_) {
            initializeAlignment(sample);
        }

        PoseSample aligned = sample;
        aligned.position = alignPosition(sample.position);
        aligned.yaw = wrapAngle(sample.yaw + alignment_yaw_);

        if (!reference_blh_initialized_ && sample.has_global_blh &&
            (auto_reference_from_first_noah_gnss_ || auto_reference_from_first_navsat_fix_)) {
            reference_blh_ = sample.global_blh;
            reference_blh_initialized_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Reference BLH anchored from first GNSS sample: lat=%.8f lon=%.8f alt=%.3f",
                reference_blh_.x() * R2D,
                reference_blh_.y() * R2D,
                reference_blh_.z());
        }

        if (publish_raw_pose_) {
            publishRawPose(aligned);
        }

        const bool had_engine = static_cast<bool>(engine_);
        latest_measurement_for_init_ = aligned;
        tryInitializeEngine();

        if (!engine_) {
            if (!reference_blh_initialized_) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Waiting for reference BLH before starting KF-GINS ESKF");
            }
            return;
        }
        if (!had_engine && engine_) {
            return;
        }

        RtkPoseMeasurement measurement = toEngineMeasurement(aligned);
        if (!pose_queue_.empty() && measurement.time <= pose_queue_.back().time) {
            return;
        }
        pose_queue_.push_back(measurement);
    }

    void initializeAlignment(const PoseSample &sample) {

        if (!align_measurement_to_initial_pose_) {
            alignment_initialized_ = true;
            alignment_yaw_ = 0.0;
            alignment_translation_ = Eigen::Vector3d::Zero();
            return;
        }

        alignment_yaw_ = wrapAngle(pending_initial_pose_.yaw - sample.yaw);
        const double c = std::cos(alignment_yaw_);
        const double s = std::sin(alignment_yaw_);

        alignment_translation_.x() =
            pending_initial_pose_.position.x() - (c * sample.position.x() - s * sample.position.y());
        alignment_translation_.y() =
            pending_initial_pose_.position.y() - (s * sample.position.x() + c * sample.position.y());
        alignment_translation_.z() = pending_initial_pose_.position.z() - sample.position.z();
        alignment_initialized_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Aligned RTK frame to map using initial pose (x=%.3f y=%.3f yaw=%.3f)",
            pending_initial_pose_.position.x(),
            pending_initial_pose_.position.y(),
            pending_initial_pose_.yaw);
    }

    Eigen::Vector3d alignPosition(const Eigen::Vector3d &raw_position) const {

        if (!align_measurement_to_initial_pose_) {
            return raw_position;
        }

        const double c = std::cos(alignment_yaw_);
        const double s = std::sin(alignment_yaw_);
        return Eigen::Vector3d(
            alignment_translation_.x() + c * raw_position.x() - s * raw_position.y(),
            alignment_translation_.y() + s * raw_position.x() + c * raw_position.y(),
            alignment_translation_.z() + raw_position.z());
    }

    void publishRawPose(const PoseSample &sample) {

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sample.time * 1e9));
        msg.header.frame_id = map_frame_;
        msg.pose.pose.position.x = sample.position.x();
        msg.pose.pose.position.y = sample.position.y();
        msg.pose.pose.position.z = sample.position.z();
        msg.pose.pose.orientation = quaternionFromYaw(sample.yaw);
        msg.pose.covariance[0] = sample.position_std.x() * sample.position_std.x();
        msg.pose.covariance[7] = sample.position_std.y() * sample.position_std.y();
        msg.pose.covariance[14] = sample.position_std.z() * sample.position_std.z();
        msg.pose.covariance[21] = 1e6;
        msg.pose.covariance[28] = 1e6;
        msg.pose.covariance[35] = sample.yaw_std * sample.yaw_std;
        raw_pose_pub_->publish(msg);
    }

    void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg) {

        latest_imu_orientation_ = msg->orientation;

        const double time = rclcpp::Time(msg->header.stamp).seconds();
        if (!have_prev_raw_imu_) {
            prev_raw_imu_ = msg;
            have_prev_raw_imu_ = true;
            return;
        }

        const double prev_time = rclcpp::Time(prev_raw_imu_->header.stamp).seconds();
        const double dt = time - prev_time;
        if (dt <= 0.0) {
            prev_raw_imu_ = msg;
            return;
        }

        IMU imu_sample;
        imu_sample.time = time;
        imu_sample.dt = dt;
        imu_sample.odovel = 0.0;

        Eigen::Vector3d gyro(
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Vector3d accel(
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        if (imu_flu_frame_) {
            gyro = fluToFrd(gyro);
            accel = fluToFrd(accel);
        }
        imu_sample.dtheta = gyro * dt;
        imu_sample.dvel = accel * dt;

        prev_raw_imu_ = msg;
        latest_imu_sample_ = imu_sample;
        have_converted_imu_ = true;

        tryInitializeEngine();
        if (!engine_) {
            return;
        }

        if (!imu_primed_) {
            engine_->addImuData(imu_sample, true);
            imu_primed_ = true;
            return;
        }

        while (!pose_queue_.empty() && pose_queue_.front().time < imu_sample.time) {
            engine_->addPoseMeasurement(pose_queue_.front());
            pose_queue_.pop_front();
        }

        engine_->addImuData(imu_sample);
        engine_->newImuProcess();
        publishFilteredState(imu_sample.time);
    }

    void tryInitializeEngine() {

        if (engine_ || !have_converted_imu_ || !latest_measurement_for_init_.has_value() ||
            !reference_blh_initialized_) {
            return;
        }

        GINSOptions options = buildOptions(*latest_measurement_for_init_);
        engine_ = std::make_unique<RtkGIEngine>(options);

        engine_->addImuData(*latest_imu_sample_, true);
        imu_primed_ = true;
        pose_queue_.clear();

        RCLCPP_INFO(
            get_logger(),
            "KF-GINS ESKF initialized at map pose (%.3f, %.3f, %.3f, %.3f)",
            latest_measurement_for_init_->position.x(),
            latest_measurement_for_init_->position.y(),
            latest_measurement_for_init_->position.z(),
            latest_measurement_for_init_->yaw);
    }

    GINSOptions buildOptions(const PoseSample &initial_measurement) {

        GINSOptions options;

        options.initstate.pos = mapPoseToBlh(initial_measurement.position);
        options.initstate.vel = loadVector3("init_velocity_mps", {0.0, 0.0, 0.0});

        Eigen::Vector3d initial_euler(default_initial_roll_deg_ * D2R, default_initial_pitch_deg_ * D2R, 0.0);
        if (use_imu_orientation_for_initial_roll_pitch_ && latest_imu_orientation_.has_value()) {
            initial_euler = Rotation::matrix2euler(enuFluToNedFrd(*latest_imu_orientation_));
        }
        initial_euler.z() = mapYawToNedHeading(initial_measurement.yaw, map_to_ned_yaw_rad_);
        options.initstate.euler = initial_euler;

        options.initstate.imuerror.gyrbias = loadVector3("init_gyrbias_degph", {0.0, 0.0, 0.0}) * D2R / 3600.0;
        options.initstate.imuerror.accbias = loadVector3("init_accbias_mgal", {0.0, 0.0, 0.0}) * 1e-5;
        options.initstate.imuerror.gyrscale = loadVector3("init_gyrscale_ppm", {0.0, 0.0, 0.0}) * 1e-6;
        options.initstate.imuerror.accscale = loadVector3("init_accscale_ppm", {0.0, 0.0, 0.0}) * 1e-6;

        options.initstate_std.pos = loadVector3("init_pos_std_m", {0.1, 0.1, 0.2});
        options.initstate_std.vel = loadVector3("init_vel_std_mps", {0.05, 0.05, 0.05});
        options.initstate_std.euler = loadVector3("init_att_std_deg", {0.5, 0.5, 1.0}) * D2R;
        options.initstate_std.imuerror.gyrbias =
            loadVector3("init_bg_std_degph", {50.0, 50.0, 50.0}) * D2R / 3600.0;
        options.initstate_std.imuerror.accbias =
            loadVector3("init_ba_std_mgal", {250.0, 250.0, 250.0}) * 1e-5;
        options.initstate_std.imuerror.gyrscale =
            loadVector3("init_sg_std_ppm", {1000.0, 1000.0, 1000.0}) * 1e-6;
        options.initstate_std.imuerror.accscale =
            loadVector3("init_sa_std_ppm", {1000.0, 1000.0, 1000.0}) * 1e-6;

        options.imunoise.gyr_arw = loadVector3("imunoise_arw_deg_sqrt_hr", {0.24, 0.24, 0.24}) * D2R / 60.0;
        options.imunoise.acc_vrw = loadVector3("imunoise_vrw_mps_sqrt_hr", {0.24, 0.24, 0.24}) / 60.0;
        options.imunoise.gyrbias_std =
            loadVector3("imunoise_gbstd_degph", {50.0, 50.0, 50.0}) * D2R / 3600.0;
        options.imunoise.accbias_std = loadVector3("imunoise_abstd_mgal", {250.0, 250.0, 250.0}) * 1e-5;
        options.imunoise.gyrscale_std = loadVector3("imunoise_gsstd_ppm", {1000.0, 1000.0, 1000.0}) * 1e-6;
        options.imunoise.accscale_std = loadVector3("imunoise_asstd_ppm", {1000.0, 1000.0, 1000.0}) * 1e-6;
        options.imunoise.corr_time = declare_parameter<double>("imunoise_corrtime_hr", 1.0) * 3600.0;

        options.antlever = loadVector3("antlever_m", {0.136, -0.301, -0.184});

        return options;
    }

    RtkPoseMeasurement toEngineMeasurement(const PoseSample &sample) const {

        RtkPoseMeasurement measurement;
        measurement.time = sample.time;
        measurement.blh = mapPoseToBlh(sample.position);
        measurement.std = sample.position_std;
        measurement.yaw = mapYawToNedHeading(sample.yaw, map_to_ned_yaw_rad_);
        measurement.yaw_std = sample.yaw_std;
        measurement.has_yaw = use_pose_yaw_measurement_;
        measurement.isvalid = true;
        return measurement;
    }

    Eigen::Vector3d mapPoseToBlh(const Eigen::Vector3d &map_position) const {

        Eigen::Vector3d delta = map_position - reference_map_position_;
        Eigen::Vector3d local_ned = mapEnuToNed(delta, map_to_ned_yaw_rad_);
        return Earth::local2global(reference_blh_, local_ned);
    }

    Eigen::Vector3d blhToMapPose(const Eigen::Vector3d &blh) const {

        Eigen::Vector3d local_ned = Earth::global2local(reference_blh_, blh);
        Eigen::Vector3d delta = nedToMapEnu(local_ned, map_to_ned_yaw_rad_);
        return reference_map_position_ + delta;
    }

    void publishFilteredState(double time) {

        NavState state = engine_->getNavState();
        Eigen::MatrixXd covariance = engine_->getCovariance();

        Eigen::Vector3d map_position = blhToMapPose(state.pos);

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
        msg.header.frame_id = map_frame_;
        msg.child_frame_id = base_frame_;
        msg.pose.pose.position.x = map_position.x();
        msg.pose.pose.position.y = map_position.y();
        msg.pose.pose.position.z = map_position.z();
        msg.pose.pose.orientation = nedFrdToMapEnuFlu(state.euler, map_to_ned_yaw_rad_);

        Eigen::Matrix3d p_pos_ned = covariance.block<3, 3>(0, 0);
        const Eigen::Matrix3d r_map_from_ned = mapEnuFromNed(map_to_ned_yaw_rad_);
        Eigen::Matrix3d p_pos_map = r_map_from_ned * p_pos_ned * r_map_from_ned.transpose();

        msg.pose.covariance[0] = p_pos_map(0, 0);
        msg.pose.covariance[1] = p_pos_map(0, 1);
        msg.pose.covariance[6] = p_pos_map(1, 0);
        msg.pose.covariance[7] = p_pos_map(1, 1);
        msg.pose.covariance[14] = p_pos_map(2, 2);
        const Eigen::Matrix3d p_att_ned = covariance.block<3, 3>(6, 6);
        const Eigen::Matrix3d p_att_map =
            r_map_from_ned * p_att_ned * r_map_from_ned.transpose();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                msg.pose.covariance[(row + 3) * 6 + col + 3] = p_att_map(row, col);
            }
        }

        Eigen::Vector3d map_velocity = nedToMapEnu(state.vel, map_to_ned_yaw_rad_);
        msg.twist.twist.linear.x = map_velocity.x();
        msg.twist.twist.linear.y = map_velocity.y();
        msg.twist.twist.linear.z = map_velocity.z();
        odom_pub_->publish(msg);
    }

private:
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_topic_;
    std::string rtk_heading_topic_;
    std::string pose_topic_;
    std::string pose_message_type_;
    std::string output_odom_topic_;
    std::string raw_pose_topic_;
    std::string initial_pose_topic_;

    bool publish_raw_pose_ = true;
    bool align_measurement_to_initial_pose_ = true;
    bool imu_flu_frame_ = true;
    bool use_imu_orientation_for_initial_roll_pitch_ = false;
    bool use_pose_yaw_measurement_ = true;
    bool use_rtk_heading_ = false;
    bool require_rtk_heading_for_initialization_ = true;
    bool noah_heading_in_degrees_ = true;
    bool auto_reference_from_first_noah_gnss_ = false;
    bool auto_reference_from_first_navsat_fix_ = false;

    double default_initial_roll_deg_ = 0.0;
    double default_initial_pitch_deg_ = 0.0;
    double default_yaw_std_rad_ = 0.0;
    double rtk_heading_timeout_sec_ = 0.0;
    double rtk_heading_std_rad_ = 0.0;
    double reference_alt_param_m_ = 0.0;
    double map_to_ned_yaw_rad_ = 0.0;

    Eigen::Vector3d default_position_std_ = Eigen::Vector3d::Zero();
    InitialPose pending_initial_pose_;
    Eigen::Vector3d reference_map_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_blh_ = Eigen::Vector3d::Zero();
    bool reference_blh_initialized_ = false;

    bool alignment_initialized_ = false;
    double alignment_yaw_ = 0.0;
    Eigen::Vector3d alignment_translation_ = Eigen::Vector3d::Zero();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr raw_pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr rtk_heading_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
    rclcpp::Subscription<noah_msgs::msg::GNSSValue>::SharedPtr noah_gnss_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_fix_sub_;

    sensor_msgs::msg::Imu::SharedPtr prev_raw_imu_;
    bool have_prev_raw_imu_ = false;
    bool have_converted_imu_ = false;
    std::optional<geometry_msgs::msg::Quaternion> latest_imu_orientation_;
    std::optional<double> latest_rtk_heading_yaw_;
    std::optional<double> latest_rtk_heading_time_;
    std::optional<IMU> latest_imu_sample_;
    std::optional<PoseSample> latest_measurement_for_init_;

    std::unique_ptr<RtkGIEngine> engine_;
    bool imu_primed_ = false;
    std::deque<RtkPoseMeasurement> pose_queue_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtkEskfLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
