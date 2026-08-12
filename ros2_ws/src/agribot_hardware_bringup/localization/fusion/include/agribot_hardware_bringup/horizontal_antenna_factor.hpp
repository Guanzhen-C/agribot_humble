#ifndef AGRIBOT_HARDWARE_BRINGUP__HORIZONTAL_ANTENNA_FACTOR_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__HORIZONTAL_ANTENNA_FACTOR_HPP_

#include <cmath>
#include <iostream>
#include <string>

#include <boost/make_shared.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace agribot_hardware_bringup::fusion
{

class HorizontalAntennaFactor final :
  public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  using This = HorizontalAntennaFactor;

  HorizontalAntennaFactor()
  : measured_antenna_map_(0.0, 0.0, 0.0), base_to_antenna_(0.0, 0.0, 0.0)
  {
  }

  HorizontalAntennaFactor(
    gtsam::Key pose_key,
    const gtsam::Point3 & measured_antenna_map,
    const gtsam::Point3 & base_to_antenna,
    const gtsam::SharedNoiseModel & noise_model)
  : Base(noise_model, pose_key),
    measured_antenna_map_(measured_antenna_map),
    base_to_antenna_(base_to_antenna)
  {
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      boost::make_shared<This>(*this));
  }

  void print(
    const std::string & label = "",
    const gtsam::KeyFormatter & key_formatter = gtsam::DefaultKeyFormatter) const override
  {
    Base::print(label, key_formatter);
    std::cout << "  measured antenna XY: " << measured_antenna_map_.x() << ", "
              << measured_antenna_map_.y() << "\n"
              << "  base-to-antenna: " << base_to_antenna_.transpose() << std::endl;
  }

  bool equals(const gtsam::NonlinearFactor & expected, double tolerance = 1.0e-9) const override
  {
    const auto * other = dynamic_cast<const This *>(&expected);
    return other != nullptr && Base::equals(expected, tolerance) &&
           (measured_antenna_map_ - other->measured_antenna_map_).norm() <= tolerance &&
           (base_to_antenna_ - other->base_to_antenna_).norm() <= tolerance;
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & map_from_base,
    boost::optional<gtsam::Matrix &> jacobian = boost::none) const override
  {
    gtsam::Point3 predicted_antenna;
    if (jacobian) {
      gtsam::Matrix36 predicted_jacobian;
      predicted_antenna = map_from_base.transformFrom(
        base_to_antenna_, &predicted_jacobian);
      *jacobian = predicted_jacobian.topRows<2>();
    } else {
      predicted_antenna = map_from_base.transformFrom(base_to_antenna_);
    }
    return gtsam::Vector2(
      predicted_antenna.x() - measured_antenna_map_.x(),
      predicted_antenna.y() - measured_antenna_map_.y());
  }

  const gtsam::Point3 & measuredAntennaMap() const
  {
    return measured_antenna_map_;
  }

  const gtsam::Point3 & baseToAntenna() const
  {
    return base_to_antenna_;
  }

private:
  gtsam::Point3 measured_antenna_map_;
  gtsam::Point3 base_to_antenna_;
};

}  // namespace agribot_hardware_bringup::fusion

#endif  // AGRIBOT_HARDWARE_BRINGUP__HORIZONTAL_ANTENNA_FACTOR_HPP_
