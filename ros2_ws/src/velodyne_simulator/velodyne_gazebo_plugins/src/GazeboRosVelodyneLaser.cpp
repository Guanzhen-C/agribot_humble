/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2021, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <velodyne_gazebo_plugins/GazeboRosVelodyneLaser.hpp>
#include <gazebo_ros/utils.hpp>

#include <algorithm>

#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
static_assert(GAZEBO_MAJOR_VERSION >= 11, "Gazebo version is too old");


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVelodyneLaser::GazeboRosVelodyneLaser() : min_range_(0), max_range_(0), gaussian_noise_(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "Loading GazeboRosVelodyneLaser\n";

  // Create node handle
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get the parent ray sensor
  parent_ray_sensor_ = _parent;
  ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
  gpu_ray_sensor_ = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
  if (!ray_sensor_ && !gpu_ray_sensor_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Parent is not a ray or gpu_ray sensor. Exiting.");
    return;
  }

  // Sets the frame name to either the supplied name, or the name of the sensor
  std::string tf_prefix = _sdf->Get<std::string>("tf_prefix", std::string("")).first;
  frame_name_ = tf_resolve(tf_prefix, gazebo_ros::SensorFrameID(*_parent, *_sdf));

  if (!_sdf->HasElement("organize_cloud")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <organize_cloud>, defaults to false");
    organize_cloud_ = false;
  } else {
    organize_cloud_ = _sdf->GetElement("organize_cloud")->Get<bool>();
  }

  if (!_sdf->HasElement("min_range")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  } else {
    min_range_ = _sdf->GetElement("min_range")->Get<double>();
  }

  if (!_sdf->HasElement("max_range")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  } else {
    max_range_ = _sdf->GetElement("max_range")->Get<double>();
  }

  min_intensity_ = std::numeric_limits<double>::lowest();
  if (!_sdf->HasElement("min_intensity")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <min_intensity>, defaults to no clipping");
  } else {
    min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
  }

  if (!_sdf->HasElement("gaussian_noise")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <gaussian_noise>, defaults to 0.0");
    gaussian_noise_ = 0;
  } else {
    gaussian_noise_ = _sdf->GetElement("gaussian_noise")->Get<double>();
  }

  // Advertise publisher
  pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out", 10);

  // ROS 2 publishers do not expose connection callbacks. Poll the subscription
  // count so the Gazebo sensor remains inactive when no consumer is present.
  using namespace std::chrono_literals;
  timer_ = ros_node_->create_wall_timer(0.5s, std::bind(&GazeboRosVelodyneLaser::ConnectCb, this));

  RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin ready");
  gzdbg << "GazeboRosVelodyneLaser LOADED\n";
}

////////////////////////////////////////////////////////////////////////////////
// Subscribe on-demand
void GazeboRosVelodyneLaser::ConnectCb()
{
  std::lock_guard<std::mutex> lock(lock_);
  if (pub_->get_subscription_count()) {
    if (!sensor_update_event_) {
      sensor_update_event_ = parent_ray_sensor_->ConnectUpdated(
        std::bind(&GazeboRosVelodyneLaser::OnUpdate, this));
    }
    parent_ray_sensor_->SetActive(true);
  } else {
    sensor_update_event_.reset();
    parent_ray_sensor_->SetActive(false);
  }
}

template<typename SensorT>
void GazeboRosVelodyneLaser::PublishScan(const std::shared_ptr<SensorT> & sensor)
{
  const ignition::math::Angle maxAngle = sensor->AngleMax();
  const ignition::math::Angle minAngle = sensor->AngleMin();

  const double maxRange = sensor->RangeMax();
  const double minRange = sensor->RangeMin();

  const int rangeCount = sensor->RangeCount();

  const int verticalRayCount = sensor->VerticalRayCount();
  const int verticalRangeCount = sensor->VerticalRangeCount();

  const ignition::math::Angle verticalMaxAngle = sensor->VerticalAngleMax();
  const ignition::math::Angle verticalMinAngle = sensor->VerticalAngleMin();

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);
  const double MIN_INTENSITY = min_intensity_;

  // Populate message fields
  const uint32_t POINT_STEP = 22;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  const auto measurement_time = sensor->LastMeasurementTime();
  msg.header.stamp.sec = measurement_time.sec;
  msg.header.stamp.nanosec = measurement_time.nsec;
  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "time";
  msg.fields[5].offset = 18;
  msg.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[5].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  int i, j;
  uint8_t *ptr = msg.data.data();
  for (i = 0; i < rangeCount; i++) {
    for (j = 0; j < verticalRangeCount; j++) {

      // Range
      const auto index = i + j * rangeCount;
      double r = sensor->Range(index);
      // Intensity
      double intensity = sensor->Retro(index);
      // Ignore points that lay outside range bands or optionally, beneath a
      // minimum intensity level.
      if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) ) {
        if (!organize_cloud_) {
          continue;
        }
      }

      // Noise
      if (gaussian_noise_ != 0.0) {
        r += gaussianKernel(0,gaussian_noise_);
      }

      // Get angles of ray to get xyz for point
      double yAngle;
      double pAngle;

      if (rangeCount > 1) {
        yAngle = i * yDiff / (rangeCount -1) + minAngle.Radian();
      } else {
        yAngle = minAngle.Radian();
      }

      if (verticalRayCount > 1) {
        pAngle = j * pDiff / (verticalRangeCount -1) + verticalMinAngle.Radian();
      } else {
        pAngle = verticalMinAngle.Radian();
      }

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle); // x
        *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle); // y
        *((float*)(ptr + 8)) = r * sin(pAngle); // z
        *((float*)(ptr + 12)) = intensity; // intensity
        *((uint16_t*)(ptr + 16)) = j; // ring
        *((float*)(ptr + 18)) = 0.0; // time
        ptr += POINT_STEP;
      } else if (organize_cloud_) {
        *((float*)(ptr + 0)) = nanf(""); // x
        *((float*)(ptr + 4)) = nanf(""); // y
        *((float*)(ptr + 8)) = nanf(""); // x
        *((float*)(ptr + 12)) = nanf(""); // intensity
        *((uint16_t*)(ptr + 16)) = j; // ring
        *((float*)(ptr + 18)) = 0.0; // time
        ptr += POINT_STEP;
      }
    }
  }

  // Populate message with number of valid points
  msg.data.resize(ptr - msg.data.data()); // Shrink to actual size
  msg.point_step = POINT_STEP;
  msg.is_bigendian = false;
  if (organize_cloud_) {
    msg.width = verticalRangeCount;
    msg.height = msg.data.size() / POINT_STEP / msg.width;
    msg.row_step = POINT_STEP * msg.width;
    msg.is_dense = false;
  } else {
    msg.width = msg.data.size() / POINT_STEP;
    msg.height = 1;
    msg.row_step = msg.data.size();
    msg.is_dense = true;
  }

  // Publish output
  pub_->publish(msg);
}

void GazeboRosVelodyneLaser::OnUpdate()
{
  if (ray_sensor_) {
    PublishScan(ray_sensor_);
  } else {
    PublishScan(gpu_ray_sensor_);
  }
}

} // namespace gazebo
