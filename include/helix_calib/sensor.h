#pragma once

#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "fmt/format.h"
#include "helix_calib/odom.h"
#include "helix_calib/surfel_map.h"
#include "helix_calib/utils.h"
#include "kontiki/measurements/accelerometer_measurement.h"
#include "kontiki/measurements/gyroscope_measurement.h"
#include "kontiki/measurements/lidar_surfel_point.h"
#include "kontiki/sensors/constant_bias_imu.h"
#include "kontiki/sensors/vlp16_lidar.h"
#include "kontiki/trajectories/split_trajectory.h"
#include "opencv2/calib3d.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/make_shared.h"
#include "pcl/pcl_macros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/time.h"
#include "rosbag/message_instance.h"

namespace helix
{
class LiDAR
{
public:
  using Sensor = kontiki::sensors::VLP16LiDAR;
  
  HELIX_MAKE_EXCEPTION

private:
  LiDARLabel label_;
  int type_;
  RawCloud::Ptr raw_cloud_;
  MapCloud::Ptr local_map_;
  ros::Time local_map_time_;
  pcl::KdTreeFLANN<MapPoint> map_kd_tree_;
  SurfelMap::Ptr local_surfel_map_;
  Odometry odom_;
  std::shared_ptr<Sensor> sensor_ = std::make_shared<Sensor>();
  std::string frame_id_;

public:
  LiDAR(LiDARLabel label = UNKNOWN_LIDAR) : label_(label)
  {
  }

  LiDARLabel label() const
  {
    return label_;
  }

  void setLabel(LiDARLabel label)
  {
    label_ = label;
  }

  int type() const
  {
    return type_;
  }

  void setType(int type)
  {
    type_ = type;
  }

  void initRawCloud()
  {
    raw_cloud_ = pcl::make_shared<RawCloud>();
  }

  const RawCloud::ConstPtr raw_cloud() const
  {
    return raw_cloud_;
  }

  const MapCloud::ConstPtr local_map() const
  {
    return local_map_;
  }

  const SurfelMap::ConstPtr local_surfel_map() const
  {
    return local_surfel_map_;
  }

  const Odometry& odom() const
  {
    return odom_;
  }

  Odometry& odom()
  {
    return odom_;
  }

  std::shared_ptr<Sensor> sensor()
  {
    return sensor_;
  }

  const std::shared_ptr<const Sensor> sensor() const
  {
    return sensor_;
  }

  const std::string& frame_id() const
  {
    return frame_id_;
  }

  template <typename PointT>
  void loadROSBagCallback(const rosbag::MessageInstance& m);

  void loadROSBagLivoxCallback(const rosbag::MessageInstance& m);

  void afterLoading()
  {
    raw_cloud_->points.shrink_to_fit();
    std::sort(raw_cloud_->begin(), raw_cloud_->end(),
              [](const RawPoint& lhs, const RawPoint& rhs) { return lhs.t < rhs.t; });
    for (std::uint32_t i = 0; i < raw_cloud_->size(); i++)
    {
      (*raw_cloud_)[i].pindex = i;
    }
  }

  void buildKDTree()
  {
    if (!local_map_)
    {
      HELIX_THROW("No local map!");
    }
    map_kd_tree_.setInputCloud(local_map_);
  }

  /**
   * \brief Project LiDAR raw cloud to LiDAR local map.
   * \param[in] odom Odometry of the map sensor
   * \param[in] map_time Time when sensor local frame is the map frame
   */
  void buildLocalMap(const Odometry& odom, ros::Time map_time);

  /**
   * \brief Project LiDAR raw cloud to LiDAR local map.
   * \param[in] map_time Time when sensor local frame is the map frame
   */
  void buildLocalMap(ros::Time map_time)
  {
    buildLocalMap(odom_, map_time);
  }

  /**
   * \brief Build local map using IMU trajectory.
   * \param[in] traj Trajectory of IMU
   * \param[in] map_time Time when sensor local frame is the map frame
   * \param[in] T_LtoI Transform bringing point of LiDAR frame IMU
   */
  void buildLocalMap(const kontiki::trajectories::SplitTrajectory& traj, ros::Time map_time,
                     const RigidTransform& T_LtoI);

  /**
   * \brief Build local surfel map, associate points to map, and down sample surfel points.
   * \param[in] down_sample_rate Down sample rate. No down sampling if this < 1.
   */
  void buildLocalSurfelMap(const std::vector<float>& roi_max, const std::vector<float>& roi_min, double surfel_size,
                           int down_sampling_rate)
  {
    if (!local_map_)
    {
      HELIX_THROW("Local map is empty!");
    }

    local_surfel_map_ = std::make_shared<SurfelMap>(label_);
    unsigned int label = label_;
    ROS_INFO("LiDAR[%u]: Building Surfel Map...", label);
    local_surfel_map_->setROIMax(roi_max);
    local_surfel_map_->setROIMin(roi_min);
    local_surfel_map_->setSurfelSize(surfel_size);
    local_surfel_map_->build(local_map_, local_map_time_);

    ROS_INFO("LiDAR[%u]: Got %zu surfels.", label, local_surfel_map_->surfels().size());
    ROS_INFO("LiDAR[%u]: Associating Surfel Map...", label);
    // local_surfel_map_->associateCloud(map_kd_tree_, raw_cloud_);
    local_surfel_map_->associateInliers(raw_cloud_);
    ROS_INFO("LiDAR[%u]: Got %zu surfel points.", label, local_surfel_map_->spoints().size());

    if (down_sampling_rate >= 1)
    {
      ROS_INFO("LiDAR[%u]: Down Sampling...", label);
      local_surfel_map_->downSampleSimple(down_sampling_rate);
    }
    ROS_INFO("LiDAR[%u]: Surfel map built.", label);
  }

  /**
   * \brief Project LiDAR raw cloud to a global map.
   * \param[in] traj Trajectory of the map sensor
   * \param[in] map_time Time when sensor local frame is the map frame
   * \param[in] T_LtoS Transform bringing point of LiDAR frame to map sensor frame
   * \param[out] out_cloud Point cloud in map frame
   */
  void projectPointCloudToGlobalMap(const kontiki::trajectories::SplitTrajectory& traj, ros::Time map_time,
                                    const RigidTransform& T_LtoS, helix::MapCloud& out_cloud);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IMUData
{
  double t;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IMU
{
public:
  using Sensor = kontiki::sensors::ConstantBiasImu;
  using GyroMeasurement = kontiki::measurements::GyroscopeMeasurement<Sensor>;
  using AccelMeasurement = kontiki::measurements::AccelerometerMeasurement<Sensor>;

private:
  AlignedVector<IMUData> data_;
  std::shared_ptr<Sensor> sensor_ = std::make_shared<Sensor>();
  std::string frame_id_;
  double scaling_ = 1;

public:
  IMU(const std::string& frame_name = "") : frame_id_(frame_name)
  {
  }

  void setAccScaling(double scale_by)
  {
    scaling_ = scale_by;
  }

  auto& data()
  {
    return data_;
  }

  const auto& data() const
  {
    return data_;
  }

  auto sensor()
  {
    return sensor_;
  }

  std::string& frame_id()
  {
    return frame_id_;
  }

  const std::string& frame_id() const
  {
    return frame_id_;
  }

  void loadROSBagCallback(const rosbag::MessageInstance& m);

  void afterLoading()
  {
    data_.shrink_to_fit();
    std::sort(data_.begin(), data_.end(), [](const IMUData& lhs, const IMUData& rhs) { return lhs.t < rhs.t; });
  }
};
}  // namespace helix

template <typename PointT>
void helix::LiDAR::loadROSBagCallback(const rosbag::MessageInstance& m)
{
  sensor_msgs::PointCloud2::Ptr p_msg = m.instantiate<sensor_msgs::PointCloud2>();

  if (p_msg)
  {
    pcl::PointCloud<PointT> cloud_t;
    pcl::fromROSMsg(*p_msg, cloud_t);
    for (auto& point : cloud_t)
    {
      raw_cloud_->push_back(PointXYZL2T::toXYZL2T(label_, *p_msg, point));
    }

    if (frame_id_.empty())
    {
      frame_id_ = p_msg->header.frame_id;
    }
  }
}
