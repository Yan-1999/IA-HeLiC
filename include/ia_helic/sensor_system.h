#pragma once

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include "fmt/core.h"
#include "ia_helic/bag_loader.h"
#include "ia_helic/calib_params.h"
#include "ia_helic/ndt_aligner.h"
#include "ia_helic/odom_aligner.h"
#include "ia_helic/sensor.h"
#include "ia_helic/surfel_map.h"
#include "ia_helic/trajectory.h"
#include "ia_helic/utils.h"
#include "opencv2/calib3d.hpp"
#include "pcl/common/io.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/make_shared.h"
#include "pcl/point_types.h"
#include "ros/time.h"

namespace ia_helic
{
class SensorSystem
{
public:
  static constexpr int REF_LIDAR = 0;

  IA_HELIC_MAKE_EXCEPTION

private:
  AlignedVector<LiDAR> lidars_;
  IMU imu_;

  CalibParams params_;
  ros::Time map_time_ = ros::Time::ZERO;
  MapCloud::Ptr cloud_map_;
  pcl::KdTreeFLANN<MapPoint> map_kd_tree_;
  SurfelMap surfel_map_;
  AlignedVector<CrossSurfelMap::Ptr> cross_surfel_maps_;
  Trajectory traj_;

public:
  SensorSystem(std::size_t n_lidar) : lidars_(n_lidar), params_(n_lidar)
  {
    for (std::size_t i = 0; i < n_lidar; i++)
    {
      lidars_[i].setLabel(i);
    }
  }

  AlignedVector<LiDAR>& lidars()
  {
    return lidars_;
  }

  const AlignedVector<LiDAR>& lidars() const
  {
    return lidars_;
  }

  std::size_t n_lidars() const
  {
    return lidars_.size();
  }

  IMU& imu()
  {
    return imu_;
  }

  const IMU& imu() const
  {
    return imu_;
  }

  auto& params()
  {
    return params_;
  }

  const auto& params() const
  {
    return params_;
  }

  MapCloud::Ptr cloud_map()
  {
    return cloud_map_;
  }

  const MapCloud::ConstPtr cloud_map() const
  {
    return cloud_map_;
  }

  const auto& map_kd_tree() const
  {
    return map_kd_tree_;
  }

  SurfelMap& surfel_map()
  {
    return surfel_map_;
  }

  const SurfelMap& surfel_map() const
  {
    return surfel_map_;
  }

  const AlignedVector<CrossSurfelMap::Ptr>& cross_surfel_maps() const
  {
    return cross_surfel_maps_;
  }

  const auto& trajectory() const
  {
    return traj_;
  }

  auto& trajectory()
  {
    return traj_;
  }

  void clearMaps()
  {
    cloud_map_.reset();
    surfel_map_.clear();
  }

  void clearMeasurements()
  {
    traj_.clearMeasurements();
  }

  void setLiDARTypes(const std::vector<int>& types)
  {
    for (std::size_t i = 0; i < lidars_.size(); i++)
    {
      lidars_[i].setType(types[i]);
    }
  }

  void loadTUMs(const std::vector<std::string>& paths)
  {
    if (paths.size() != lidars_.size())
    {
      IA_HELIC_THROW(fmt::format("Path number do not match. Expect {}, got: {}.", lidars_.size()));
    }
    for (std::size_t i = 0; i < lidars_.size(); i++)
    {
      lidars_[i].odom().loadTUM(paths[i]);
    }
  }

  void loadROSBag(const std::string& filepath, const std::vector<std::string>& lidar_topics,
                  const std::string& imu_topic);

  void alignOdom(cv::HandEyeCalibrationMethod method = cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_TSAI)
  {
    ros::Time ref_time;
    OdomAligner oa(method);
    for (std::size_t i = 1; i < lidars_.size(); i++)
    {
      oa.align(lidars_[i].odom(), lidars_[0].odom(), params_.L_to_L0()[i], ref_time);
      map_time_ = std::max(ref_time, map_time_);
    }
    map_time_ = std::max(ros::Time(imu_.data().front().t), map_time_);
  }

  void alignCloudNDT(const ia_helic::NDTAligner& ndt);

  void buildGlobalCloudMap(bool use_trajectory);

  /**
   * \brief Build LiDAR local maps.
   */
  void buildLocalCloudMaps(bool use_trajectory)
  {
    for (std::size_t i = 0; i < lidars_.size(); i++)
    {
      auto& lidar = lidars_[i];
      if (use_trajectory)
      {
        lidar.buildLocalMap(*traj_.get(), map_time_, params_.Li_to_I(i));
      }
      else
      {
        lidar.buildLocalMap(map_time_);
      }
      ROS_INFO("%s", fmt::format("LiDAR[{}]: Got {} map points.", i, lidar.local_map()->size()).c_str());
    }
  }

  void buildSurfelMap(int down_sample_rate, bool down_sample_farthest = false, double ndt_resolution = 0.5,
                      double associated_radius = 0.05);

  /**
   *\brief Build LiDAR local surfel maps.
   */
  void buildLocalSurfelMaps(const std::vector<float>& roi_max, const std::vector<float>& roi_min, double surfel_size,
                            int down_sample_rate)
  {
#pragma omp parallel for
    for (std::size_t i = 0; i < lidars_.size(); i++)
    {
      auto& lidar = lidars_[i];
      lidar.buildLocalSurfelMap(roi_max, roi_min, surfel_size, down_sample_rate);
    }
  }

  void associateCrossSurfelPoints();

  /**
   * \brief Initialize system B-Spline trajectory as tarjectory of IMU and solves `R_L0toI`.
   * \note The trajectory time interval is LiDAR[0] odometry time interval (padding added).
   * \param[in] kont_distance B-Spline kont distance.
   * \param[in] time_padding Padding to both sides of B_Spline.
   */
  void initSystemTrajectoryIMU(double kont_distance, double time_padding);

  /**
   * \brief Initialize system B-Spline trajectory as trajectory of LiDAR[0].
   * \note The trajectory time interval is LiDAR[0] odometry time interval.
   * \param[in] kont_distance B-Spline kont distance.
   */
  // void initSystemTrajectoryLidar0(double kont_distance);

  void refineTrajectory(bool wo_imu = false, bool wo_cross_surfel = false, bool wo_local_surfel = false)
  {
    traj_.trajRefineWithLocalAndCrossSurfel(lidars_, imu_, params_, cross_surfel_maps_, false, wo_imu, wo_cross_surfel,
                                            wo_local_surfel);
  }

  std::ostream& writeCalibResults(std::ostream& os)
  {
    for (std::size_t i = 0; i < lidars_.size(); i++)
    {
      if (i != 0)
      {
        writeTransform(os, lidars_[i].frame_id(), lidars_[0].frame_id(), params_.L_to_L0()[i]);
      }
    }
    return os;
  }
};
}  // namespace ia_helic
