#include "ia_helic/sensor.h"

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "ia_helic/odom.h"
#include "ia_helic/utils.h"
#include "kontiki/trajectories/split_trajectory.h"
#include "kontiki/trajectories/trajectory.h"
#include "livox_ros_driver/CustomMsg.h"
#include "omp.h"
#include "pcl/PCLPointField.h"
#include "pcl/common/io.h"
#include "pcl/make_shared.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/time.h"
#include "rosbag/bag.h"
#include "rosbag/message_instance.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sophus/so3.hpp"

#ifdef PCL_NO_PRECOMPILE
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#endif

void ia_helic::LiDAR::loadROSBagLivoxCallback(const rosbag::MessageInstance& m)
{
  livox_ros_driver::CustomMsg::Ptr p_msg_cloud = m.instantiate<livox_ros_driver::CustomMsg>();

  if (p_msg_cloud)
  {
    // directly times 1e-9 is not accurate
    double t_base = static_cast<double>(p_msg_cloud->timebase / 1'000'000'000) +
                    static_cast<double>(p_msg_cloud->timebase % 1'000'000'000) * 1e-9f;
    for (auto& p : p_msg_cloud->points)
    {
      raw_cloud_->emplace_back(p.x, p.y, p.z, label_, 0, static_cast<double>(p.offset_time) * 1e-9f + t_base);
    }

    if (frame_id_.empty())
    {
      frame_id_ = p_msg_cloud->header.frame_id;
    }
  }
}

template <typename Scalar>
inline bool isfinite(const Eigen::Matrix<Scalar, 3, 1>& vec)
{
  return std::isfinite(vec(0)) && std::isfinite(vec(1)) && std::isfinite(vec(2));
}

void ia_helic::LiDAR::projectPointCloudToGlobalMap(const kontiki::trajectories::SplitTrajectory& traj, ros::Time map_time,
                                                const RigidTransform& T_LtoS, ia_helic::MapCloud& out_cloud)
{
  constexpr std::size_t BATCH_SIZE = 65536;
  constexpr double T_TOLERENCE = 0.0005;
  constexpr int FLAGS = kontiki::trajectories::EvalOrientation | kontiki::trajectories::EvalPosition;

  if (!raw_cloud_)
  {
    IA_HELIC_THROW("Raw cloud is not initialized!");
  }

  std::size_t point_cnt = 0;
  double last_t = 0;
  auto q_LtoS = T_LtoS.unit_quaternion();
  auto p_LinS = T_LtoS.translation();
  Eigen::Quaterniond q_Sk_to_S0;
  Eigen::Vector3d p_Sk_in_S0;
  std::size_t n_raw_points = raw_cloud_->size();
  std::size_t percentage = 0;

  out_cloud.reserve(n_raw_points);

  std::size_t index = 0;
  auto ret = traj.Evaluate(map_time.toSec(), FLAGS);
  assert(ret);
  auto q_S0toM = ret->orientation;
  auto p_S0inM = ret->position;

#pragma omp parallel for
  for (std::size_t batch = 0; batch < raw_cloud_->size(); batch += BATCH_SIZE)
  {
    MapCloud batch_cloud;
    std::size_t max_index = std::min(batch + BATCH_SIZE, n_raw_points);
    Eigen::Quaterniond q_Sk_to_S0;
    Eigen::Vector3d p_Sk_in_S0;
    double last_t = 0;
    bool have_pose = false;

    for (std::size_t i = batch; i < max_index; i++)
    {
      auto& p_Lk = (*raw_cloud_)[i];
      if (p_Lk.t < traj.MinTime() || p_Lk.t >= traj.MaxTime())
      {
        continue;
      }
      if (p_Lk.t - last_t < T_TOLERENCE)
      {
        have_pose = true;  // use previous pose
      }
      else
      {
        auto ret = traj.Evaluate(p_Lk.t, FLAGS);
        q_Sk_to_S0 = ret->orientation;
        p_Sk_in_S0 = ret->position;
        last_t = p_Lk.t;
        have_pose = true;
      }
      if (have_pose)
      {
        Eigen::Vector3d p_Lk_xyz{ p_Lk.x, p_Lk.y, p_Lk.z };
        Eigen::Vector3d p_Sk = q_LtoS * p_Lk_xyz + p_LinS;
        Eigen::Vector3d p_M_xyz = q_S0toM.conjugate() * (q_Sk_to_S0 * p_Sk + p_Sk_in_S0 - p_S0inM);
        if (isfinite(p_M_xyz))
        {
          batch_cloud.emplace_back(p_M_xyz(0), p_M_xyz(1), p_M_xyz(2), p_Lk.label, p_Lk.pindex, p_Lk.t);
        }
      }
    }
#pragma omp critical
    {
      out_cloud += batch_cloud;
      point_cnt += (max_index - batch);
      std::cout << "Transforming points: " << point_cnt * 100 / n_raw_points << "%\r" << std::flush;
    }
  }
  out_cloud.points.shrink_to_fit();
}

void ia_helic::LiDAR::buildLocalMap(const Odometry& odom, ros::Time map_time)
{
  constexpr std::size_t BATCH_SIZE = 65536;
  constexpr double T_TOLERENCE = 0.0005;

  if (!raw_cloud_)
  {
    IA_HELIC_THROW("Raw cloud is not initialized!");
  }

  if (odom.empty())
  {
    IA_HELIC_THROW("Odometry is empty!");
  }

  local_map_time_ = map_time;

  std::size_t point_cnt = 0;
  std::size_t n_raw_points = raw_cloud_->size();
  PoseStamped ref_pose;

  local_map_ = pcl::make_shared<MapCloud>();
  local_map_->reserve(n_raw_points);

  std::size_t index = 0;
  bool ret = odom_.getPose(map_time, index, ref_pose);
  if (!ret)
  {
    IA_HELIC_THROW("Map time is not in odometry!");
  }

  auto T_L0toM = ref_pose.transform().inverse().cast<float>();

#pragma omp parallel for
  for (std::size_t batch = 0; batch < n_raw_points; batch += BATCH_SIZE)
  {
    std::size_t max_index = std::min(batch + BATCH_SIZE, n_raw_points);
    std::size_t index_odom = 0;
    Sophus::SE3f T_LktoL0;
    ia_helic::PoseStamped pose;
    double last_t = 0;
    MapCloud batch_cloud;
    batch_cloud.reserve(max_index);

    for (std::size_t i = batch; i < max_index; i++)
    {
      RawPoint& p_Lk = (*raw_cloud_)[i];
      bool have_pose = false;
      Eigen::Vector3f p_Lk_xyz{ p_Lk.x, p_Lk.y, p_Lk.z };
      if (p_Lk.t - last_t < T_TOLERENCE)
      {
        have_pose = true;  // use previous pose
      }
      else
      {
        have_pose = odom.getPose(ros::Time(p_Lk.t), index_odom, pose);
        last_t = p_Lk.t;
      }
      if (have_pose)
      {
        T_LktoL0 = pose.transform().cast<float>();
        Eigen::Vector3f p_M_xyz = T_L0toM * (T_LktoL0 * p_Lk_xyz);
        if (isfinite(p_M_xyz))
        {
          batch_cloud.emplace_back(p_M_xyz(0), p_M_xyz(1), p_M_xyz(2), p_Lk.label, p_Lk.pindex, p_Lk.t);
        }
      }
    }
#pragma omp critical
    {
      *local_map_ += batch_cloud;
      point_cnt += (max_index - batch);
      std::cout << "Transforming points: " << point_cnt * 100 / n_raw_points << "%\r" << std::flush;
    }
  }
  local_map_->points.shrink_to_fit();
}

void ia_helic::LiDAR::buildLocalMap(const kontiki::trajectories::SplitTrajectory& traj, ros::Time map_time,
                                 const RigidTransform& T_LtoI)
{
  constexpr double T_TOLERENCE = 0.0005;
  constexpr std::size_t BATCH_SIZE = 65536;
  constexpr int FLAGS = kontiki::trajectories::EvalOrientation | kontiki::trajectories::EvalPosition;

  if (!raw_cloud_)
  {
    IA_HELIC_THROW("Raw cloud is not initialized!");
  }

  std::size_t point_cnt = 0;

  auto q_LtoI = T_LtoI.unit_quaternion();
  auto p_LinI = T_LtoI.translation();
  std::size_t n_raw_points = raw_cloud_->size();

  local_map_ = pcl::make_shared<MapCloud>();
  local_map_->reserve(n_raw_points);

  std::size_t index = 0;
  auto ret = traj.Evaluate(map_time.toSec(), FLAGS);
  assert(ret);
  auto q_I0toIm = ret->orientation;
  auto p_I0inIm = ret->position;

#pragma omp parallel for
  for (std::size_t batch = 0; batch < n_raw_points; batch += BATCH_SIZE)
  {
    MapCloud batch_cloud;
    std::size_t max_index = std::min(batch + BATCH_SIZE, n_raw_points);
    Eigen::Quaterniond q_Ik_to_I0;
    Eigen::Vector3d p_Ik_in_I0;
    double last_t = 0;
    bool have_pose = false;

    batch_cloud.reserve(max_index);

    for (std::size_t i = batch; i < max_index; i++)
    {
      auto& p_Lk = (*raw_cloud_)[i];
      if (p_Lk.t < traj.MinTime() || p_Lk.t >= traj.MaxTime())
      {
        continue;
      }
      if (p_Lk.t - last_t < T_TOLERENCE)
      {
        have_pose = true;  // use previous pose
      }
      else
      {
        auto ret = traj.Evaluate(p_Lk.t, FLAGS);
        q_Ik_to_I0 = ret->orientation;
        p_Ik_in_I0 = ret->position;
        last_t = p_Lk.t;
        have_pose = true;
      }
      if (have_pose)
      {
        Eigen::Vector3d p_Lk_xyz{ p_Lk.x, p_Lk.y, p_Lk.z };
        Eigen::Vector3d p_Ik = q_LtoI * p_Lk_xyz + p_LinI;
        Eigen::Vector3d p_Im = q_I0toIm.conjugate() * (q_Ik_to_I0 * p_Ik + p_Ik_in_I0 - p_I0inIm);
        Eigen::Vector3d p_Lm_xyz = q_LtoI.conjugate() * (p_Im - p_LinI);
        if (isfinite(p_Lm_xyz))
        {
          batch_cloud.emplace_back(p_Lm_xyz(0), p_Lm_xyz(1), p_Lm_xyz(2), p_Lk.label, p_Lk.pindex, p_Lk.t);
        }
      }
    }
#pragma omp critical
    {
      *local_map_ += batch_cloud;
      point_cnt += (max_index - batch);
      std::cout << "Transforming points: " << point_cnt * 100 / n_raw_points << "%\r" << std::flush;
    }
  }
  local_map_->points.shrink_to_fit();
}

void ia_helic::IMU::loadROSBagCallback(const rosbag::MessageInstance& m)
{
  auto p_msg_imu = m.instantiate<sensor_msgs::Imu>();

  if (p_msg_imu)
  {
    data_.emplace_back(IMUData{ p_msg_imu->header.stamp.toSec(),
                                Eigen::Vector3d{
                                    p_msg_imu->angular_velocity.x,
                                    p_msg_imu->angular_velocity.y,
                                    p_msg_imu->angular_velocity.z,
                                },
                                Eigen::Vector3d{
                                    p_msg_imu->linear_acceleration.x * scaling_,
                                    p_msg_imu->linear_acceleration.y * scaling_,
                                    p_msg_imu->linear_acceleration.z * scaling_,
                                } });
    if (frame_id_.empty())
    {
      frame_id_ = p_msg_imu->header.frame_id;
    }
  }
}
