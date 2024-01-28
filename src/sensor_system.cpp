#include "helix_calib/sensor_system.h"

#include <algorithm>
#include <boost/throw_exception.hpp>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>

#include "Eigen/Geometry"
#include "helix_calib/odom.h"
#include "helix_calib/sensor.h"
#include "helix_calib/surfel_map.h"
#include "helix_calib/trajectory.h"
#include "helix_calib/utils.h"
#include "pcl/common/io.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/make_shared.h"
#include "pcl/point_cloud.h"

#ifdef PCL_NO_PRECOMPILE
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#endif

void helix::SensorSystem::alignCloudNDT(const helix::NDTAligner& ndt)
{
  if (ndt.is_enabled())
  {
    cloud_map_ = pcl::make_shared<MapCloud>();
    MapCloud ndt_out;
    auto ref_cloud = lidars_[0].local_map();
    if (!ref_cloud)
    {
      HELIX_THROW("No local map for LiDAR[0]!");
    }

    pcl::copyPointCloud(*ref_cloud, *cloud_map_);
    for (std::size_t i = 1; i < lidars_.size(); i++)
    {
      auto transformed_cloud = pcl::make_shared<MapCloud>();
      auto cur_cloud = lidars_[i].local_map();
      if (!cur_cloud)
      {
        HELIX_THROW(fmt::format("No local map for LiDAR[{}]!", i));
      }
      ROS_INFO("NDT Aligning for LiDAR[%zu] -> LiDAR[0]...", i);
      ndt.align(cur_cloud, ref_cloud, params_.L_to_L0()[i], ndt_out);
      pcl::transformPointCloud(*cur_cloud, *transformed_cloud, params_.L_to_L0()[i].matrix());
      *cloud_map_ += *transformed_cloud;
      ROS_INFO("NDT Aligning for LiDAR[%zu] -> LiDAR[0] is done.", i);
    }
    ROS_INFO("Get %zu map points", cloud_map_->size());
  }
  else
  {
    HELIX_THROW("NDT aligner not enabled!");
  }
}

void helix::SensorSystem::buildGlobalCloudMap(bool use_trajectory)
{
  if (!map_time_.isValid())
  {
    HELIX_THROW("Map time is not valid!");
  }
  cloud_map_ = pcl::make_shared<MapCloud>();
  for (std::size_t i = 0; i < lidars_.size(); i++)
  {
    auto transformed_cloud = pcl::make_shared<MapCloud>();
    if (use_trajectory)
    {
      lidars_[i].projectPointCloudToGlobalMap(*traj_.get(), map_time_, params_.Li_to_I(i), *transformed_cloud);
    }
    else
    {
      auto cur_cloud = pcl::make_shared<MapCloud>();
      // lidars_[i].buildLocalMap(map_time_);
      pcl::transformPointCloud(*lidars_[i].local_map(), *transformed_cloud, params_.L_to_L0()[i].matrix());
    }
    {
      *cloud_map_ += *transformed_cloud;
    }
  }
  ROS_INFO("Get %zu map points", cloud_map_->size());
  // ROS_INFO("Building KD-Tree for Cloud Map...");
  // map_kd_tree_.setInputCloud(cloud_map_);
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived>& v3d)
{
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  m << typename Derived::Scalar(0), -v3d.z(), v3d.y(), v3d.z(), typename Derived::Scalar(0), -v3d.x(), -v3d.y(),
      v3d.x(), typename Derived::Scalar(0);
  return m;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived>& q)
{
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
  typename Derived::Scalar q4 = q.w();
  m.block(0, 0, 3, 3) << q4 * Eigen::Matrix3d::Identity() + SkewSymmetric(vq);
  m.block(3, 0, 1, 3) << -vq.transpose();
  m.block(0, 3, 3, 1) << vq;
  m(3, 3) = q4;
  return m;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived>& p)
{
  Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
  Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
  typename Derived::Scalar p4 = p.w();
  m.block(0, 0, 3, 3) << p4 * Eigen::Matrix3d::Identity() - SkewSymmetric(vp);
  m.block(3, 0, 1, 3) << -vp.transpose();
  m.block(0, 3, 3, 1) << vp;
  m(3, 3) = p4;
  return m;
}

bool EstimateRotation(const helix::Trajectory& traj, const helix::Odometry& odom, Eigen::Quaterniond& q_ItoS)
{
  constexpr int FLAGS = kontiki::trajectories::EvalOrientation;
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> p_traj = traj.get();

  assert(p_traj);

  helix::AlignedVector<Eigen::Matrix4d> A_vec;
  for (size_t j = 1; j < odom.poses().size(); ++j)
  {
    size_t i = j - 1;
    double ti = odom.poses().at(i).t().toSec();
    double tj = odom.poses().at(j).t().toSec();
    if (ti < p_traj->MinTime())
    {
      continue;
    }
    else if (tj >= p_traj->MaxTime())
    {
      break;
    }
    auto result_i = p_traj->Evaluate(ti, FLAGS);
    auto result_j = p_traj->Evaluate(tj, FLAGS);
    Eigen::Quaterniond delta_qij_imu = result_i->orientation.conjugate() * result_j->orientation;

    Eigen::Matrix3d R_Si_toS0 = odom.poses().at(i).orientation().matrix();
    Eigen::Matrix3d R_Sj_toS0 = odom.poses().at(j).orientation().matrix();
    Eigen::Matrix3d delta_ij_sensor = R_Si_toS0.transpose() * R_Sj_toS0;
    Eigen::Quaterniond delta_qij_sensor(delta_ij_sensor);

    Eigen::AngleAxisd R_vector1(delta_qij_sensor.toRotationMatrix());
    Eigen::AngleAxisd R_vector2(delta_qij_imu.toRotationMatrix());
    double delta_angle = 180 / M_PI * std::fabs(R_vector1.angle() - R_vector2.angle());
    double huber = delta_angle > 1.0 ? 1.0 / delta_angle : 1.0;

    Eigen::Matrix4d lq_mat = LeftQuatMatrix(delta_qij_sensor);
    Eigen::Matrix4d rq_mat = RightQuatMatrix(delta_qij_imu);
    A_vec.push_back(huber * (lq_mat - rq_mat));
  }
  size_t valid_size = A_vec.size();
  if (valid_size < 15)
  {
    return false;
  }
  Eigen::MatrixXd A(valid_size * 4, 4);
  for (size_t i = 0; i < valid_size; ++i)
    A.block<4, 4>(i * 4, 0) = A_vec.at(i);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond q_ItoS_est(x);
  Eigen::Vector4d cov = svd.singularValues();

  if (cov(2) > 0.25)
  {
    q_ItoS = q_ItoS_est;
    return true;
  }
  else
  {
    return false;
  }
}

void helix::SensorSystem::initSystemTrajectoryIMU(double kont_distance, double time_padding)
{
  double min_time = imu_.data().front().t;
  double max_time = imu_.data().back().t;
  for (const auto& lidar : lidars_)
  {
    min_time = std::max(lidar.odom().min_t().toSec(), min_time);
    max_time = std::min(lidar.odom().max_t().toSec(), max_time);
  }
  traj_.initialTrajTo(min_time, max_time, kont_distance, time_padding);
  ROS_INFO("Trajectory time %.9f -> %.9f.", traj_.get()->MinTime(), traj_.get()->MaxTime());
  traj_.initSO3TrajWithGyro(imu_);
  ROS_INFO("Trajectory initialized using gyroscope.");
  Eigen::Quaterniond q_ItoS;
  if (EstimateRotation(traj_, lidars_[0].odom(), q_ItoS))
  {
    params_.L0_to_I().setQuaternion(q_ItoS.conjugate());
  }
  else
  {
    HELIX_THROW("Cannot solve rotation from LiDAR[0] to IMU!");
  }
}

/*
void helix::SensorSystem::initSystemTrajectoryLidar0(double kont_distance)
{
  double min_time = lidars_[0].odom().min_t().toSec();
  double max_time = lidars_[0].odom().max_t().toSec();
  ROS_INFO("Trajectory time %.9f -> %.9f.", min_time, max_time);
  traj_.initialTrajTo(min_time, max_time, kont_distance);
  traj_.initTrajWithLidarOdom(lidars_[0]);
  ROS_INFO("Trajectory initialized using LiDAR[0].");
}
*/

void helix::SensorSystem::buildSurfelMap(int down_sample_rate, bool down_sample_farthest, double surfel_size,
                                         double associated_radius)
{
  helix::AlignedVector<std::size_t> planes;
  std::vector<std::size_t> point_cnt_lidar(lidars_.size(), 0);
  surfel_map_.setSurfelSize(surfel_size);
  surfel_map_.setAssociateRadius(associated_radius);

  ROS_INFO("Building Surfel Map...");
  surfel_map_.build(cloud_map_, map_time_);
  ROS_INFO("Get %zu surfels.", surfel_map_.surfels().size());

  ROS_INFO("Associating Points, Using Multi-Threads...");
  AlignedVector<MapCloud::ConstPtr> raw_clouds(lidars_.size());
  for (std::size_t i = 0; i < lidars_.size(); i++)
  {
    raw_clouds[i] = lidars_[i].raw_cloud();
  }
  surfel_map_.associateMultipleClouds(map_kd_tree_, raw_clouds);
  ROS_INFO("Get %zu surfel points.", surfel_map_.spoints().size());
  ROS_INFO("Down Sampling Points...");
  if (down_sample_farthest)
  {
    surfel_map_.downSampleFarthest(down_sample_rate, lidars_.size());
  }
  else
  {
    surfel_map_.downSampleSimple(down_sample_rate);
  }
  std::fill(point_cnt_lidar.begin(), point_cnt_lidar.end(), 0);
  for (const auto& p : surfel_map_.spoints_down_sampled())
  {
    point_cnt_lidar[p.lidar_label]++;
  }
  for (std::size_t i = 0; i < lidars_.size(); i++)
  {
    ROS_INFO("Surfel point for LiDAR[%zu]: %zu", i, point_cnt_lidar[i]);
  }

  std::srand(std::time(0));
  auto cloud_dbg = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < 20; i++)
  {
    auto cloud_dbg_part = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    int idx = std::rand() % surfel_map_.spoints_down_sampled().size();
    pcl::copyPointCloud(*(surfel_map_.surfels()[surfel_map_.spoints_down_sampled()[idx].plane_id].cloud_inlier),
                        *cloud_dbg_part);
    for (auto& point : *cloud_dbg_part)
    {
      point.rgba = 0xBF3EFF;
    }
    pcl::PointXYZRGB point;
    point.x = surfel_map_.spoints_down_sampled()[idx].map_point.x;
    point.y = surfel_map_.spoints_down_sampled()[idx].map_point.y;
    point.z = surfel_map_.spoints_down_sampled()[idx].map_point.z;
    point.rgba = 0xFFE7BA;
    cloud_dbg_part->push_back(point);
    *cloud_dbg += *cloud_dbg_part;
  }
}

void buildSurfelPointKDTree(const helix::AlignedVector<helix::SurfelPoint>& spoints,
                            pcl::KdTreeFLANN<helix::MapPoint>& kd_tree)
{
  auto local_surfel_points = pcl::make_shared<helix::MapCloud>();
  local_surfel_points->resize(spoints.size());
  std::transform(spoints.begin(), spoints.end(), local_surfel_points->begin(),
                 [](const helix::SurfelPoint& sp) { return sp.map_point; });
  kd_tree.setInputCloud(local_surfel_points);
}

void helix::SensorSystem::associateCrossSurfelPoints()
{
  cross_surfel_maps_.resize(n_lidars() * n_lidars());
  AlignedVector<pcl::KdTreeFLANN<MapPoint>> spoint_kd_trees(lidars_.size());

#pragma omp parallel for
  for (std::size_t i = 0; i < n_lidars(); i++)
  {
    ROS_INFO("Building Surfel Point KD-Tree of LiDAR[%zu]...", i);
    buildSurfelPointKDTree(lidars_[i].local_surfel_map()->spoints_down_sampled(), spoint_kd_trees[i]);
    ROS_INFO("Surfel point KD-Tree of LiDAR[%zu] built.", i);
  }

#pragma omp parallel for collapse(2)
  for (std::size_t i = 0; i < n_lidars(); i++)
  {
    for (std::size_t j = 0; j < n_lidars(); j++)
    {
      if (i == j)
      {
        cross_surfel_maps_[i * n_lidars() + j].reset();
      }
      else
      {
        ROS_INFO("Associating Cross Surfel Map with Surfel Points from LiDAR[%zu] -> Surfels from LiDAR[%zu]", i, j);
        cross_surfel_maps_[i * n_lidars() + j] = std::make_shared<CrossSurfelMap>(i, j, lidars_[j].local_surfel_map());
        cross_surfel_maps_[i * n_lidars() + j]->associate(spoint_kd_trees[i], params_.Li_to_Lj(i, j),
                                                          *lidars_[i].raw_cloud());
        ROS_INFO("Got %zu surfel points from LiDAR[%zu] -> surfels from LiDAR[%zu].",
                 cross_surfel_maps_[i * n_lidars() + j]->spoints().size(), i, j);
      }
    }
  }
}

void helix::SensorSystem::loadROSBag(const std::string& filepath, const std::vector<std::string>& lidar_topics,
                                     const std::string& imu_topic)
{
  if (lidar_topics.size() < lidars_.size())
  {
    HELIX_THROW(fmt::format("More LiDAR topics than required! Provide {} LiDAR topics, {} required.",
                            lidar_topics.size(), lidars_.size()));
  }
  else if (lidar_topics.size() > lidars_.size())
  {
    ROS_INFO("More LiDAR topics than required. %zu provided, %zu required.", lidar_topics.size(), lidars_.size());
  }
  ROS_INFO("Open ROS bag: %s.", filepath.c_str());
  ROSBagLoader loader(filepath);
  for (std::size_t i = 0; i < lidars_.size(); i++)
  {
    auto& lidar = lidars_[i];
    lidar.initRawCloud();
    switch (lidar.type())
    {
      case MECHANICAL_XYZIRT:
        loader.addCallback(lidar_topics[i],
                           std::bind(&helix::LiDAR::loadROSBagCallback<PointXYZIRT>, &lidar, std::placeholders::_1));
        break;
      case LIVOX_SOLID_STATE:
        loader.addCallback(lidar_topics[i],
                           std::bind(&helix::LiDAR::loadROSBagLivoxCallback, &lidar, std::placeholders::_1));
        break;
      case MECHANICAL_XYZ:
        loader.addCallback(lidar_topics[i],
                           std::bind(&helix::LiDAR::loadROSBagCallback<pcl::PointXYZ>, &lidar, std::placeholders::_1));
        break;
      default:
        HELIX_THROW(fmt::format("Unkown LiDAR type: {}.", lidar.type()));
    }
  }
  loader.addCallback(imu_topic, std::bind(&IMU::loadROSBagCallback, &imu_, std::placeholders::_1));
  loader.load();
  ROS_INFO("Bag loaded. Post-loading steps...");

#pragma omp parallel for
  for (auto& lidar : lidars_)
  {
    lidar.afterLoading();
  }
  imu_.afterLoading();

  for (std::size_t i = 0; i < lidars_.size(); i++)
  {
    ROS_INFO("Got %zu LiDAR points for LiDAR[%zu].", lidars_[i].raw_cloud()->size(), i);
  }
  ROS_INFO("Got %zu IMU readings.", imu_.data().size());
}
