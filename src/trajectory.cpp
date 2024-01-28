#include "helix_calib/trajectory.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <numeric>
#include <ostream>
#include <vector>

#include "helix_calib/calib_params.h"
#include "helix_calib/ceres_callbacks.h"
#include "helix_calib/sensor.h"
#include "helix_calib/surfel_map.h"
#include "helix_calib/utils.h"
#include "kontiki/trajectories/split_trajectory.h"

using namespace kontiki::trajectories;

/* --- measurement adding helper functions --- */
template <typename TrajectoryModel>
void addGyroscopeMeasurements(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
                              std::shared_ptr<helix::IMU::Sensor> imu,
                              const helix::AlignedVector<helix::IMUData>& imu_data,
                              helix::AlignedVector<std::shared_ptr<helix::IMU::GyroMeasurement>>& measurements,
                              double weight)
{
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();
  measurements.clear();

  std::size_t cnt = 0;

  for (const auto& datum : imu_data)
  {
    double time = datum.t;
    if (min_time > time || max_time <= time)
    {
      continue;
    }
    auto mg = std::make_shared<helix::IMU::GyroMeasurement>(imu, time, datum.gyro, weight);
    measurements.push_back(mg);
    estimator->template AddMeasurement<helix::IMU::GyroMeasurement>(mg);
    cnt++;
  }
  ROS_INFO("#Gyro: %zu.", cnt);
}

template <typename TrajectoryModel>
void addAccelerometerMeasurements(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
                                  std::shared_ptr<helix::IMU::Sensor> imu,
                                  const helix::AlignedVector<helix::IMUData>& imu_data,
                                  helix::AlignedVector<std::shared_ptr<helix::IMU::AccelMeasurement>>& measurements,
                                  double weight)
{
  measurements.clear();
  const double min_time = estimator->trajectory()->MinTime();
  const double max_time = estimator->trajectory()->MaxTime();

  for (const auto& datum : imu_data)
  {
    if (min_time > datum.t || max_time <= datum.t)
    {
      continue;
    }
    auto ma = std::make_shared<helix::IMU::AccelMeasurement>(imu, datum.t, datum.accel, weight);
    measurements.push_back(ma);
    estimator->template AddMeasurement<helix::IMU::AccelMeasurement>(ma);
  }
}
/*
template <typename TrajectoryModel>
void addGlobalSurfMeasurements(
    std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
    std::shared_ptr<helix::LiDAR::Sensor> lidar, helix::LiDARLabel lidar_label, const helix::SurfelMap& surfel_map,
    helix::AlignedVector<std::shared_ptr<helix::Trajectory::GlobalSurfelMeasurement>>& measurements,
    helix::AlignedVector<Eigen::Vector3d>& plane_param_vec, double weight)
{
  double map_time = surfel_map.map_time().toSec();
  const double min_time = std::max(map_time, estimator->trajectory()->MinTime());
  const double max_time = estimator->trajectory()->MaxTime();
  for (auto const& spoint : surfel_map.spoints_down_sampled())
  {
    double time = spoint.raw_point.t;
    if (spoint.raw_point.label == lidar_label && time >= min_time && time < max_time)
    {
      size_t plane_id = spoint.plane_id;
      Eigen::Vector3d point(spoint.raw_point.getVector3fMap().cast<double>());

      auto msp = std::make_shared<helix::Trajectory::GlobalSurfelMeasurement>(
          lidar, point, plane_param_vec.at(plane_id).data(), time, map_time, 5.0, weight);
      msp->Lock(true);
      measurements.push_back(msp);
      estimator->template AddMeasurement<helix::Trajectory::GlobalSurfelMeasurement>(msp);
    }
  }
}
*/

template <typename TrajectoryModel>
void addLocalSurfMeasurements(
    std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
    std::shared_ptr<helix::LiDAR::Sensor> lidar, const helix::SurfelMap& surfel_map,
    helix::AlignedVector<std::shared_ptr<helix::Trajectory::LocalSurfelMeasurement>>& measurements,
    helix::AlignedVector<Eigen::Vector3d>& plane_param_vec, double weight)
{
  double map_time = surfel_map.map_time().toSec();
  const double min_time = std::max(map_time, estimator->trajectory()->MinTime());
  const double max_time = estimator->trajectory()->MaxTime();
  for (auto const& spoint : surfel_map.spoints_down_sampled())
  {
    double time = spoint.raw_point.t;
    if (time >= min_time && time < max_time)
    {
      size_t plane_id = spoint.plane_id;
      Eigen::Vector3d point(spoint.raw_point.getVector3fMap().cast<double>());

      auto msp = std::make_shared<helix::Trajectory::LocalSurfelMeasurement>(
          lidar, point, plane_param_vec.at(plane_id).data(), time, map_time, 5.0, weight);
      msp->Lock(true);
      measurements.push_back(msp);
      estimator->template AddMeasurement<helix::Trajectory::LocalSurfelMeasurement>(msp);
    }
  }
}

template <typename TrajectoryModel>
void addCrossSurfMeasurements(
    std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
    std::shared_ptr<helix::LiDAR::Sensor> lidar_spoint, std::shared_ptr<helix::LiDAR::Sensor> lidar_surfel,
    helix::AlignedVector<Eigen::Vector3d>& plane_param_vec, const helix::CrossSurfelMap& cross_surfel_map,
    helix::AlignedVector<std::shared_ptr<helix::Trajectory::CrossSurfelMeasurement>>& measurements, double weight)
{
  double map_time = cross_surfel_map.map_time().toSec();
  const double min_time = std::max(map_time, estimator->trajectory()->MinTime());
  const double max_time = estimator->trajectory()->MaxTime();
  for (auto const& spoint : cross_surfel_map.spoints())
  {
    double time = spoint.raw_point.t;
    if (time >= min_time && time < max_time)
    {
      size_t plane_id = spoint.plane_id;
      Eigen::Vector3d point(spoint.raw_point.getVector3fMap().cast<double>());

      auto msp = std::make_shared<helix::Trajectory::CrossSurfelMeasurement>(
          lidar_spoint, lidar_surfel, point, plane_param_vec.at(plane_id).data(), time, map_time, 5.0, weight);
      msp->Lock(true);
      measurements.push_back(msp);
      estimator->template AddMeasurement<helix::Trajectory::CrossSurfelMeasurement>(msp);
    }
  }
}

/*
template <typename TrajectoryModel>
void addGlobalSurfelRefineCallback(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
                                   std::shared_ptr<helix::LiDAR::Sensor> lidar)
{
  // Add callback for debug
  std::unique_ptr<helix::CheckStateCallback> cb = std::make_unique<helix::CheckStateCallback>();
  cb->addCheckState("q_LtoL0     :", 4, lidar->relative_orientation().coeffs().data());
  cb->addCheckState("p_LinL0     :", 3, lidar->relative_position().data());
  // imu not used
  // cb->addCheckState("time_offset:", 1, &lidar->time_offset());
  // cb->addCheckState("g_roll     :", 1, &imu->gravity_orientation_roll());
  // cb->addCheckState("g_pitch    :", 1, &imu->gravity_orientation_pitch());
  estimator->AddCallback(std::move(cb), true);
}
*/

template <typename TrajectoryModel>
void addLocalSurfelRefineCallback(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator,
                                  std::shared_ptr<helix::LiDAR::Sensor> lidar, std::shared_ptr<helix::IMU::Sensor> imu)
{
  // Add callback for debug
  std::unique_ptr<helix::CheckStateCallback> cb = std::make_unique<helix::CheckStateCallback>();
  cb->addCheckState("q_LtoI      :", 4, lidar->relative_orientation().coeffs().data());
  cb->addCheckState("p_LinI      :", 3, lidar->relative_position().data());
  cb->addCheckState("time_offset:", 1, &lidar->time_offset());
  cb->addCheckState("g_roll     :", 1, &imu->gravity_orientation_roll());
  cb->addCheckState("g_pitch    :", 1, &imu->gravity_orientation_pitch());
  estimator->AddCallback(std::move(cb), true);
}

void helix::Trajectory::initSO3TrajWithGyro(IMU& imu)
{
  assert(!imu.data().empty() && "[initialSO3TrajWithGyro]: There's NO imu data for initialization.");
  auto estimator_SO3 = std::make_shared<SO3TrajEstimator>(traj_->SO3Spline());

  ROS_INFO("Add Gyroscope measurements...");
  addGyroscopeMeasurements(estimator_SO3, imu.sensor(), imu.data(), gyro_measurements_, global_opt_gyro_weight_);

  // fix the initial pose of trajectory
  double weight_t0 = global_opt_gyro_weight_;
  double t0 = traj_->SO3Spline()->MinTime();

  // using identity will cause NaN when solving
  Eigen::Quaterniond q0(Eigen::AngleAxisd(0.0001, Eigen::Vector3d(0, 0, 1)));
  auto m_q0 = std::make_shared<OrientationMeasurement>(t0, q0, weight_t0);
  estimator_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

  ROS_INFO("Solving IMU Trajectory.");
  ceres::Solver::Summary summary = estimator_SO3->Solve(30, false);
  std::cout << summary.BriefReport() << std::endl;
}

/*
void helix::Trajectory::trajRefineWithGlobalSurfel(helix::AlignedVector<helix::LiDAR>& lidars,
                                                   const helix::SurfelMap& surfel_map, CalibParams& params,
                                                   bool opt_time_offset)
{
  AlignedVector<std::shared_ptr<GlobalSurfelMeasurement>> surfel_measurements;
  AlignedVector<Eigen::Vector3d> plane_param_vec;
  lidars[0].sensor()->LockRelativeOrientation(true);
  lidars[0].sensor()->LockRelativePosition(true);
  for (std::size_t i = 1; i < lidars.size(); i++)
  {
    auto sensor = lidars[i].sensor();
    sensor->set_relative_orientation(params.L_to_L0()[i].unit_quaternion());
    sensor->set_relative_position(params.L_to_L0()[i].translation());
    sensor->LockRelativeOrientation(false);
    sensor->LockRelativePosition(false);
    if (opt_time_offset && time_offset_padding_ > 0)
    {
      sensor->LockTimeOffset(false);
      sensor->set_max_time_offset(time_offset_padding_);
    }
    else
    {
      sensor->LockTimeOffset(true);
    }
  }

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  plane_param_vec.resize(surfel_map.surfels().size());
  for (std::size_t i = 0; i < surfel_map.surfels().size(); i++)
  {
    plane_param_vec[i] = surfel_map.surfels()[i].Pi;
  }
  for (auto& lidar : lidars)
  {
    addGlobalSurfMeasurements(estimator_split, lidar.sensor(), lidar.label(), surfel_map, surfel_measurements,
                              plane_param_vec, global_opt_lidar_weight);
    addGlobalSurfelRefineCallback(estimator_split, lidar.sensor());
  }
  ROS_INFO("#Surfel Measurements: %zu.", surfel_measurements.size());
  ROS_INFO("Refine Trajectory...");
  ceres::Solver::Summary summary = estimator_split->Solve(50, true);
  ROS_INFO("%s", summary.BriefReport().c_str());
  for (std::size_t i = 1; i < lidars.size(); i++)
  {
    params.L_to_L0()[i].setQuaternion(lidars[i].sensor()->relative_orientation());
    params.L_to_L0()[i].translation() = lidars[i].sensor()->relative_position();
  }
}
*/

template <class SurfelMeasurementT>
void getSurfelError(const kontiki::trajectories::SplitTrajectory& traj,
                    const helix::AlignedVector<std::shared_ptr<SurfelMeasurementT>>& measurements,
                    helix::ErrorStat& stat)
{
  stat.size_ = measurements.size();
  if (stat.size_ > 0)
  {
    helix::AlignedVector<double> errors(stat.size_);
    for (auto& m : measurements)
    {
      errors.push_back(std::abs(m->template point2plane<kontiki::trajectories::SplitTrajectory>(traj)(0, 0)));
    }
    double avg = std::accumulate(errors.begin(), errors.end(), 0.) / measurements.size();
    stat.avg_ = avg;
    double dist_sq = std::accumulate(errors.begin(), errors.end(), 0.,
                                     [avg](double dist_sq, double val) { return dist_sq + (val - avg) * (val - avg); });
    stat.std_ = std::sqrt(dist_sq / stat.size_);
  }
  else
  {
    stat.avg_ = stat.std_ = 0;
  }
}

void helix::Trajectory::trajRefineWithLocalAndCrossSurfel(AlignedVector<LiDAR>& lidars, IMU& imu, CalibParams& params,
                                                          const AlignedVector<CrossSurfelMap::Ptr>& cross_surfel_maps,
                                                          bool opt_time_offset, bool wo_imu, bool wo_cross_surfel,
                                                          bool wo_local_surfel)
{
  AlignedVector<std::shared_ptr<LocalSurfelMeasurement>> surfel_measurements;
  AlignedVector<std::shared_ptr<CrossSurfelMeasurement>> cross_surfel_measurements;
  AlignedVector<std::shared_ptr<IMU::AccelMeasurement>> acc_measurements;
  AlignedVector<std::shared_ptr<helix::IMU::GyroMeasurement>> gyro_measurements;
  AlignedVector<AlignedVector<Eigen::Vector3d>> plane_params_of_lidars(lidars.size());

  std::shared_ptr<SplitTrajEstimator> estimator_split;
  estimator_split = std::make_shared<SplitTrajEstimator>(traj_);

  auto imu_sensor = imu.sensor();
  imu_sensor->LockAccelerometerBias(false);
  imu_sensor->LockGyroscopeBias(false);
  if (!wo_imu)
  {
    imu_sensor->LockAccelerometerBias(false);
    imu_sensor->LockGyroscopeBias(false);
    addAccelerometerMeasurements(estimator_split, imu_sensor, imu.data(), acc_measurements, global_opt_acc_weight_);
    addGyroscopeMeasurements(estimator_split, imu_sensor, imu.data(), gyro_measurements, global_opt_acc_weight_);
  }
  else
  {
    imu_sensor->LockAccelerometerBias(true);
    imu_sensor->LockGyroscopeBias(true);
  }
  auto pm = std::make_shared<PositionMeasurement>(traj_->MinTime(), Eigen::Vector3d(0, 0, 0), global_opt_acc_weight_);
  estimator_split->AddMeasurement(pm);

  for (auto& lidar : lidars)
  {
    auto lidar_sensor = lidar.sensor();
    LiDARLabel label = lidar.label();
    lidar_sensor->set_relative_orientation(params.Li_to_I(label).unit_quaternion());
    lidar_sensor->set_relative_position(params.Li_to_I(label).translation());
    lidar_sensor->LockRelativeOrientation(false);
    lidar_sensor->LockRelativePosition(false);
    if (opt_time_offset && time_offset_padding_ > 0)
    {
      lidar_sensor->LockTimeOffset(false);
      lidar_sensor->set_max_time_offset(time_offset_padding_);
    }
    else
    {
      lidar_sensor->LockTimeOffset(true);
    }

    auto surfel_map = lidar.local_surfel_map();
    if (!surfel_map)
    {
      HELIX_THROW(fmt::format("No surfel map for LiDAR {}!", lidar.label()));
    }
    auto& plane_params = plane_params_of_lidars[label];
    plane_params.resize(surfel_map->surfels().size());
    for (std::size_t i = 0; i < surfel_map->surfels().size(); i++)
    {
      plane_params[i] = surfel_map->surfels()[i].Pi;
    }

    if (!wo_local_surfel)
    {
      addLocalSurfMeasurements(estimator_split, lidar_sensor, *surfel_map, surfel_measurements, plane_params,
                               global_opt_lidar_weight_);
    }
    addLocalSurfelRefineCallback(estimator_split, lidar_sensor, imu_sensor);
  }
  if (wo_imu)
  {
    lidars[0].sensor()->LockRelativeOrientation(true);
    lidars[0].sensor()->LockRelativePosition(true);
  }

  if (!wo_cross_surfel)
  {
    for (auto p_cross_surfel_map : cross_surfel_maps)
    {
      if (p_cross_surfel_map)
      {
        addCrossSurfMeasurements(estimator_split, lidars[p_cross_surfel_map->spoint_label()].sensor(),
                                 lidars[p_cross_surfel_map->surfel_label()].sensor(),
                                 plane_params_of_lidars[p_cross_surfel_map->surfel_label()], *p_cross_surfel_map,
                                 cross_surfel_measurements, global_opt_cross_lidar_weight_);
      }
    }
  }

  getSurfelError(*traj_, cross_surfel_measurements, last_before_opt_.cross_surfel_);
  ROS_INFO("Before refine: Avg. Cross Surfel Error = %f.", last_before_opt_.cross_surfel_.avg_);
  getSurfelError(*traj_, surfel_measurements, last_before_opt_.local_surfel_);
  ROS_INFO("Before refine: Avg. Local Surfel Error = %f.", last_before_opt_.local_surfel_.avg_);

  ROS_INFO("Refine Trajectory...");
  ceres::Solver::Summary summary = estimator_split->Solve(max_iterations_, true);
  ROS_INFO("%s", summary.BriefReport().c_str());

  getSurfelError(*traj_, cross_surfel_measurements, last_after_opt_.cross_surfel_);
  ROS_INFO("After refine: Avg. Cross Surfel Error = %f.", last_after_opt_.cross_surfel_.avg_);
  getSurfelError(*traj_, surfel_measurements, last_after_opt_.local_surfel_);
  ROS_INFO("After refine: Avg. Local Surfel Error = %f.", last_after_opt_.local_surfel_.avg_);

  params.acc_bias() = imu_sensor->accelerometer_bias();
  params.gyro_bias() = imu_sensor->gyroscope_bias();
  params.gravity() = imu_sensor->refined_gravity();

  for (const auto& lidar : lidars)
  {
    auto lidar_sensor = lidar.sensor();
    RigidTransform LtoI = RigidTransform(lidar_sensor->relative_orientation(), lidar_sensor->relative_position());
    params.setL_to_I(lidar.label(), LtoI);
  }
}

/*
void helix::Trajectory::initTrajWithLidarOdom(const LiDAR& lidar)
{
  if (lidar.odom().empty())
  {
    HELIX_THROW(fmt::format("No odometry data for LiDAR {}!", lidar.label()));
  }

  AlignedVector<std::shared_ptr<OrientationMeasurement>> om_vec;
  AlignedVector<std::shared_ptr<PositionMeasurement>> pm_vec;
  auto estimator_so3 = std::make_shared<SO3TrajEstimator>(traj_->SO3Spline());
  auto estimator_r3 = std::make_shared<R3TrajEstimator>(traj_->R3Spline());
  Eigen::AngleAxisd rotation_vector(0.0001, Eigen::Vector3d(0, 0, 1));
  Eigen::Quaterniond q0 = Eigen::Quaterniond(rotation_vector.matrix());
  double t0 = traj_->MinTime();
  // estimator_so3->AddMeasurement<OrientationMeasurement>(mq0);
  for (const auto& pose : lidar.odom().poses())
  {
    double t = pose.t().toSec();
    if (traj_->MinTime() <= t && t < traj_->MaxTime())
    {
      auto pm = std::make_shared<PositionMeasurement>(t, pose.position(), 1);
      pm_vec.push_back(pm);
      estimator_r3->AddMeasurement<PositionMeasurement>(pm);
      auto q = pose.orientation();
      if (q.angularDistance(Eigen::Quaterniond::Identity()) > 0.0001)
      {
        auto om = std::make_shared<OrientationMeasurement>(t, q, 1);
        om_vec.push_back(om);
        estimator_so3->AddMeasurement<OrientationMeasurement>(om);
      }
    }
  }
  ceres::Solver::Summary summary = estimator_so3->Solve(100, false);
  ROS_INFO("%s", summary.BriefReport().c_str());
  estimator_r3->Solve(false);
  ROS_INFO("%s", summary.BriefReport().c_str());
}
*/
