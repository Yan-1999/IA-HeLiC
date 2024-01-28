#pragma once

#include <fmt/core.h>
#include <cstddef>
#include <fstream>
#include <ios>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "helix_calib/calib_params.h"
#include "helix_calib/odom.h"
#include "helix_calib/sensor.h"
#include "helix_calib/surfel_map.h"
#include "helix_calib/utils.h"
#include "kontiki/measurements/lidar_common_surfel_point.h"
#include "kontiki/measurements/lidar_cross_surfel_point.h"
#include "kontiki/measurements/lidar_surfel_point.h"
#include "kontiki/measurements/orientation_measurement.h"
#include "kontiki/measurements/position_measurement.h"
#include "kontiki/trajectories/split_trajectory.h"
#include "kontiki/trajectories/trajectory.h"
#include "kontiki/trajectories/uniform_r3_spline_trajectory.h"
#include "kontiki/trajectories/uniform_so3_spline_trajectory.h"
#include "kontiki/trajectory_estimator.h"

namespace helix
{
struct ErrorStat
{
  std::size_t size_;
  double avg_;
  double std_;
};

/**
 * \brief B-Spline Trajectory manager. Initialize, manage, and refine trajectory. Also refines sensor parameters.
 * \note It uses two B-Splines: SO3 x R3
 */
class Trajectory
{
public:
  HELIX_MAKE_EXCEPTION

  using SO3TrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformSO3SplineTrajectory>;
  using R3TrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformR3SplineTrajectory>;
  using SplitTrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::SplitTrajectory>;

  using OrientationMeasurement = kontiki::measurements::OrientationMeasurement;
  using PositionMeasurement = kontiki::measurements::PositionMeasurement;
  using GlobalSurfelMeasurement = kontiki::measurements::LiDARCommonSurfelPoint<LiDAR::Sensor>;
  using LocalSurfelMeasurement = kontiki::measurements::LiDARSurfelPoint<LiDAR::Sensor>;
  using CrossSurfelMeasurement = kontiki::measurements::LiDARCrossSurfelPoint<LiDAR::Sensor>;

  struct Errors
  {
    ErrorStat local_surfel_;
    ErrorStat cross_surfel_;
  };

public:
  // optimzation parameters

  typedef std::shared_ptr<Trajectory> Ptr;
  using Result = std::unique_ptr<kontiki::trajectories::TrajectoryEvaluation<double>>;

  Trajectory()
  {
  }

  auto get() const
  {
    return traj_;
  }

  void setAccWeight(float weight)
  {
    global_opt_acc_weight_ = weight;
  }

  void setGyroWeight(float weight)
  {
    global_opt_gyro_weight_ = weight;
  }

  void setLidarWeight(float weight)
  {
    global_opt_lidar_weight_ = weight;
  }

  void setMaxIterations(std::size_t iterations)
  {
    max_iterations_ = iterations;
  }

  void clearMeasurements()
  {
    gyro_measurements_.clear();
    acc_measurements_.clear();
  }

  const Errors& lastBeforeOpt() const
  {
    return last_before_opt_;
  }

  const Errors& lasstAfterOpt() const
  {
    return last_after_opt_;
  }

  /**
   * \brief Initializing all trajectory konts to identity.
   * \note The time interval of trajectory will be [start_time - time_offset_padding, end_time + time_offset_padding).
   * \param[in] start_time Trajectory start time.
   * \param[in] end_time Trajectory end time.
   * \param[in] kont_distance B-Spline kont distance in seconds.
   * \param[in] time_offset_padding Time padding added to both sides of time interval.
   */
  void initialTrajTo(double start_time, double end_time, double knot_distance, double traj_time_padding = 0)
  {
    traj_ = std::make_shared<kontiki::trajectories::SplitTrajectory>(knot_distance, knot_distance,
                                                                     start_time - traj_time_padding, start_time);
    Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p0(0, 0, 0);
    traj_->R3Spline()->ExtendTo(end_time + traj_time_padding, p0);
    traj_->SO3Spline()->ExtendTo(end_time + traj_time_padding, q0);
  }

  /**
   * \brief Initialize SO3 (rotation) part of IMU trjectory using gyroscope readings.
   * \param[in] imu IMU.
   */
  void initSO3TrajWithGyro(IMU& imu);

  /**
   * \brief Refine trajectory with surfel measurements using global surfel map.
   * \warning This function assumes that this trajectory is the trajectory of LiDAR `lidars[0]` (not other sensor), and
   * parameters in `params.L_to_L0_[]` is consistent with LiDARs in `lidars`.
   * \pre `initTrajWithLidarOdom()` is called.
   * \param[in,out] lidars LiDARs. Modifying `sensor_` atrributes.
   * \param[in] surfel_map @b Global surfel map.
   * \param[in,out] params Extrisic parameters.
   * \param[in] opt_time_offset Optimise time offset or not.
   */
  // void trajRefineWithGlobalSurfel(AlignedVector<LiDAR>& lidars, const SurfelMap& surfel_map, CalibParams& params,
  //                                 bool opt_time_offset);

  /**
   * \brief Refine trajectory with surfel measurements using local surfel map and cross surfel maps.
   * \warning This function assumes that this trajectory is the trajectory of IMU.
   * \pre Initialization of this trajectory and estimations in `params`.
   * \note This function wll update T_L0_to_I if using LiDAR[0], otherwise will update T_Li_to_L0.
   * \param[in,out] lidars LiDARs. Modifying `sensor_` atrributes.
   * \param[in,out] imu IMU. Modifying `sensor_` atrributes.
   * \param[in,out] params Extrisic parameters.
   * \param[in] cross_surfel_maps Cross surfel maps. Input an empty vector if no cross surfel maps involved.
   * \param[in] opt_time_offset Optimise time offset or not.
   */
  void trajRefineWithLocalAndCrossSurfel(AlignedVector<LiDAR>& lidars, IMU& imu, CalibParams& params,
                                         const AlignedVector<CrossSurfelMap::Ptr>& cross_surfel_maps,
                                         bool opt_time_offset, bool wo_imu = false, bool wo_cross_surfel = false,
                                         bool wo_local_surfel = false);

  /**
   * \brief Initialize this trajectory using LiDAR odometry.
   * \param[in] lidar The LiDAR of this trajctory.
   */
  // void initTrajWithLidarOdom(const LiDAR& lidar);

  void dumpTUM(const std::string& filepath)
  {
    Odometry odom;
    constexpr int FLAGS = kontiki::trajectories::EvalOrientation | kontiki::trajectories::EvalPosition;
    for (double t = traj_->MinTime(); t < traj_->MaxTime(); t += traj_->SO3Spline()->dt())
    {
      auto ret = traj_->Evaluate(t, FLAGS);
      odom.emplace_back(t, ret->orientation, ret->position);
    }
    odom.dumpTUM(filepath);
  }

  void reportErrors(const std::string& filepath, size_t iteration)
  {
    using namespace fmt::literals;

    if (filepath.empty())
    {
      return;
    }

    std::ios_base::openmode mode = std::ios_base::out | ((iteration == 0) ? std::ios_base::trunc : std::ios_base::app);

    std::fstream fs(filepath, mode);
    if (!fs.is_open())
    {
      HELIX_THROW("Cannot open file: " + filepath);
    }
    fs << fmt::format(
              "Iteration: {iter}\n"
              "Local surfel error:\n"
              "    {local_before}\n"
              "->  {local_after}\n"
              "Cross surfel error:\n"
              "    {cross_before}\n"
              "->  {cross_after}",
              "iter"_a = iteration, "local_before"_a = last_before_opt_.local_surfel_,
              "local_after"_a = last_after_opt_.local_surfel_, "cross_before"_a = last_before_opt_.cross_surfel_,
              "cross_after"_a = last_after_opt_.cross_surfel_)
       << std::endl;
  }

private:
  float global_opt_gyro_weight_ = 28.0;
  float global_opt_acc_weight_ = 18.5;
  float global_opt_lidar_weight_ = 2.0;
  float global_opt_cross_lidar_weight_ = 10.0;
  std::size_t max_iterations_ = 50;
  double time_offset_padding_ = 0.005;
  std::shared_ptr<kontiki::trajectories::SplitTrajectory> traj_;

  // persist measurements till solving
  AlignedVector<std::shared_ptr<helix::IMU::GyroMeasurement>> gyro_measurements_;
  AlignedVector<std::shared_ptr<helix::IMU::AccelMeasurement>> acc_measurements_;

  Errors last_before_opt_;
  Errors last_after_opt_;
};
}  // namespace helix

namespace fmt
{
template <>
struct formatter<helix::ErrorStat>
{
  constexpr format_parse_context::iterator parse(format_parse_context& ctx)
  {
    auto it = ctx.begin(), end = ctx.end();
    while (it != end && *it != '}')
    {
      it++;
    }
    return it;
  }

  auto format(const helix::ErrorStat& error, format_context& ctx) const
  {
    return fmt::format_to(ctx.out(), "size: {}, avg: {}, std: {}", error.size_, error.avg_, error.std_);
  }
};
}  // namespace fmt
