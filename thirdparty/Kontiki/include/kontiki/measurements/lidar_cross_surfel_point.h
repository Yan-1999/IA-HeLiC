#ifndef LIDAR_CROSS_SURFEL_POINT_H
#define LIDAR_CROSS_SURFEL_POINT_H

#include <Eigen/Dense>

#include <memory>
#include "../sensors/lidar.h"
#include "../trajectories/trajectory.h"
#include "../trajectory_estimator.h"
#include "ceres/rotation.h"

namespace kontiki
{
namespace measurements
{
template <typename LiDARModel>
class LiDARCrossSurfelPoint
{
public:
  LiDARCrossSurfelPoint(std::shared_ptr<LiDARModel> lidar_spoint, std::shared_ptr<LiDARModel> lidar_surfel,
                        Eigen::Vector3d spoint, double* plane, double timestamp, double map_time, double huber_loss,
                        double weight)
    : lidar_spoint_(lidar_spoint)
    , lidar_surfel_(lidar_surfel)
    , spoint_(spoint)
    , plane_(plane)
    , timestamp_(timestamp)
    , map_time_(map_time)
    , loss_function_(huber_loss)
    , weight(weight)
  {
  }

  LiDARCrossSurfelPoint(std::shared_ptr<LiDARModel> lidar_spoint, std::shared_ptr<LiDARModel> lidar_surfel,
                        Eigen::Vector3d spoint, double* plane, double timestamp, double map_time, double huber_loss)
    : LiDARCrossSurfelPoint(lidar_spoint, lidar_surfel, spoint, plane, timestamp, map_time, huber_loss, 1.0)
  {
  }

  LiDARCrossSurfelPoint(std::shared_ptr<LiDARModel> lidar_spoint, std::shared_ptr<LiDARModel> lidar_surfel,
                        Eigen::Vector3d spoint, double* plane, double timestamp, double map_time)
    : LiDARCrossSurfelPoint(lidar_spoint, lidar_surfel, spoint, plane, timestamp, map_time, 5.)
  {
  }

  template <typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 1, 1> reprojectPoint2map(const type::Trajectory<TrajectoryModel, T>& trajectory,
                                            const type::LiDAR<LiDARModel, T>& lidar_spoint,
                                            const type::LiDAR<LiDARModel, T>& lidar_surfel, const T* plane_cp) const
  {
    int flags = trajectories::EvaluationFlags::EvalPosition | trajectories::EvaluationFlags::EvalOrientation;

    auto T_I0toG = trajectory.Evaluate(T(map_time_) + lidar_surfel.time_offset(), flags);
    auto T_IktoG = trajectory.Evaluate(T(timestamp_) + lidar_spoint.time_offset(), flags);

    const Eigen::Matrix<T, 3, 1> p_LpinI = lidar_spoint.relative_position();
    const Eigen::Quaternion<T> q_LptoI = lidar_spoint.relative_orientation();

    const Eigen::Matrix<T, 3, 1> p_LsinI = lidar_surfel.relative_position();
    const Eigen::Quaternion<T> q_LstoI = lidar_surfel.relative_orientation();

    Eigen::Matrix<T, 3, 1> p_Lk = spoint_.cast<T>();
    Eigen::Matrix<T, 3, 1> p_I = q_LptoI * p_Lk + p_LpinI;

    Eigen::Matrix<T, 3, 1> p_temp =
        T_I0toG->orientation.conjugate() * (T_IktoG->orientation * p_I + T_IktoG->position - T_I0toG->position);
    Eigen::Matrix<T, 3, 1> p_M = q_LstoI.conjugate() * (p_temp - p_LsinI);

    Eigen::Matrix<T, 3, 1> Pi = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(plane_cp);
    T plane_d = T(Pi.norm());
    T plane_norm[3];
    plane_norm[0] = T(Pi[0]) / plane_d;
    plane_norm[1] = T(Pi[1]) / plane_d;
    plane_norm[2] = T(Pi[2]) / plane_d;

    T dist = ceres::DotProduct(plane_norm, p_M.data()) - plane_d;

    Eigen::Matrix<T, 1, 1> error(dist);
    return error;
  }

  template <typename TrajectoryModel, typename T>
  Eigen::Matrix<T, 1, 1> Error(const type::Trajectory<TrajectoryModel, T>& trajectory,
                               const type::LiDAR<LiDARModel, T>& lidar_spoint,
                               const type::LiDAR<LiDARModel, T>& lidar_surfel, const T* plane_cp) const
  {
    Eigen::Matrix<T, 1, 1> dist =
        reprojectPoint2map<TrajectoryModel, T>(trajectory, lidar_spoint, lidar_surfel, plane_cp);
    return T(weight) * (dist);
  }

  template <typename TrajectoryModel>
  Eigen::Matrix<double, 1, 1> point2plane(const type::Trajectory<TrajectoryModel, double>& trajectory) const
  {
    Eigen::Matrix<double, 1, 1> dist =
        reprojectPoint2map<TrajectoryModel, double>(trajectory, *lidar_spoint_, *lidar_surfel_, plane_);
    return dist;
  }

  void Lock(bool lock)
  {
    locked_ = lock;
  }

  bool IsLocked()
  {
    return locked_;
  }

  std::shared_ptr<LiDARModel> lidar_spoint_;
  std::shared_ptr<LiDARModel> lidar_surfel_;

  Eigen::Vector3d spoint_;
  double* plane_;  // double plane_[3]; //Eigen::Vector4d plane_;
  double timestamp_;
  double map_time_;
  double weight;

  bool locked_ = true;

protected:
  template <typename TrajectoryModel>
  struct Residual
  {
    Residual(const LiDARCrossSurfelPoint<LiDARModel>& m) : measurement(m)
    {
    }

    template <typename T>
    bool operator()(T const* const* params, T* residual) const
    {
      size_t offset = 0;
      auto trajectory = entity::Map<TrajectoryModel, T>(&params[offset], trajectory_meta);

      offset += trajectory_meta.NumParameters();
      auto lidar_spoint = entity::Map<LiDARModel, T>(&params[offset], lidar_spoint_meta);

      offset += lidar_spoint_meta.NumParameters();
      auto lidar_surfel = entity::Map<LiDARModel, T>(&params[offset], lidar_surfel_meta);

      offset += lidar_surfel_meta.NumParameters();
      const T* plane = params[offset];

      Eigen::Map<Eigen::Matrix<T, 1, 1>> r(residual);
      r = measurement.Error<TrajectoryModel, T>(trajectory, lidar_spoint, lidar_surfel, plane);
      return true;
    }

    const LiDARCrossSurfelPoint& measurement;
    typename TrajectoryModel::Meta trajectory_meta;
    typename LiDARModel::Meta lidar_spoint_meta;
    typename LiDARModel::Meta lidar_surfel_meta;
  };  // Residual;

  template <typename TrajectoryModel>
  void AddToEstimator(kontiki::TrajectoryEstimator<TrajectoryModel>& estimator)
  {
    using ResidualImpl = Residual<TrajectoryModel>;
    auto residual = new ResidualImpl(*this);
    auto cost_function = new ceres::DynamicAutoDiffCostFunction<ResidualImpl>(residual);
    std::vector<entity::ParameterInfo<double>> parameters;

    // Find timespans for the two observations
    double tmin, tmax;
    double map_min_time, map_max_time;
    double lidar_spoint_timeoffset = lidar_spoint_->TimeOffsetIsLocked() ? 0 : lidar_spoint_->max_time_offset();
    double lidar_surfel_timeoffset = lidar_surfel_->TimeOffsetIsLocked() ? 0 : lidar_surfel_->max_time_offset();
    double time_offset = std::max(lidar_spoint_timeoffset, lidar_surfel_timeoffset);

    tmin = timestamp_ - time_offset;
    tmax = timestamp_ + time_offset;

    map_min_time = map_time_;
    map_max_time = map_time_;
    map_min_time = map_time_ - time_offset;
    map_max_time = map_time_ + time_offset;

    /// A.1 先将轨迹参数添加到parameters中 (control points)
    estimator.AddTrajectoryForTimes({ { map_min_time, map_max_time }, { tmin, tmax } }, residual->trajectory_meta,
                                    parameters);

    /// A.2 再将lidar传感器的参数添加到parameters中 (relative pose and timeoffset and so on ..(if have))
    lidar_spoint_->AddToProblem(estimator.problem(), { { map_min_time, map_max_time }, { tmin, tmax } },
                                residual->lidar_spoint_meta, parameters);
    lidar_surfel_->AddToProblem(estimator.problem(), { { map_min_time, map_max_time }, { tmin, tmax } },
                                residual->lidar_surfel_meta, parameters);

    /// A.3 最后将面片参数添加至 problem
    estimator.problem().AddParameterBlock(this->plane_, 3);

    parameters.push_back(entity::ParameterInfo<double>(this->plane_, 3));
    if (this->IsLocked())
    {
      estimator.problem().SetParameterBlockConstant(this->plane_);
    }

    /// B.1 先往cost_function中添加待优化参数
    // Add parameters to cost function
    for (auto& pi : parameters)
    {
      cost_function->AddParameterBlock(pi.size);
    }
    // Add measurement info
    cost_function->SetNumResiduals(1);

    /// B.2 再添加residual
    // Give residual block to Problem
    std::vector<double*> params = entity::ParameterInfo<double>::ToParameterBlocks(parameters);

    estimator.problem().AddResidualBlock(cost_function, &loss_function_, params);
    //      estimator.problem().AddResidualBlock(cost_function,
    //                                           nullptr,
    //                                           params);

    //      estimator.problem().AddResidualBlock(cost_function,
    //                                           nullptr,
    //                                           entity::ParameterInfo<double>::ToParameterBlocks(parameters));
  }

  // The loss function is not a pointer since the Problem does not take ownership.
  ceres::HuberLoss loss_function_;

  template <typename TrajectoryModel>
  friend class kontiki::TrajectoryEstimator;
};

}  // namespace measurements
}  // namespace kontiki

#endif  // LIDAR_MEASUREMENT_H
