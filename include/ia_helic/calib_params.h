#pragma once
#include <cassert>
#include <cstddef>
#include <ostream>
#include <sstream>

#include "Eigen/Core"
#include "Eigen/src/Core/IO.h"
#include "fmt/format.h"
#include "ia_helic/utils.h"

namespace ia_helic
{
struct CalibParams
{
private:
  // Extrnsics
  AlignedVector<RigidTransform> L_to_L0_;
  RigidTransform L0_to_I_;

  // IMU Intrinsics
  Eigen::Vector3d gravity_ = { 0, 0, -9.805 };
  double time_offset_ = 0;
  Eigen::Vector3d acc_bias_ = { 0, 0, 0 };
  Eigen::Vector3d gyro_bias_ = { 0, 0, 0 };

public:
  CalibParams(std::size_t n_lidar) : L_to_L0_(n_lidar)
  {
  }

  auto& L_to_L0()
  {
    return L_to_L0_;
  }

  const auto& L_to_L0() const
  {
    return L_to_L0_;
  }

  auto& L0_to_I()
  {
    return L0_to_I_;
  }

  const auto& L0_to_I() const
  {
    return L0_to_I_;
  }

  void setL_to_I(LiDARLabel i, RigidTransform& L_to_I)
  {
    if (i == 0)
    {
      L0_to_I_ = L_to_I;
    }
    else
    {
      L_to_L0_[i] = L0_to_I_.inverse() * L_to_I;
    }
  }

  const RigidTransform Li_to_Lj(LiDARLabel i, LiDARLabel j) const
  {
    return L_to_L0_.at(j).inverse() * L_to_L0_.at(i);
  }

  const RigidTransform Li_to_I(LiDARLabel i) const
  {
    return L0_to_I_ * L_to_L0_.at(i);
  }

  auto& gravity()
  {
    return gravity_;
  }

  const auto& gravity() const
  {
    return gravity_;
  }

  auto& acc_bias()
  {
    return acc_bias_;
  }

  const auto& acc_bias() const
  {
    return acc_bias_;
  }

  auto& gyro_bias()
  {
    return gyro_bias_;
  }

  const auto& gyro_bias() const
  {
    return gyro_bias_;
  }

  double time_offset() const
  {
    return time_offset_;
  }

  double& time_offset()
  {
    return time_offset_;
  }

  std::ostream& writeIMUParams(std::ostream& os) const
  {
    using namespace fmt::literals;
    const static auto FMT = Eigen::IOFormat(Eigen::StreamPrecision, 0, "", ", ", "", "", "[", "]");
    std::stringstream ss_acc, ss_gyro, ss_g;
    ss_acc << acc_bias_.format(FMT);
    ss_gyro << gyro_bias_.format(FMT);
    ss_g << gravity_.format(FMT);
    os << fmt::format(
        "imu_parameters:\n"
        "  time_offset: {to}\n"
        "  accelerometer_bias: {acc}\n"
        "  gyroscope_bias: {gyro}\n"
        "gravity: {g}\n", "to"_a = time_offset_, "acc"_a = ss_acc.str(), "gyro"_a = ss_gyro.str(), "g"_a = ss_g.str());
    return os;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ia_helic
