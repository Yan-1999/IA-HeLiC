#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <string>

#include "ia_helic/odom.h"
#include "ia_helic/utils.h"

namespace ia_helic
{
class OdomAligner
{
public:
    IA_HELIC_MAKE_EXCEPTION

  OdomAligner(cv::HandEyeCalibrationMethod method = cv::HandEyeCalibrationMethod::CALIB_HAND_EYE_TSAI) : method_(method)
  {
  }

  /**
   * Use hand-eye calibration to align odometry.
   * \param odom_A This odometry is a spare one that we use it directly.
   * \param odom_B This odometry is a dense one that we interpolate on it.
   * \param out_T_AtoB extric parameter transforming a point from `A` to `B`
   * \param out_ref_time the time used as the frame base
   */
  void align(const ia_helic::Odometry& odom_A, const ia_helic::Odometry& odom_B, ia_helic::RigidTransform& out_T_AtoB,
             ros::Time& out_ref_time);

private:
  cv::HandEyeCalibrationMethod method_;
};
}  // namespace ia_helic
