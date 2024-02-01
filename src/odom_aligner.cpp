#include "ia_helic/odom_aligner.h"
#include <sys/wait.h>

#include <array>
#include <boost/throw_exception.hpp>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "ia_helic/odom.h"
#include "ia_helic/utils.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/traits.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/core/eigen.hpp"
#include "ros/package.h"
#include "ros/time.h"
#include "sophus/se3.hpp"

void ia_helic::OdomAligner::align(const ia_helic::Odometry& odom_A, const ia_helic::Odometry& odom_B,
                               ia_helic::RigidTransform& out_T_AtoB, ros::Time& out_ref_time)
{
  if (odom_A.poses().empty() || odom_B.poses().empty())
  {
    IA_HELIC_THROW("No odometry data!");
  }
    
  ros::Time min_t = odom_B.min_t(), max_t = odom_B.max_t();
  bool ret;

  // Consider time bias, the odometry of `from` and `to` are not synchoronized.
  size_t A_map_idx = 0;
  while (A_map_idx < odom_A.size() && odom_A.poses()[A_map_idx].t() < min_t)
  {
    A_map_idx++;
  }
  if (A_map_idx == odom_A.size())
  {
    IA_HELIC_THROW("Odometry time do not overlap.");
  }

  // choose a time to be the common reference time
  out_ref_time = odom_A.poses()[A_map_idx].t();

  // Now we use 'a0' and 'b0' to annotate the odometry beginning time of each odometry,
  // and 'r' to annotate the common reference time.
  PoseStamped T_BrtoBb0;
  std::size_t get_pose_idx = 0;
  ret = odom_B.getPose(out_ref_time, get_pose_idx, T_BrtoBb0);
  assert(ret);

  ia_helic::RigidTransform T_Bb0toBr = T_BrtoBb0.transform().inverse();
  ia_helic::RigidTransform T_Aa0toAr = odom_A.poses()[A_map_idx].getTransform().inverse();

  std::vector<cv::Mat> T_Ar_to_Ai_cv_R, T_Bi_to_Br_cv_R, T_Ar_to_Ai_cv_t, T_Bi_to_Br_cv_t;
  cv::Mat T_AtoB_R_cv, T_AtoB_t_cv;
  Eigen::Matrix3d T_AtoB_R;
  Eigen::Vector3d T_AtoB_t;

  T_Ar_to_Ai_cv_R.reserve(odom_A.size());
  T_Bi_to_Br_cv_R.reserve(odom_A.size());
  T_Ar_to_Ai_cv_t.reserve(odom_A.size());
  T_Bi_to_Br_cv_t.reserve(odom_A.size());

  std::size_t constraint_cnt = 0;
  ia_helic::PoseStamped T_Bi_to_Bb0;
  for (std::size_t i = A_map_idx; i < odom_A.size(); i++)
  {
    auto& T_Ai_to_Aa0 = odom_A.poses()[i];
    ret = odom_B.getPose(T_Ai_to_Aa0.t(), get_pose_idx, T_Bi_to_Bb0);
    if (ret)
    {
      ia_helic::RigidTransform T_Ai_to_Ar = T_Aa0toAr * T_Ai_to_Aa0.getTransform();
      ia_helic::RigidTransform T_Bi_to_Br = T_Bb0toBr * T_Bi_to_Bb0.transform();
      ia_helic::RigidTransform T_Ar_to_Ai = T_Ai_to_Ar.inverse();
      T_Ar_to_Ai_cv_R.emplace_back(3, 3, cv::DataType<double>::type);
      T_Bi_to_Br_cv_R.emplace_back(3, 3, cv::DataType<double>::type);
      T_Ar_to_Ai_cv_t.emplace_back(3, 1, cv::DataType<double>::type);
      T_Bi_to_Br_cv_t.emplace_back(3, 1, cv::DataType<double>::type);

      cv::eigen2cv(T_Ar_to_Ai.rotationMatrix(), T_Ar_to_Ai_cv_R.back());
      cv::eigen2cv(T_Bi_to_Br.rotationMatrix(), T_Bi_to_Br_cv_R.back());
      cv::eigen2cv(T_Ar_to_Ai.translation(), T_Ar_to_Ai_cv_t.back());
      cv::eigen2cv(T_Bi_to_Br.translation(), T_Bi_to_Br_cv_t.back());
      constraint_cnt++;
    }
  }
  if (constraint_cnt < 6)
  {
    IA_HELIC_THROW(fmt::format("Too few constraint! Got {}.", constraint_cnt));
  }

  cv::calibrateHandEye(T_Bi_to_Br_cv_R, T_Bi_to_Br_cv_t, T_Ar_to_Ai_cv_R, T_Ar_to_Ai_cv_t, T_AtoB_R_cv, T_AtoB_t_cv,
                       method_);
  cv::cv2eigen(T_AtoB_R_cv, T_AtoB_R);
  cv::cv2eigen(T_AtoB_t_cv, T_AtoB_t);
  out_T_AtoB.setRotationMatrix(T_AtoB_R);
  out_T_AtoB.translation() = T_AtoB_t;
}
