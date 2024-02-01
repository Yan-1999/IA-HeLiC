#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "fmt/format.h"
#include "geometry_msgs/PoseStamped.h"
#include "ia_helic/utils.h"
#include "ros/time.h"

namespace ia_helic
{
class PoseStamped
{
protected:
  ros::Time t_;
  ia_helic::RigidTransform transform_;

public:
  PoseStamped()
  {
  }

  PoseStamped(double t, double x, double y, double z, double qx, double qy, double qz, double qw)
    : t_(t), transform_(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(x, y, z))
  {
  }

  PoseStamped(double t, const Eigen::Quaterniond& q, const Eigen::Vector3d& v) : t_(t), transform_(q, v)
  {
  }

  ros::Time& t()
  {
    return t_;
  };

  ros::Time t() const
  {
    return t_;
  };

  RigidTransform& transform()
  {
    return transform_;
  }

  const RigidTransform& getTransform() const
  {
    return transform_;
  }

  Eigen::Quaterniond orientation() const
  {
    return transform_.unit_quaternion();
  }

  Eigen::Vector3d position() const
  {
    return transform_.translation();
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Odometry
{
public:
  IA_HELIC_MAKE_EXCEPTION

private:
  bool is_sorted_;
  ia_helic::AlignedVector<PoseStamped> poses_;

public:
  void emplace_back(double t, double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    is_sorted_ = false;
    poses_.emplace_back(t, x, y, z, qx, qy, qz, qw);
  };

  void emplace_back(double t, const Eigen::Quaterniond& q, const Eigen::Vector3d& v)
  {
    is_sorted_ = false;
    poses_.emplace_back(t, q, v);
  };

  void push_back(const PoseStamped& pos)
  {
    is_sorted_ = false;
    poses_.push_back(pos);
  }

  std::size_t size() const
  {
    return poses_.size();
  }

  bool empty() const
  {
    return poses_.empty();
  }

  void sort()
  {
    std::sort(poses_.begin(), poses_.end(),
              [](const PoseStamped& lhs, const PoseStamped& rhs) { return lhs.t() < rhs.t(); });
    is_sorted_ = true;
  }

  ia_helic::AlignedVector<PoseStamped>& poses()
  {
    is_sorted_ = false;
    return poses_;
  }

  const ia_helic::AlignedVector<PoseStamped>& poses() const
  {
    return poses_;
  }

  bool is_sorted() const
  {
    return is_sorted_;
  }

  ros::Time min_t() const
  {
    assert(is_sorted());
    return poses_.front().t();
  }

  ros::Time max_t() const
  {
    assert(is_sorted());
    return poses_.back().t();
  }

  static void interpPose(ros::Time t, const PoseStamped& lhs, const PoseStamped& rhs, PoseStamped& out,
                         double t_tolerance = 1e-5)
  {
    assert(lhs.t() <= t && rhs.t() >= t);
    if ((t - lhs.t()).toSec() <= t_tolerance)
    {
      out = lhs;
    }
    else if ((rhs.t() - t).toSec() <= t_tolerance)
    {
      out = rhs;
    }
    else
    {
      double diff_t = (rhs.t() - lhs.t()).toSec();
      double k = (t - lhs.t()).toSec() / diff_t;
      out.transform() =
          lhs.getTransform() * RigidTransform::exp(k * (lhs.getTransform().inverse() * rhs.getTransform()).log());
      out.t() = t;
    }
  }

  [[nodiscard]] bool getPose(ros::Time t, std::size_t& index, PoseStamped& out_pose) const
  {
    assert(is_sorted());
    std::size_t size = poses_.size();

    if (index >= size - 1 || t < poses_[index].t() || t > max_t())
    {
      return false;
    }

    while (poses_[index + 1].t() < t)
    {
      index++;
    }

    interpPose(t, poses_[index], poses_[index + 1], out_pose);

    return true;
  }

  bool getPose(ros::Time t, PoseStamped& out_pose) const
  {
    if (t < min_t())
    {
      return false;
    }
    auto it = std::lower_bound(poses_.begin(), poses_.end(), t,
                               [](const PoseStamped& pose, ros::Time t) { return pose.t() < t; });
    if (it == poses_.end())
    {
      return false;
    }
    interpPose(t, *(it - 1), *it, out_pose);
    return true;
  }

  void loadTUM(const std::string& filepath);
  void dumpTUM(const std::string& filepath);
};
}  // namespace ia_helic
