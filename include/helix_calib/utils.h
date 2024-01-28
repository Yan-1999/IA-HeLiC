#pragma once
#define PCL_NO_PRECOMPILE

#include <cstddef>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <ios>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "boost/throw_exception.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "fmt/core.h"
#include "fmt/format.h"
#include "geometry_msgs/PoseStamped.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "ros/exception.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "sophus/se3.hpp"

namespace helix
{
class Exception : public std::runtime_error
{
public:
  Exception(const std::string& what) : std::runtime_error(what)
  {
  }
};

namespace ros_param
{
class NoROSParamException : public helix::Exception
{
public:
  NoROSParamException(const std::string& key) : helix::Exception("Could not find " + key + " parameter.")
  {
  }
};

template <typename T>
void getEssentialParam(ros::NodeHandle& nh, const std::string& key, T& param)
{
  if (!nh.getParam(key, param))
  {
    BOOST_THROW_EXCEPTION(NoROSParamException(key));
  }
}
}  // namespace ros_param

#define HELIX_MAKE_EXCEPTION                                                                                           \
  class Exception : public ::helix::Exception                                                                          \
  {                                                                                                                    \
  public:                                                                                                              \
    Exception(const std::string& what) : ::helix::Exception(what)                                                      \
    {                                                                                                                  \
    }                                                                                                                  \
  };

#define HELIX_THROW(what) BOOST_THROW_EXCEPTION(Exception(what))

/*Type of LiDARs*/
enum ELiDARType : int
{
  MECHANICAL_XYZIRT = 0,
  LIVOX_SOLID_STATE = 1,
  MECHANICAL_XYZ = 2,
};

using LiDARLabel = std::uint32_t;
enum ELiDARLabel : LiDARLabel
{
  UNKNOWN_LIDAR = std::numeric_limits<LiDARLabel>::max(),
};

struct EIGEN_ALIGN16 PointXYZIRT
{
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  std::uint16_t ring;
  double timestamp;
  PCL_MAKE_ALIGNED_OPERATOR_NEW;
};

struct EIGEN_ALIGN16 PointXYZL2T
{
  PCL_ADD_POINT4D;
  LiDARLabel label;      //< Label of LiDAR label
  std::uint32_t pindex;  //< Point label in LiDAR's raw cloud
  double t;              //< time
  PCL_MAKE_ALIGNED_OPERATOR_NEW;

  PointXYZL2T()
  {
  }

  constexpr PointXYZL2T(float x, float y, float z, LiDARLabel label, std::int32_t pindex, double t)
    : x(x), y(y), z(z), label(label), pindex(pindex), t(t)
  {
  }

  template <typename MsgT, typename PointT>
  static PointXYZL2T toXYZL2T(LiDARLabel label, const MsgT& msg, const PointT& point);
};

using RawPoint = helix::PointXYZL2T;
using RawCloud = pcl::PointCloud<RawPoint>;
using MapPoint = helix::PointXYZL2T;
using MapCloud = pcl::PointCloud<MapPoint>;
using RigidTransform = Sophus::SE3d;

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

std::ostream& printTUM(std::ostream& os, const geometry_msgs::PoseStamped& pose);
std::ostream& writeTransform(std::ostream& os, const std::string& child, const std::string& parent,
                             const helix::RigidTransform& T);

// This function is written for Eigen 3.3,
// which does not have `Eigen::Vector<>` and `Eigen::Matrix<>::begin()` and `Eigen::Matrix<>::end()`.
template <typename Scalar, int Rows>
void argsortDesc(const Eigen::Matrix<Scalar, Rows, 1>& vec, Eigen::Matrix<Scalar, Rows, 1>& sorted_vec,
                 Eigen::Matrix<int, Rows, 1>& ind)
{
  ind = Eigen::Matrix<int, Rows, 1>::LinSpaced(vec.size(), 0, vec.size() - 1);
  std::sort(ind.data(), ind.data() + ind.size(), [vec](int i, int j) { return vec(i) > vec(j); });
  for (std::size_t i = 0; i < vec.size(); i++)
  {
    sorted_vec(i) = vec(ind(i));
  }
}

}  // namespace helix

POINT_CLOUD_REGISTER_POINT_STRUCT(helix::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(helix::PointXYZL2T,
                                  (float, x, x)(float, y, y)(float, z, z)(helix::LiDARLabel, label,
                                                                          label)(std::uint32_t, pindex, pindex)(double,
                                                                                                                t, t))
