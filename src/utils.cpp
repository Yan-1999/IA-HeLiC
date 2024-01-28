#include "helix_calib/utils.h"

#include <cstddef>

#include "livox_ros_driver/CustomMsg.h"
#include "livox_ros_driver/CustomPoint.h"
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"

template <>
helix::PointXYZL2T helix::PointXYZL2T::toXYZL2T<sensor_msgs::PointCloud2, pcl::PointXYZ>(
    LiDARLabel label, const sensor_msgs::PointCloud2& msg, const pcl::PointXYZ& point)
{
  return PointXYZL2T(point.x, point.y, point.z, label, 0, msg.header.stamp.toSec());
}

template <>
helix::PointXYZL2T helix::PointXYZL2T::toXYZL2T<sensor_msgs::PointCloud2, helix::PointXYZIRT>(
    LiDARLabel label, const sensor_msgs::PointCloud2& msg, const helix::PointXYZIRT& point)
{
  return PointXYZL2T(point.x, point.y, point.z, label, 0, point.timestamp);
}

template <>
helix::PointXYZL2T helix::PointXYZL2T::toXYZL2T<livox_ros_driver::CustomMsg, livox_ros_driver::CustomPoint>(
    LiDARLabel label, const livox_ros_driver::CustomMsg& msg, const livox_ros_driver::CustomPoint& point)
{
  double t_base =
      static_cast<double>(msg.timebase / 1'000'000'000) + static_cast<double>(msg.timebase % 1'000'000'000) * 1e-9f;
  return PointXYZL2T(point.x, point.y, point.z, label, 0, static_cast<double>(point.offset_time) * 1e-9f + t_base);
}

std::ostream& helix::printTUM(std::ostream& os, const geometry_msgs::PoseStamped& pose)
{
  using namespace fmt::literals;
  ros::Time stamp = pose.header.stamp;
  auto& position = pose.pose.position;
  auto& orientation = pose.pose.orientation;
  fmt::print("{sec:d}.{nsec:09d} {px} {py} {pz} {qx} {qy} {qz} {qw}\n", "sec"_a = stamp.sec, "nsec"_a = stamp.nsec,
             "px"_a = position.x, "py"_a = position.y, "pz"_a = position.z, "qx"_a = orientation.x,
             "qy"_a = orientation.y, "qz"_a = orientation.z, "qw"_a = orientation.w);
  return os;
}

std::ostream& helix::writeTransform(std::ostream& os, const std::string& child, const std::string& parent,
                                    const helix::RigidTransform& T)
{
  fmt::print(os,
             "{child}:\n"
             "  extrinsics:\n"
             "#   rotX: {rotX}deg, rotY: {rotY}deg, rotZ: {rotZ}deg\n"
             "    quaternion: [{qw}, {qx}, {qy}, {qz}] # [w, x, y, z]\n"
             "    translation: [{x}, {y}, {z}]\n"
             "  parent: {parent}\n",
             fmt::arg("child", child), fmt::arg("parent", parent), fmt::arg("rotX", T.angleX() * 180 / M_PI),
             fmt::arg("rotY", T.angleY() * 180 / M_PI), fmt::arg("rotZ", T.angleZ() * 180 / M_PI),
             fmt::arg("qw", T.unit_quaternion().w()), fmt::arg("qx", T.unit_quaternion().x()),
             fmt::arg("qy", T.unit_quaternion().y()), fmt::arg("qz", T.unit_quaternion().z()),
             fmt::arg("x", T.translation().x()), fmt::arg("y", T.translation().y()),
             fmt::arg("z", T.translation().z()));
  return os;
}
