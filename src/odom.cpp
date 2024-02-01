#include "ia_helic/odom.h"

#include <cstddef>
#include <fstream>
#include <ios>
#include <sstream>
#include <stdexcept>
#include <string>

#include "fmt/format.h"
#include "ia_helic/utils.h"
#include "ros/console.h"

void ia_helic::Odometry::loadTUM(const std::string& filepath)
{
  std::string buf;
  std::size_t pose_cnt = 0;
  double t, x, y, z, qx, qy, qz, qw;
  std::fstream fs(filepath, std::ios_base::in);

  if (!fs.is_open())
  {
    IA_HELIC_THROW("Cannot open file: " + filepath);
  }

  ROS_INFO("Open TUM File: %s.", filepath.c_str());

  poses_.clear();
  while (std::getline(fs, buf))
  {
    if (buf.front() == '#')  // this is a TUM comment
    {
      continue;
    }
    std::cout << " Loading pose: \e[1m" << pose_cnt++ << "\e[0m from TUM." << '\r' << std::flush;
    std::stringstream ss(buf);
    ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw;
    emplace_back(t, x, y, z, qx, qy, qz, qw);
  }
  sort();
  ROS_INFO("Loaded from: %s, frames: %zu, time span: %.9f -> %.9f", filepath.c_str(), pose_cnt, min_t().toSec(),
           max_t().toSec());
}

void ia_helic::Odometry::dumpTUM(const std::string& filepath)
{
  using namespace fmt::literals;
  std::fstream fs(filepath, std::ios_base::out | std::ios_base::trunc);

  if (!fs.is_open())
  {
    IA_HELIC_THROW("Cannot open file: " + filepath);
  }

  for (const auto& pose : poses_)
  {
    ros::Time stamp = pose.t();
    auto position = pose.position();
    auto orientation = pose.orientation();
    fmt::print(fs, "{sec:d}.{nsec:09d} {px} {py} {pz} {qx} {qy} {qz} {qw}\n", "sec"_a = stamp.sec,
               "nsec"_a = stamp.nsec, "px"_a = position.x(), "py"_a = position.y(), "pz"_a = position.z(),
               "qx"_a = orientation.x(), "qy"_a = orientation.y(), "qz"_a = orientation.z(), "qw"_a = orientation.w());
  }
}