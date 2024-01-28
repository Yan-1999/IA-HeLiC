#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <unordered_set>

#include "fmt/format.h"
#include "nav_msgs/Path.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "ros/time.h"

std::fstream fs;
std::unordered_set<std::uint64_t> time_set;

/* From utils.h */
class NoROSParamException : public std::runtime_error
{
public:
  NoROSParamException(const std::string& key) : std::runtime_error("Could not find " + key + " parameter.")
  {
  }
};

template <typename T>
void getEssentialParam(ros::NodeHandle& nh, const std::string& key, T& param)
{
  if (!nh.getParam(key, param))
  {
    throw NoROSParamException(key);
  }
}

class FileOpenFailure : public std::runtime_error
{
public:
  FileOpenFailure(const std::string& file_path) : std::runtime_error("Could not open file: " + file_path + ".")
  {
  }
};

void printTUM(std::FILE* fp, const geometry_msgs::PoseStamped& pose)
{
  using namespace fmt::literals;
  ros::Time stamp = pose.header.stamp;
  auto& position = pose.pose.position;
  auto& orientation = pose.pose.orientation;
  fmt::println(fp, "{sec:d}.{nsec:09d} {px} {py} {pz} {qx} {qy} {qz} {qw}", "sec"_a = stamp.sec, "nsec"_a = stamp.nsec,
               "px"_a = position.x, "py"_a = position.y, "pz"_a = position.z, "qx"_a = orientation.x,
               "qy"_a = orientation.y, "qz"_a = orientation.z, "qw"_a = orientation.w);
}

void pathCallback(std::FILE* fp, const nav_msgs::Path::ConstPtr& msg)
{
  for (auto& pose : msg->poses)
  {
    std::uint64_t t = pose.header.stamp.toNSec();
    if (time_set.insert(t).second)
    {
      printTUM(fp, pose);
    }
  }
}

void SigHandle(int sig)
{
  fs.close();
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_helper");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(1);

  signal(SIGINT, SigHandle);

  try
  {
    std::string path_topic, result_path;
    getEssentialParam(nh, "path_topic", path_topic);
    getEssentialParam(nh, "result_path", result_path);

    std::FILE* fp = std::fopen(result_path.c_str(), "w");
    if (!fp)
    {
      throw FileOpenFailure(result_path);
    }

    ros::Subscriber subPath =
        nh.subscribe<nav_msgs::Path>(path_topic, 256, std::bind(pathCallback, fp, std::placeholders::_1));

    while (ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    return EXIT_FAILURE;
  }
}
