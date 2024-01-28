#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "boost/exception/diagnostic_information.hpp"
#include "boost/throw_exception.hpp"
#include "fmt/core.h"
#include "helix_calib/calib_params.h"
#include "helix_calib/ndt_aligner.h"
#include "helix_calib/sensor.h"
#include "helix_calib/sensor_system.h"
#include "helix_calib/surfel_map.h"
#include "helix_calib/utils.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "ros/init.h"
#include "ros/package.h"

template <typename PointT>
void writePCD(const std::string& out_path, const typename pcl::PointCloud<PointT>::ConstPtr cloud)
{
  ROS_INFO("Writting PCD File: %s...", out_path.c_str());
  pcl::io::savePCDFile(out_path, *cloud);
  ROS_INFO("Written PCD file: %s.", out_path.c_str());
}

void writeSurfelMapPCD(const std::string& out_path, const helix::SurfelMap::ConstPtr map)
{
  helix::SurfelMap::ColoredCloud cloud;
  map->getColoredSurfels(cloud);
  ROS_INFO("Writting PCD File: %s...", out_path.c_str());
  pcl::io::savePCDFile(out_path, cloud);
  ROS_INFO("Written PCD file: %s.", out_path.c_str());
}

void writeSurfelPointsPCD(const std::string& out_path, const helix::SurfelMap::ConstPtr map)
{
  helix::MapCloud cloud;
  map->getDownSampledSurfelPoints(cloud);
  ROS_INFO("Writting PCD File: %s...", out_path.c_str());
  pcl::io::savePCDFile(out_path, cloud);
  ROS_INFO("Written PCD file: %s.", out_path.c_str());
}

void writeCrossSurfelPointsPCD(const std::string& out_path, const helix::CrossSurfelMap::ConstPtr map)
{
  helix::CrossSurfelMap::ColoredCloud cloud;
  map->getColoredSurfelPoints(cloud);
  ROS_INFO("Writting PCD File: %s...", out_path.c_str());
  pcl::io::savePCDFile(out_path, cloud);
  ROS_INFO("Written PCD file: %s.", out_path.c_str());
}

int main(int argc, char** argv)
{
  using namespace std::string_literals;
  using namespace helix::ros_param;
  using namespace fmt::literals;

  ros::init(argc, argv, "calib_node");
  ros::NodeHandle nh("~");

  std::filesystem::path this_package = ros::package::getPath("helix_calib");
  std::vector<helix::CalibParams> params_vec;
  helix::NDTAligner ndt;
  std::vector<std::thread> pcd_threads;

  try
  {
    /* Init Params */
    std::vector<int> lidar_types;
    getEssentialParam(nh, "lidar_types", lidar_types);
    if (lidar_types.empty())
    {
      BOOST_THROW_EXCEPTION(helix::Exception("No LiDAR!"));
    }
    helix::SensorSystem sys(lidar_types.size());

    std::string calib_file_path =
        nh.param<std::string>("calib_file_path", this_package / "result" / "calib_result_iter{iter}.yml");
    double imu_acc_scale_by = nh.param("imu_acc_scale_by", 1);

    auto roi_min = nh.param<std::vector<float>>("ROI/min", { -256, -256, -256 });
    auto roi_max = nh.param<std::vector<float>>("ROI/max", { 256, 256, 256 });

    bool skip_opt = nh.param("optimization/skip", false);
    double surfel_size = nh.param("optimization/surfel_size", 0.5);
    int down_sample_rate = nh.param("optimization/down_sample_rate", 200);
    double kont_distance = nh.param("optimization/kont_distance", 0.2);
    std::size_t iterations = nh.param("optimization/iterations", 3);
    std::size_t solver_max_iterations = nh.param("optimization/solver_max_iterations", 50);
    double time_padding = nh.param("optimization/time_padding", 0.15);
    double acc_weight = nh.param("optimization/acc_weight", 18.5);
    double gyro_weight = nh.param("optimization/gyro_weight", 28.0);
    double lidar_weight = nh.param("optimization/lidar_weight", 10.0);
    bool wo_imu = nh.param("optimization/wo_imu", false);
    bool wo_cross_surfel = nh.param("optimization/wo_cross_surfel", false);
    bool wo_local_surfel = nh.param("optimization/wo_local_surfel", false);

    std::string imu_trajectory_path = nh.param("debug/imu_trajectory", ""s);
    std::string surfel_map_path = nh.param("debug/surfel_maps", ""s);
    std::string surfel_points_path = nh.param("debug/surfel_points", ""s);
    std::string cross_surfel_points_path = nh.param("debug/cross_surfel_points", ""s);
    std::string local_map_path = nh.param("debug/local_maps", ""s);
    std::string global_map_path = nh.param("debug/global_maps", ""s);
    std::string errors_path = nh.param<std::string>("debug/errors_path", ""s);

    std::string bag_path, imu_topic;
    std::vector<std::string> lidar_odoms, lidar_topics;

    getEssentialParam(nh, "lidar_types", lidar_types);
    getEssentialParam(nh, "lidar_odoms", lidar_odoms);
    getEssentialParam(nh, "bag_path", bag_path);
    getEssentialParam(nh, "topics/lidars", lidar_topics);
    getEssentialParam(nh, "topics/imu", imu_topic);

    if (roi_max.size() < 3 && roi_min.size() < 3)
    {
      BOOST_THROW_EXCEPTION(helix::Exception("ROI number of dimentions is less than 3!"));
    }

    int64_t dx = static_cast<int64_t>((roi_max[0] - roi_min[0]) / surfel_size) + 1;
    int64_t dy = static_cast<int64_t>((roi_max[1] - roi_min[1]) / surfel_size) + 1;
    int64_t dz = static_cast<int64_t>((roi_max[2] - roi_min[2]) / surfel_size) + 1;
    if (dx * dy * dz > std::numeric_limits<std::int32_t>::max())
    {
      ROS_WARN("ROI too big, surfel building may fail!");
    }

    /* Setting Params */
    ndt.loadParams(nh, "ndt_align");
    ndt.setROI(roi_max, roi_min);
    sys.setLiDARTypes(lidar_types);
    sys.imu().setAccScaling(imu_acc_scale_by);
    sys.trajectory().setMaxIterations(solver_max_iterations);
    sys.trajectory().setAccWeight(acc_weight);
    sys.trajectory().setGyroWeight(gyro_weight);
    sys.trajectory().setLidarWeight(lidar_weight);

    /* Pre-processing */
    ROS_INFO("===Data Reading Step===");
    sys.loadTUMs(lidar_odoms);
    sys.loadROSBag(bag_path, lidar_topics, imu_topic);

    ROS_INFO("===Odometry Aligning Step===");
    ROS_INFO("Aligning Odometry...");
    sys.alignOdom();
    ROS_INFO("Get Frame Transform: ");
    sys.writeCalibResults(std::cout);
    std::string calib_file_name = fmt::format(calib_file_path, "iter"_a = "Init");
    std::fstream calib_file(calib_file_name, std::ios_base::out);
    if (calib_file.is_open())
    {
      sys.writeCalibResults(calib_file);
      calib_file.close();
      ROS_INFO("Written to file: %s", calib_file_name.c_str());
    }

    ROS_INFO("===Trajectory Initializing Step===");
    sys.initSystemTrajectoryIMU(kont_distance, time_padding);
    ROS_INFO("Get Frame Transform: ");
    sys.writeCalibResults(std::cout);

    /* Refinement */
    bool use_trajectory = false;
    bool enable_ndt_align = ndt.is_enabled();
    iterations = skip_opt ? 1 : iterations;
    for (std::size_t iteration = 0; iteration < iterations; iteration++)
    {
      ROS_INFO("===Iteration: %zu===", iteration);
      ROS_INFO("Build local maps using %s...", use_trajectory ? "refined trajectory" : "odometry");
      sys.buildLocalCloudMaps(use_trajectory);
      if (!local_map_path.empty())
      {
        for (std::size_t i = 0; i < sys.lidars().size(); i++)
        {
          std::string out_path = fmt::format(local_map_path, "iter"_a = iteration, "label"_a = i);
          pcd_threads.emplace_back(writePCD<helix::MapPoint>, out_path, sys.lidars()[i].local_map());
        }
      }
      if (enable_ndt_align)
      {
        ROS_INFO("===NDT Aligning Step===");
        sys.alignCloudNDT(ndt);
        ROS_INFO("Get Frame Transform: ");
        sys.writeCalibResults(std::cout);
        std::string calib_file_name = fmt::format(calib_file_path, "iter"_a = "NDT");
        std::fstream calib_file(calib_file_name, std::ios_base::out);
        if (calib_file.is_open())
        {
          sys.writeCalibResults(calib_file);
          calib_file.close();
          ROS_INFO("Written to file: %s", calib_file_name.c_str());
        }
      }
      else
      {
        ROS_INFO("Skip NDT.");
      }

      if (skip_opt)
      {
        ROS_INFO("Optimization Skipped.");
      }
      else
      {
        ROS_INFO("===Surfel Map Building Step===");
        sys.buildLocalSurfelMaps(roi_max, roi_min, surfel_size, down_sample_rate);
        if (!wo_cross_surfel)
        {
          sys.associateCrossSurfelPoints();
        }
        if (!surfel_map_path.empty())
        {
          for (std::size_t i = 0; i < sys.lidars().size(); i++)
          {
            std::string out_path = fmt::format(surfel_map_path, "iter"_a = iteration, "label"_a = i);
            pcd_threads.emplace_back(writeSurfelMapPCD, out_path, sys.lidars()[i].local_surfel_map());
          }
        }
        if (!surfel_points_path.empty())
        {
          for (std::size_t i = 0; i < sys.lidars().size(); i++)
          {
            std::string out_path = fmt::format(surfel_points_path, "iter"_a = iteration, "label"_a = i);
            pcd_threads.emplace_back(writeSurfelPointsPCD, out_path, sys.lidars()[i].local_surfel_map());
          }
        }
        if (!wo_cross_surfel && !cross_surfel_points_path.empty())
        {
          for (const auto& p_cross_surfel_map : sys.cross_surfel_maps())
          {
            if (p_cross_surfel_map)
            {
              std::string out_path = fmt::format(cross_surfel_points_path, "iter"_a = iteration,
                                                 "label_spoint"_a = p_cross_surfel_map->spoint_label(),
                                                 "label_surfel"_a = p_cross_surfel_map->surfel_label());
              pcd_threads.emplace_back(writeCrossSurfelPointsPCD, out_path, p_cross_surfel_map);
            }
          }
        }

        ROS_INFO("===Optimization Step===");
        sys.refineTrajectory(wo_imu, wo_cross_surfel, wo_local_surfel);
        sys.writeCalibResults(std::cout);
        if (!imu_trajectory_path.empty())
        {
          sys.trajectory().dumpTUM(fmt::format(imu_trajectory_path, "iter"_a = iteration));
        }

        std::string calib_file_name = fmt::format(calib_file_path, "iter"_a = iteration);
        std::fstream calib_file(calib_file_name, std::ios_base::out);
        if (calib_file.is_open())
        {
          sys.writeCalibResults(calib_file);
          calib_file.close();
          ROS_INFO("Written to file: %s", calib_file_name.c_str());
        }
        sys.trajectory().reportErrors(errors_path, iteration);
      }

      // after optmize
      enable_ndt_align = false;
      use_trajectory = true;
      params_vec.push_back(sys.params());
      if (!global_map_path.empty())
      {
        ROS_INFO("Building Global Map...");
        sys.buildGlobalCloudMap(true);
        std::string out_path = fmt::format(global_map_path, "iter"_a = iteration);
        pcd_threads.emplace_back(writePCD<helix::MapPoint>, out_path, sys.cloud_map());
      }
      sys.clearMaps();
    }

    ROS_INFO("Wating for PCD Writting threads...");
    for (auto& thrd : pcd_threads)
    {
      thrd.join();
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("" << boost::diagnostic_information(e));
    return EXIT_FAILURE;
  }
  return 0;
}
