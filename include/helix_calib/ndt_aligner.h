#pragma once

#include <vector>
#include "helix_calib/odom.h"
#include "helix_calib/sensor.h"
#include "helix_calib/utils.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "ros/node_handle.h"

namespace helix
{
class NDTAligner
{
private:
  // parameters
  bool enabled = true;
  float voxel_size = 0.2;
  float ndt_epsilon = 0.01;
  float ndt_step_size = 0.1;
  float ndt_resolution = 1.0;
  int ndt_iterations = 100;

  pcl::CropBox<MapPoint> roi_crop_temp_;

public:
  bool is_enabled() const
  {
    return enabled;
  }

  void setROI(const std::vector<float>& max, const std::vector<float>& min)
  {
    roi_crop_temp_.setMax({ max.at(0), max.at(1), max.at(2), 1. });
    roi_crop_temp_.setMin({ min.at(0), min.at(1), min.at(2), 1. });
  }

  /**
   * \brief Aligning two point clouds.
   * \pre `io_T` contains a initial guess.
   * \note This method will align anyway, regardless enabled or not.
   * \param[in] from Source point cloud.
   * \param[in] to Target point cloud.
   * \param[in,out] io_T Transform from source to target. Refines initial guess previously stored.
   * \param[out] out_cloud Merged point cloud.
   */
  void align(const MapCloud::ConstPtr& from, const MapCloud::ConstPtr& to, RigidTransform& io_T,
             MapCloud& out_cloud) const;

  void loadParams(ros::NodeHandle& nh, const std::string& prefix);
};
}  // namespace helix
