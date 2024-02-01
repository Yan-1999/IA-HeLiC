#include <array>
#include <cstddef>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ia_helic/ndt_aligner.h"
#include "ia_helic/utils.h"
#include "omp.h"
#include "pcl/common/io.h"
#include "pcl/make_shared.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pclomp/ndt_omp.h"

#ifdef PCL_NO_PRECOMPILE
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/filters/impl/voxel_grid.hpp"
#include "pcl/impl/pcl_base.hpp"
#endif

void ia_helic::NDTAligner::loadParams(ros::NodeHandle& nh, const std::string& prefix)
{
  nh.param(prefix + "/enable", enabled, true);
  nh.param(prefix + "/voxel_size", voxel_size, 0.2f);
  nh.param(prefix + "/ndt_epsilon", ndt_epsilon, 0.01f);
  nh.param(prefix + "/ndt_step_size", ndt_step_size, 0.1f);
  nh.param(prefix + "/ndt_resolution", ndt_resolution, 1.0f);
  nh.param(prefix + "/ndt_iterations", ndt_iterations, 100);
}

void ia_helic::NDTAligner::align(const MapCloud::ConstPtr& from, const MapCloud::ConstPtr& to, RigidTransform& io_T,
                              MapCloud& out_cloud) const
{
  std::array<MapCloud::ConstPtr, 2> input_clouds = { from, to };
  std::array<MapCloud::Ptr, 2> down_input_clouds;

#pragma omp parallel for
  for (std::size_t i = 0; i < 2; i++)
  {
    down_input_clouds[i] = pcl::make_shared<MapCloud>();

    auto roi_crop = roi_crop_temp_;
    auto in_roi = pcl::make_shared<MapCloud>();
    roi_crop.setInputCloud(input_clouds[i]);
    roi_crop.filter(*in_roi);

    pcl::VoxelGrid<MapPoint> voxel_filter;
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.setMinimumPointsNumberPerVoxel(5);
    voxel_filter.setInputCloud(in_roi);
    voxel_filter.filter(*(down_input_clouds[i]));
  }
  auto down_from = down_input_clouds[0];
  auto down_to = down_input_clouds[1];

  pclomp::NormalDistributionsTransform<MapPoint, MapPoint> ndt_aligner;

  ndt_aligner.setResolution(ndt_resolution);
  ndt_aligner.setNeighborhoodSearchMethod(pclomp::KDTREE);
  ndt_aligner.setTransformationEpsilon(ndt_epsilon);
  ndt_aligner.setStepSize(ndt_step_size);
  ndt_aligner.setMaximumIterations(ndt_iterations);
  ndt_aligner.setInputSource(down_from);
  ndt_aligner.setInputTarget(down_to);
  ndt_aligner.align(out_cloud, io_T.matrix().cast<float>());
  out_cloud += *down_to;
  auto T = ndt_aligner.getFinalTransformation();
  io_T = Sophus::SE3f::fitToSE3(T).cast<double>();
}
