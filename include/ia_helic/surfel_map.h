#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "fmt/format.h"
#include "ia_helic/utils.h"
#include "pcl/common/common.h"
#include "pcl/common/io.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/make_shared.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pclomp/ndt_omp.h"
#include "ros/time.h"

#ifdef PCL_NO_PRECOMPILE
#include "pcl/common/impl/common.hpp"
#include "pcl/filters/impl/crop_box.hpp"
#endif

namespace ia_helic
{
struct Surfel
{
  Surfel(const MapCloud::Ptr& cloud, const MapCloud::Ptr& cloud_inlier, Eigen::Vector4d& coeff)
    : cloud(cloud), cloud_inlier(cloud_inlier), coeff(coeff), Pi(-coeff(3) * coeff.head<3>())
  {
    MapPoint min, max;
    pcl::getMinMax3D(*cloud, min, max);
    box_min = min.getVector3fMap().cast<double>();
    box_max = max.getVector3fMap().cast<double>();
  }

  MapCloud::Ptr cloud;
  MapCloud::Ptr cloud_inlier;
  Eigen::Vector4d coeff;
  Eigen::Vector3d Pi;  //< closest point paramization, [-d * n_x, -d * n_y, -d * n_z]
  Eigen::Vector3d box_min;
  Eigen::Vector3d box_max;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SurfelPoint
{
  MapPoint raw_point;
  MapPoint map_point;
  LiDARLabel lidar_label;
  std::size_t plane_id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SurfelMap
{
public:
  IA_HELIC_MAKE_EXCEPTION

  using ColoredPoint = pcl::PointXYZRGB;
  using ColoredCloud = pcl::PointCloud<ColoredPoint>;
  using Ptr = std::shared_ptr<SurfelMap>;
  using ConstPtr = std::shared_ptr<const SurfelMap>;

private:
  static constexpr int MIN_POINTS_IN_CELL = 10;

  LiDARLabel label_;

  ros::Time map_time_;
  AlignedVector<Surfel> surfels_;
  AlignedVector<AlignedVector<SurfelPoint>> spoints_in_surfel_;
  AlignedVector<SurfelPoint> spoints_;
  AlignedVector<SurfelPoint> spoints_down_sampled_;
  pcl::CropBox<MapPoint> roi_crop_;

  pcl::KdTreeFLANN<MapPoint> kdtree_spoints_down_sampled_;

  // parameters
  float surfel_size_ = 0.5;
  double associated_radius_ = 0.05;
  double plane_lambda_ = 0.8;

public:
  SurfelMap(LiDARLabel label)
    :label_(label)
  {
  }

  SurfelMap()
    :SurfelMap(UNKNOWN_LIDAR)
  {
  }

  auto& surfels()
  {
    return surfels_;
  }

  const auto& surfels() const
  {
    return surfels_;
  }

  const auto& spoints() const
  {
    return spoints_;
  }

  const auto& spoints_down_sampled() const
  {
    return spoints_down_sampled_;
  }

  void setLabel(LiDARLabel label)
  {
    label_ = label;
  }

  LiDARLabel label() const
  {
    return label_;
  }

  ros::Time& map_time()
  {
    return map_time_;
  }

  const ros::Time& map_time() const
  {
    return map_time_;
  }

  void clear()
  {
    spoints_.clear();
    spoints_in_surfel_.clear();
    surfels_.clear();
    spoints_down_sampled_.clear();
  }

  void setSurfelSize(double surfel_size)
  {
    surfel_size_ = surfel_size;
  }

  double surfel_size() const
  {
    return surfel_size_;
  }

  void setAssociateRadius(double associate_radius)
  {
    associated_radius_ = associate_radius;
  }

  double associate_radius() const
  {
    return associated_radius_;
  }

  void setROIMin(const std::vector<float>& roi_min)
  {
    roi_crop_.setMin({ roi_min[0], roi_min[1], roi_min[2], 1 });
  }

  void setROIMax(const std::vector<float>& roi_max)
  {
    roi_crop_.setMax({ roi_max[0], roi_max[1], roi_max[2], 1 });
  }

  /**
   * \brief Build surfel map.
   * \param[in] cloud Map cloud.
   * \param[in] map_time Map time.
   */
  void build(const MapCloud::Ptr& cloud, ros::Time map_time)
  {
    if (cloud->empty())
    {
      IA_HELIC_THROW("Input cloud is empty!");
    }

    auto cloud_in_roi = pcl::make_shared<MapCloud>();
    roi_crop_.setInputCloud(cloud);
    roi_crop_.filter(*cloud_in_roi);

    pclomp::NormalDistributionsTransform<MapPoint, MapPoint> ndt;
    ndt.setResolution(surfel_size_);
    ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt.setTransformationEpsilon(1e-3);
    ndt.setStepSize(0.01);
    ndt.setMaximumIterations(50);
    ndt.setInputTarget(cloud_in_roi);
    build(ndt, map_time);
  }

  /**
   * \brief Associate cloud points to surfels.
   * \pre `build()` is called.
   * \param[in] kd_tree KD-Tree used for map cloud.
   * \param[in] raw_clouds Points in LiDAR local frames.
   */
  void associateMultipleClouds(const pcl::KdTreeFLANN<MapPoint>& kd_tree,
                               const AlignedVector<RawCloud::ConstPtr>& raw_clouds);

  /**
   * \brief Associate cloud points to surfels.
   * \note This function will only use `pindex` as index of `raw_cloud` and ignore `label` field.
   * \pre `build()` is called.
   * \param[in] kd_tree KD-Tree used for map cloud.
   * \param[in] raw_cloud Points in LiDAR local frames.
   */
  void associateCloud(const pcl::KdTreeFLANN<MapPoint>& kd_tree, const RawCloud::ConstPtr raw_cloud);

  /**
   * \brief Associate surfel inliers.
   * \note This function will only use `pindex` as index of `raw_cloud` and ignore `label` field.
   * \pre `build()` is called.
   * \param[in] raw_cloud Points in LiDAR local frames.
   */
  void associateInliers(const RawCloud::ConstPtr raw_cloud);

  /**
   * \brief Down sample surfel point cloud by picking 1 surfel point in every `rate` points (not randomly).
   * \pre `associate()` is called.
   * \param[in] rate Down sample rate.
   */
  void downSampleSimple(int rate);

  /**
   * \brief Down sample sufel point cloud by farthest point sampling (FPS) in points aggregated of each LiDAR in every
   * given time span.
   * \pre `associate()` is called.
   * \param[in] rate Down sample rate.
   * \param[in] n_lidar Number of LiDARs.
   * \param[in] time_span Time span.
   */
  void downSampleFarthest(int rate, std::size_t n_lidar, double time_span = 0.5);

  /**
   * \brief Color surfels for debug purpose.
   * \pre `build()` is called.
   * \param[out] cloud Output PointXYZRGB cloud.
   */
  void getColoredSurfels(ColoredCloud& cloud) const;

  void getDownSampledSurfelPoints(MapCloud& map_cloud) const
  {
    map_cloud.resize(spoints_down_sampled_.size());
    std::transform(spoints_down_sampled_.begin(), spoints_down_sampled_.end(), map_cloud.begin(),
                   [](const SurfelPoint& sp) { return sp.map_point; });
  }

  const pcl::KdTreeFLANN<MapPoint>& kdtree_spoints_down_sampled() const
  {
    return kdtree_spoints_down_sampled_;
  }

  void buildDownSampledSurfelPointsKDTree()
  {
    auto map_points = pcl::make_shared<MapCloud>();
    map_points->resize(spoints_down_sampled_.size());
    getDownSampledSurfelPoints(*map_points);
    kdtree_spoints_down_sampled_.setInputCloud(map_points);
  }

private:
  void build(const pclomp::NormalDistributionsTransform<MapPoint, MapPoint>& ndt, ros::Time map_time);
};

/**
 * Associating surfel points with surfels of other LiDAR.
 */
class CrossSurfelMap
{
public:
  using Ptr = std::shared_ptr<CrossSurfelMap>;
  using ConstPtr = std::shared_ptr<const CrossSurfelMap>;
  using ColoredPoint = pcl::PointXYZRGB;
  using ColoredCloud = pcl::PointCloud<ColoredPoint>;

  IA_HELIC_MAKE_EXCEPTION

private:
  double associated_radius_ = 0.1;

  LiDARLabel spoint_label_;
  LiDARLabel surfel_label_;
  SurfelMap::ConstPtr surfel_map_;
  AlignedVector<SurfelPoint> spoints_;

public:
  CrossSurfelMap(LiDARLabel spoint_label, LiDARLabel surfel_label, const SurfelMap::ConstPtr surfel_map)
    : spoint_label_(spoint_label), surfel_label_(surfel_label), surfel_map_(surfel_map)
  {
  }

  LiDARLabel spoint_label() const
  {
    return spoint_label_;
  }

  LiDARLabel surfel_label() const
  {
    return surfel_label_;
  }

  ros::Time map_time() const
  {
    return surfel_map_->map_time();
  }

  const AlignedVector<SurfelPoint>& spoints() const
  {
    return spoints_;
  }

  const AlignedVector<Surfel>& surfels() const
  {
    return surfel_map_->surfels();
  }

  void associate(const pcl::KdTreeFLANN<MapPoint>& kd_tree, const RigidTransform& T_spoint_to_surfel,
                 const MapCloud& raw_cloud);

  void getColoredSurfelPoints(ColoredCloud& cloud) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ia_helic
